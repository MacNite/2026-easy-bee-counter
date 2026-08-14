// ============================================================================
// HiveTraffic bee counter — ESP32-C6 mini firmware
// ----------------------------------------------------------------------------
// Author: rewritten 2026 for the ESP32-C6 mini board.
//
// 2026-08 hardware revision: the IR emitters are split across THREE MOSFET
// banks instead of two — one IRLB8721 per MCP23017, driven from silk D8/D9/D10
// (GPIO19/GPIO20/GPIO18). See pins.h for the full map.
//
// 2026-06 power revision: the IR emitters are PULSED instead of left on
// continuously. Each poll turns the emitter banks on only for the brief window
// needed to settle the phototransistors and read the MCP23017s, then turns
// them off again. At the default POLL_INTERVAL_MS=5 / LED_SETTLE_US=250 with a
// ~1.5 ms three-chip read, the emitter duty cycle drops from 100% to roughly
// 35%, and raising POLL_INTERVAL_MS lowers it proportionally. See the
// "Pulsed-LED sampling" section below.
//
// What this firmware does
// -----------------------
//   1. Continuously polls 24 entrance gates (each gate = Inner IR sensor +
//      Outer IR sensor) through three MCP23017 I2C port expanders on Wire.
//   2. Detects directional bee crossings:
//        Outer-then-Inner = bee entered the hive   -> "in" counter++
//        Inner-then-Outer = bee left the hive      -> "out" counter++
//   3. Maintains LIFETIME totals, and serves them over BLE/GATT to HiveHub,
//      which differences consecutive reads into per-interval counts.
//   4. Accepts a firmware image over the same GATT service (see ble_link.cpp),
//      verifying size and CRC-32 before swapping app slots.
//
// The wired HiveScale link is gone
// --------------------------------
// Earlier revisions ran the C6's second I2C controller as a permanent slave at
// 0x30, serving a register map (lifetime totals, per-interval counters reset by
// CMD_LATCH, per-gate arrays, and an OTA-over-I2C block) to a HiveScale polling
// over J1. All of it has been removed: HiveHub reads counters over BLE only and
// dropped its wired client, so nothing spoke that protocol anymore.
//
// What that removal simplifies, and why the code no longer mentions it:
//   - No slave callbacks, no register pointer, no ISR-context response buffer.
//   - No interval or per-gate counters. The wire format is totals-only, so a
//     missed connection cannot lose traffic — there is nothing to latch and
//     nothing to reset.
//   - One I2C bus, used only as a master for the MCP23017s.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "pins.h"
#include "counter_protocol.h"

// The BLE/GATT transport: a connectable NimBLE GATT server serving the
// measurement characteristic and the OTA characteristics. See ble_link.h.
#include "ble_link.h"

// ============================================================================
// Compile-time tuning knobs — keep these together for easy field adjustment
// ============================================================================

// How often (ms) we poll the MCP23017s for crossing detection. With a
// dedicated master bus we can poll continuously; 5 ms keeps CPU use modest
// while still catching the ~12 ms beam dwell of a crossing bee.
//
// With pulsed LEDs, this is also the emitter pulse PERIOD: the LEDs are lit
// for (LED_SETTLE_US + the three-chip read time) once every POLL_INTERVAL_MS.
// Raising it lowers both CPU use and emitter duty cycle, at the cost of coarser
// timing resolution on a crossing. 5 ms still resolves a ~12 ms beam dwell into
// ~2-3 samples, which is enough for the inner/outer ordering to be reliable.
static constexpr uint32_t POLL_INTERVAL_MS = 5;

// Minimum time a sensor must read "blocked" (LOW) to be considered a real
// detection event rather than electrical noise. Bees crossing at ~25 cm/s
// across a 3 mm beam dwell ~12 ms in the beam, so 3 ms debouncing is plenty.
static constexpr uint32_t SENSOR_DEBOUNCE_MS = 3;

// Maximum elapsed time between the two sensors of one gate triggering for
// the pair to count as a directional crossing. Anything longer is just a
// bee sitting in the tunnel and is discarded.
static constexpr uint32_t GATE_PAIRING_WINDOW_MS = 2000;

// If a sensor is continuously "blocked" for longer than this, we flag the
// sensor-fault status bit. A bee cannot physically block a beam for 30 s.
static constexpr uint32_t SENSOR_STUCK_MS = 30000;

// Run the short, on-board MCP23017 bus at its 400 kHz fast-mode limit. The IR
// emitters stay lit for the complete three-chip read, so this reduces their
// on-time and CPU blocking roughly 3-4x versus the former 100 kHz default. Fall
// back to 100 kHz if an assembled board cannot meet fast-mode signal integrity.
static constexpr uint32_t I2C_MASTER_HZ = 400000;

// ---------------------------------------------------------------------------
// Pulsed-LED sampling
// ---------------------------------------------------------------------------
// The QRE1113 reflective sensor's phototransistor needs a short settle time
// after its IR emitter switches on before the collector voltage is valid. The
// device's optical rise/fall times are on the order of ~10 us; 250 us is a
// comfortable, conservative margin that also covers the RC settling of the
// 100k MCP pull-up against the phototransistor + any board capacitance.
//
// The sampling sequence each poll is:
//     1. turn ALL emitter banks ON
//     2. busy-wait LED_SETTLE_US for the phototransistors to settle
//     3. read all three MCP23017s (emitters stay on across the whole read)
//     4. turn ALL emitter banks OFF
//
// All banks are pulsed together (not per-bank) so that a single readGPIOAB()
// sweep of all three chips sees every gate correctly lit. Since the 2026-08
// revision each bank feeds exactly one MCP23017, so a per-bank read is now
// possible in principle — but it would triple the number of settle windows
// (each ~250 us) for no benefit, since the three chips are read back to back
// anyway. Average emitter current ≈ peak * (settle + read) /
// (POLL_INTERVAL_MS * 1000). With the defaults that is ~35% of the old
// always-on draw; raise POLL_INTERVAL_MS and/or I2C_MASTER_HZ to push lower.
//
// FORCE_ON keeps the old continuous behaviour (useful for bench/oscilloscope
// work); FORCE_OFF blacks the emitters out entirely (counts will read clear);
// AUTO is the new pulsed mode and the default.
static constexpr uint32_t LED_SETTLE_US = 250;

// ============================================================================
// Per-gate state machine
// ============================================================================
//
// We model each gate as a tiny state machine. The two sensors (Inner, Outer)
// each can be in one of two debounced states: CLEAR (HIGH, beam clear) or
// BLOCKED (LOW, body in beam). Transitions to BLOCKED are timestamped. When
// the second sensor of a pair becomes BLOCKED within GATE_PAIRING_WINDOW_MS
// of the first, we emit a directional event:
//
//     first BLOCKED = Outer -> then Inner BLOCKED  =>  INCOMING bee
//     first BLOCKED = Inner -> then Outer BLOCKED  =>  OUTGOING bee
//
// After an event the gate resets to IDLE only once BOTH sensors have
// returned to CLEAR for at least SENSOR_DEBOUNCE_MS.
// ============================================================================

enum class GateState : uint8_t {
    IDLE,           // both sensors clear, no event in progress
    OUTER_FIRST,    // outer triggered first, awaiting inner
    INNER_FIRST,    // inner triggered first, awaiting outer
    PAIRED,         // both triggered, waiting for both to clear
};

struct GateRuntime {
    GateState state = GateState::IDLE;
    bool inner_blocked = false;
    bool outer_blocked = false;
    uint32_t inner_change_ms = 0;     // last sample-time the inner toggled
    uint32_t outer_change_ms = 0;
    uint32_t event_start_ms = 0;      // when current pairing began
    uint32_t inner_stuck_since_ms = 0;
    uint32_t outer_stuck_since_ms = 0;
};

static GateRuntime g_gate_rt[gates::NUM_GATES];

// Aggregate counters. LIFETIME totals only: the BLE contract is totals-only and
// HiveHub derives each interval by differencing consecutive reads, so a missed
// connection can never lose traffic. (The wired path additionally kept
// per-interval and per-gate counters, zeroed by CMD_LATCH; both went with it.)
static volatile uint32_t g_total_in  = 0;
static volatile uint32_t g_total_out = 0;

static volatile uint16_t g_glitch_count  = 0;
static volatile uint8_t  g_status_flags  = 0;

// ============================================================================
// MCP23017 driver objects  (all on Wire / bus 0)
// ============================================================================
static Adafruit_MCP23X17 g_mcp_u2;   // 0x20, gates 00..07
static Adafruit_MCP23X17 g_mcp_u3;   // 0x21, gates 10..17
static Adafruit_MCP23X17 g_mcp_u4;   // 0x22, gates 20..27

static bool g_mcp_u2_ok = false;
static bool g_mcp_u3_ok = false;
static bool g_mcp_u4_ok = false;

// LED-bank control.
// AUTO now PULSES the LEDs: they are lit only for the settle+read window of
// each poll (see pollAllGates / sampleGatesPulsed). FORCE_ON restores the old
// always-on behaviour; FORCE_OFF keeps them dark.
// The IRLB8721 N-FET is logic-level and just needs a digital HIGH on its gate.
enum class LedMode : uint8_t { AUTO, FORCE_ON, FORCE_OFF };
static volatile LedMode g_led_mode = LedMode::AUTO;

// ============================================================================
// Low-level helpers
// ============================================================================

// Drive every emitter-bank MOSFET to the same state and update the status bit.
// This is the raw control used by FORCE_ON/FORCE_OFF and by the pulsed sampler.
// It does NOT consult g_led_mode (the caller decides), so the pulsed sampler
// can momentarily turn the LEDs on/off within AUTO mode without fighting the
// mode gate.
static void driveIrLeds(bool on) {
    for (uint8_t b = 0; b < pins::NUM_LED_BANKS; b++) {
        digitalWrite(pins::IR_LED_BANK_EN[b], on ? HIGH : LOW);
    }
    if (on) g_status_flags |=  beecounter_proto::STATUS_IR_LEDS_ON;
    else    g_status_flags &= ~beecounter_proto::STATUS_IR_LEDS_ON;
}

// Apply a steady LED state honouring the mode overrides. Used when the mode
// changes (the IR_DEBUG console's '1'/'0'/'a' keys) and at boot. In AUTO mode the steady state is
// OFF — the emitters are only lit transiently by the pulsed sampler — so a
// call here with on=true while in AUTO is treated as "leave pulsing to the
// sampler" and forces the steady level OFF.
static void setIrLeds(bool on) {
    if (g_led_mode == LedMode::FORCE_OFF) on = false;
    else if (g_led_mode == LedMode::FORCE_ON) on = true;
    else /* AUTO */ on = false;   // pulsed: steady level is OFF between samples
    driveIrLeds(on);
}


// ============================================================================
// Crossing detection
// ============================================================================

// Process one gate after its two sensor states have been freshly read.
// raw_inner / raw_outer: true means "beam blocked" (sensor reads LOW).
static void updateGate(uint8_t gate_idx, bool raw_inner, bool raw_outer,
                       uint32_t now_ms) {
    GateRuntime& rt = g_gate_rt[gate_idx];

    // ----- debounce -----
    if (raw_inner != rt.inner_blocked &&
        now_ms - rt.inner_change_ms >= SENSOR_DEBOUNCE_MS) {
        rt.inner_blocked    = raw_inner;
        rt.inner_change_ms  = now_ms;
        if (raw_inner) rt.inner_stuck_since_ms = now_ms;
    } else if (raw_inner == rt.inner_blocked) {
        rt.inner_change_ms  = now_ms;   // refresh "still in this state" stamp
    }

    if (raw_outer != rt.outer_blocked &&
        now_ms - rt.outer_change_ms >= SENSOR_DEBOUNCE_MS) {
        rt.outer_blocked    = raw_outer;
        rt.outer_change_ms  = now_ms;
        if (raw_outer) rt.outer_stuck_since_ms = now_ms;
    } else if (raw_outer == rt.outer_blocked) {
        rt.outer_change_ms  = now_ms;
    }

    // ----- stuck-sensor health check -----
    if (rt.inner_blocked && now_ms - rt.inner_stuck_since_ms > SENSOR_STUCK_MS) {
        g_status_flags |= beecounter_proto::STATUS_SENSOR_FAULT_FLAG;
    }
    if (rt.outer_blocked && now_ms - rt.outer_stuck_since_ms > SENSOR_STUCK_MS) {
        g_status_flags |= beecounter_proto::STATUS_SENSOR_FAULT_FLAG;
    }

    // ----- state machine -----
    switch (rt.state) {
    case GateState::IDLE:
        if (rt.inner_blocked && !rt.outer_blocked) {
            rt.state = GateState::INNER_FIRST;
            rt.event_start_ms = now_ms;
        } else if (rt.outer_blocked && !rt.inner_blocked) {
            rt.state = GateState::OUTER_FIRST;
            rt.event_start_ms = now_ms;
        } else if (rt.inner_blocked && rt.outer_blocked) {
            // both blocked at the same poll -- ambiguous direction, treat
            // as a glitch and stay paired until both clear.
            rt.state = GateState::PAIRED;
            g_glitch_count += 1;
        }
        break;

    case GateState::INNER_FIRST:
        // expecting outer to block next -> OUTGOING bee
        if (now_ms - rt.event_start_ms > GATE_PAIRING_WINDOW_MS) {
            // timeout: bee stayed inside or backed off. Reset on clear.
            if (!rt.inner_blocked && !rt.outer_blocked) {
                rt.state = GateState::IDLE;
            }
        } else if (rt.outer_blocked) {
            // direction confirmed
            g_total_out += 1;
            rt.state = GateState::PAIRED;
        } else if (!rt.inner_blocked) {
            // inner cleared before outer ever blocked: false start
            rt.state = GateState::IDLE;
            g_glitch_count += 1;
        }
        break;

    case GateState::OUTER_FIRST:
        // expecting inner to block next -> INCOMING bee
        if (now_ms - rt.event_start_ms > GATE_PAIRING_WINDOW_MS) {
            if (!rt.inner_blocked && !rt.outer_blocked) {
                rt.state = GateState::IDLE;
            }
        } else if (rt.inner_blocked) {
            g_total_in += 1;
            rt.state = GateState::PAIRED;
        } else if (!rt.outer_blocked) {
            rt.state = GateState::IDLE;
            g_glitch_count += 1;
        }
        break;

    case GateState::PAIRED:
        // wait until both sensors clear, then return to IDLE
        if (!rt.inner_blocked && !rt.outer_blocked) {
            rt.state = GateState::IDLE;
        }
        break;
    }

    // overflow detection on the lifetime counters
    if (g_total_in == 0xFFFFFFFFu || g_total_out == 0xFFFFFFFFu) {
        g_status_flags |= beecounter_proto::STATUS_OVERFLOW_FLAG;
    }
}

// Read all three MCP23017s into v_u2/v_u3/v_u4. The emitters must already be
// lit and settled before this is called. Returns false if any chip failed to
// respond (its value is left at 0 = all-blocked, but the caller skips updating
// gates on a failed chip via the any_fail path it tracks separately).
static bool readAllMcp(uint16_t& v_u2, uint16_t& v_u3, uint16_t& v_u4) {
    bool any_fail = false;
    v_u2 = v_u3 = v_u4 = 0;
    if (g_mcp_u2_ok) v_u2 = g_mcp_u2.readGPIOAB(); else any_fail = true;
    if (g_mcp_u3_ok) v_u3 = g_mcp_u3.readGPIOAB(); else any_fail = true;
    if (g_mcp_u4_ok) v_u4 = g_mcp_u4.readGPIOAB(); else any_fail = true;
    return !any_fail;
}

// Acquire one fresh set of sensor readings with the emitters pulsed on only for
// the settle + read window. In FORCE_ON the emitters are already steady-on, so
// we skip the extra toggling. In FORCE_OFF we never light them (readings will
// look "clear"), preserving the diagnostic meaning of that mode.
static bool sampleGates(uint16_t& v_u2, uint16_t& v_u3, uint16_t& v_u4) {
    switch (g_led_mode) {
    case LedMode::FORCE_OFF:
        // Emitters stay dark; read whatever the sensors show (nominally clear).
        return readAllMcp(v_u2, v_u3, v_u4);

    case LedMode::FORCE_ON:
        // Emitters are already steady-on (set when the mode was entered); just
        // read. No per-sample toggling so a scope trace shows a clean DC level.
        return readAllMcp(v_u2, v_u3, v_u4);

    case LedMode::AUTO:
    default:
        // Pulsed path: light all banks, let the phototransistors settle, read
        // across the lit window, then black the emitters out again.
        driveIrLeds(true);
        delayMicroseconds(LED_SETTLE_US);
        {
            bool ok = readAllMcp(v_u2, v_u3, v_u4);
            driveIrLeds(false);
            return ok;
        }
    }
}

// Poll all three MCP23017s and update every gate. Returns false if any
// chip failed to respond -- in that case we don't update the affected
// gates' sensor state this cycle, which is harmless.
static bool pollAllGates() {
    const uint32_t now_ms = millis();

    // Acquire a fresh sensor snapshot with the emitters pulsed (AUTO) or steady
    // (FORCE_ON) per the current LED mode. Adafruit_MCP23X17::readGPIOAB()
    // returns GPIOA in the low byte and GPIOB in the high byte.
    uint16_t v_u2 = 0, v_u3 = 0, v_u4 = 0;
    bool ok = sampleGates(v_u2, v_u3, v_u4);

    // Named getBit to avoid clashing with Arduino.h's bit(b) macro.
    auto getBit = [](uint16_t v, uint8_t pin) -> bool {
        return (v >> pin) & 0x1;
    };

    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        const auto& loc = gates::TABLE[i];
        uint16_t v;
        switch (loc.mcp_address) {
        case i2c_addr::MCP_GATES_00_07: v = v_u2; break;
        case i2c_addr::MCP_GATES_10_17: v = v_u3; break;
        case i2c_addr::MCP_GATES_20_27: v = v_u4; break;
        default: continue;
        }
        // QRE1113 phototransistor: BLOCKED -> sensor line LOW (bit=0).
        // We pass "blocked = (bit == 0)" into the state machine.
        bool inner_blocked = !getBit(v, loc.inner_pin);
        bool outer_blocked = !getBit(v, loc.outer_pin);
        updateGate(i, inner_blocked, outer_blocked, now_ms);
    }

    return ok;
}

// ============================================================================
// USB serial IR-sensor debug console  (compile-time, -DIR_DEBUG only)
// ============================================================================
//
// A tiny interactive console for bench bring-up: it lets you watch the raw
// inner/outer beam state of all 24 gates over the USB CDC serial link without
// any HiveScale / I2C master attached. It is intentionally excluded from the
// production build — none of this code is compiled unless -DIR_DEBUG is set.
// No PlatformIO environment defines it; build the console ad hoc with
//     PLATFORMIO_BUILD_FLAGS=-DIR_DEBUG pio run -t upload
//
// Commands (single keypress in the serial monitor):
//   r  read & print every gate's sensors once
//   s  toggle continuous streaming (every DBG_STREAM_INTERVAL_MS)
//   1  force IR LEDs ON   (steady)
//   0  force IR LEDs OFF
//   a  IR LEDs AUTO       (normal pulsed mode)
//   h  print the command list
// ============================================================================
#ifdef IR_DEBUG

static bool     g_dbg_stream  = false;
static uint32_t g_dbg_last_ms = 0;
static constexpr uint32_t DBG_STREAM_INTERVAL_MS = 200;

static void irDebugPrintHelp() {
    Serial.println();
    Serial.println(F("[IR-DEBUG] interactive sensor console — commands:"));
    Serial.println(F("  r  read & print all 24 gates once"));
    Serial.println(F("  s  toggle continuous streaming (~200 ms)"));
    Serial.println(F("  i  scan the master I2C bus for devices"));
    Serial.println(F("  1  force IR LEDs ON (steady)"));
    Serial.println(F("  0  force IR LEDs OFF"));
    Serial.println(F("  a  IR LEDs AUTO (pulsed, normal mode)"));
    Serial.println(F("  h  show this help"));
    Serial.println();
}

// Take one fresh reading of all 24 gates and print it. The emitters are pulsed
// on for the settle+read window regardless of the current LED mode, so the
// readout is always valid on the bench even in AUTO/FORCE_OFF; afterwards the
// mode-correct steady level is restored.
static void irDebugReadAndPrint() {
    driveIrLeds(true);
    delayMicroseconds(LED_SETTLE_US);
    uint16_t v_u2 = 0, v_u3 = 0, v_u4 = 0;
    bool ok = readAllMcp(v_u2, v_u3, v_u4);
    setIrLeds(true);   // restore mode-correct steady level (OFF in AUTO)

    auto getBit = [](uint16_t v, uint8_t pin) -> bool { return (v >> pin) & 0x1; };

    Serial.printf("[IR] t=%lus raw U2=0x%04X U3=0x%04X U4=0x%04X read_ok=%d\n",
                  (unsigned long)(millis() / 1000), v_u2, v_u3, v_u4, ok ? 1 : 0);
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        const auto& loc = gates::TABLE[i];
        uint16_t v;
        switch (loc.mcp_address) {
        case i2c_addr::MCP_GATES_00_07: v = v_u2; break;
        case i2c_addr::MCP_GATES_10_17: v = v_u3; break;
        case i2c_addr::MCP_GATES_20_27: v = v_u4; break;
        default: continue;
        }
        // BLOCKED == beam reflected/interrupted == sensor line LOW (bit 0).
        bool inner_blocked = !getBit(v, loc.inner_pin);
        bool outer_blocked = !getBit(v, loc.outer_pin);
        // The bank column makes a dead emitter FET obvious on the bench: a
        // whole bank reading "clear" with bees present points at Q1/Q2/Q3
        // rather than at the sensors.
        Serial.printf("  %-8s bank:%u inner:%-5s outer:%-5s\n", loc.tag,
                      (unsigned)loc.led_bank,
                      inner_blocked ? "BLOCK" : "clear",
                      outer_blocked ? "BLOCK" : "clear");
    }
}

// Probe every 7-bit address on the master bus (Wire) and report what ACKs.
// This is the quickest way to tell a wiring/pin fault (nothing responds) from
// an address-strap fault (something responds, but not at the expected 0x20).
static void irDebugI2cScan() {
    Serial.println(F("[IR-DEBUG] scanning master I2C bus (Wire / SDA,SCL in pins.h)..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  device ACK at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        Serial.println(F("  no devices responded."));
        Serial.println(F("  -> check: SDA/SCL not swapped, pull-ups to 3V3, "
                         "MCP /RESET high, common GND, A0/A1/A2 strapped"));
    } else {
        Serial.printf("  %u device(s) total. Expected MCP23017s at "
                      "0x20/0x21/0x22.\n", found);
    }
}

// Drain any pending serial input and service streaming. Called from loop().
static void irDebugPoll() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        switch (c) {
        case 'r':
            irDebugReadAndPrint();
            break;
        case 's':
            g_dbg_stream  = !g_dbg_stream;
            g_dbg_last_ms = 0;   // stream immediately on enable
            Serial.printf("[IR-DEBUG] streaming %s\n", g_dbg_stream ? "ON" : "OFF");
            break;
        case 'i':
            irDebugI2cScan();
            break;
        case '1':
            g_led_mode = LedMode::FORCE_ON;
            setIrLeds(true);
            Serial.println(F("[IR-DEBUG] IR LEDs -> FORCE_ON"));
            break;
        case '0':
            g_led_mode = LedMode::FORCE_OFF;
            setIrLeds(false);
            Serial.println(F("[IR-DEBUG] IR LEDs -> FORCE_OFF"));
            break;
        case 'a':
            g_led_mode = LedMode::AUTO;
            setIrLeds(false);
            Serial.println(F("[IR-DEBUG] IR LEDs -> AUTO (pulsed)"));
            break;
        case 'h':
        case '?':
            irDebugPrintHelp();
            break;
        case '\r':
        case '\n':
        case ' ':
            break;   // ignore whitespace/line endings
        default:
            Serial.printf("[IR-DEBUG] unknown key '%c' — press 'h' for help\n",
                          (char)c);
            break;
        }
    }

    if (g_dbg_stream && millis() - g_dbg_last_ms >= DBG_STREAM_INTERVAL_MS) {
        g_dbg_last_ms = millis();
        irDebugReadAndPrint();
    }
}

#endif  // IR_DEBUG

// ----------------------------------------------------------------------------
// BLE transport callbacks (defined here because the counter state lives in this
// translation unit). ble_link.cpp drives the GATT server and calls into these.
// ----------------------------------------------------------------------------
namespace ble {

void getTelemetry(Telemetry& t) {
    uint32_t up = millis() / 1000;
    if (up > 0xFFFF) up = 0xFFFF;
    t.protocol_version = beecounter_proto::PROTOCOL_VERSION;
    t.status_flags     = g_status_flags;
    t.uptime_s         = (uint16_t)up;
    t.num_gates        = gates::NUM_GATES;
    t.gates_healthy    = (uint8_t)((g_mcp_u2_ok ? 1 : 0) +
                                   (g_mcp_u3_ok ? 1 : 0) +
                                   (g_mcp_u4_ok ? 1 : 0));
    t.total_in         = g_total_in;
    t.total_out        = g_total_out;
    t.glitch_count     = g_glitch_count;
}

}  // namespace ble


// ============================================================================
// Bring-up helpers
// ============================================================================

static bool initMcp(Adafruit_MCP23X17& mcp, uint8_t addr, const char* tag) {
    if (!mcp.begin_I2C(addr, &Wire)) {
        Serial.printf("[MCP] %s @ 0x%02X: NOT FOUND\n", tag, addr);
        return false;
    }
    // All 16 pins are sensor inputs. The board provides 100k pull-ups, so we
    // use plain INPUT (not INPUT_PULLUP) to keep the MCP's weak internal
    // pull-ups out of the picture.
    for (uint8_t p = 0; p < 16; p++) {
        mcp.pinMode(p, INPUT);
    }
    Serial.printf("[MCP] %s @ 0x%02X: OK\n", tag, addr);
    return true;
}


// ============================================================================
// Arduino setup() / loop()
// ============================================================================

static uint32_t g_last_poll_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("==============================================");
    Serial.println("Easy Bee Counter 2026 — firmware booting (BLE/GATT link)");
    Serial.println("==============================================");

    // Configure LED enable pins -- start with LEDs off so we can verify
    // the MCP23017 pull-up baseline before powering the emitters.
    for (uint8_t b = 0; b < pins::NUM_LED_BANKS; b++) {
        pinMode(pins::IR_LED_BANK_EN[b], OUTPUT);
        digitalWrite(pins::IR_LED_BANK_EN[b], LOW);
    }

    // The MCP23017 bus. There is only one bus now: the second I2C controller
    // used to run a permanent slave for the HiveScale, and that link is gone.
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);

    g_mcp_u2_ok = initMcp(g_mcp_u2, i2c_addr::MCP_GATES_00_07, "U2 (gates 00..07)");
    g_mcp_u3_ok = initMcp(g_mcp_u3, i2c_addr::MCP_GATES_10_17, "U3 (gates 10..17)");
    g_mcp_u4_ok = initMcp(g_mcp_u4, i2c_addr::MCP_GATES_20_27, "U4 (gates 20..27)");

    if (g_mcp_u2_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U2_OK;
    if (g_mcp_u3_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U3_OK;
    if (g_mcp_u4_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U4_OK;

    ble::begin();   // connectable NimBLE GATT server on the GPIO-less radio

    // Emitters now default to PULSED (AUTO): they stay dark between samples and
    // are lit only for the settle+read window inside pollAllGates(). Leave the
    // steady level OFF here; the first pollAllGates() below will pulse them.
    g_led_mode = LedMode::AUTO;
    setIrLeds(false);

    // One warm-up poll so the state machine has fresh baselines before any
    // crossings start counting. This also exercises the pulsed-sample path.
    pollAllGates();

    g_status_flags |= beecounter_proto::STATUS_READY;

    Serial.println("[SETUP] Entering normal counting loop (pulsed IR)");

#ifdef IR_DEBUG
    Serial.println("[SETUP] IR_DEBUG build — USB sensor console enabled");
    irDebugPrintHelp();
#endif
}

void loop() {
    // Services the deferred post-OTA reboot, once the central has had a chance
    // to read DONE off the status characteristic.
    ble::loopOta();

    const uint32_t now = millis();

#ifdef IR_DEBUG
    // Service the USB serial sensor console (bench bring-up builds only).
    irDebugPoll();
#endif

    // ---- Poll the MCP23017s every POLL_INTERVAL_MS ----
    // An OTA pauses sensing: flash writes must not leave an emitter pulse
    // active or corrupt a crossing in progress. Counting is deliberately
    // sacrificed for the duration of a transfer.
    if (ble::isOtaActive()) {
        driveIrLeds(false);
    } else if (now - g_last_poll_ms >= POLL_INTERVAL_MS) {
        g_last_poll_ms = now;
        pollAllGates();   // pulses the emitters internally in AUTO mode
    }

    // Keep the steady LED level in sync with the current mode. In AUTO this
    // resolves to OFF (the pulsed sampler owns the emitters); FORCE_ON drives
    // them steady-on; FORCE_OFF holds them dark.
    static LedMode last_led_mode = LedMode::AUTO;
    if (g_led_mode != last_led_mode) {
        last_led_mode = g_led_mode;
        setIrLeds(true);   // setIrLeds() applies the mode-correct steady level
    }

    // Periodic debug dump on serial -- once every 30 s.
    static uint32_t last_dump_ms = 0;
    if (now - last_dump_ms > 30000) {
        last_dump_ms = now;
        Serial.printf(
            "[STAT] uptime=%lus total_in=%lu total_out=%lu "
            "glitches=%u status=0x%02X\n",
            (unsigned long)(now / 1000),
            (unsigned long)g_total_in,
            (unsigned long)g_total_out,
            (unsigned)g_glitch_count,
            (unsigned)g_status_flags
        );
    }
}
