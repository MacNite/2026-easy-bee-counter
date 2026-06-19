// ============================================================================
// Easy Bee Counter 2026 — ESP32-C6 mini firmware  (DUAL-I2C revision)
// ----------------------------------------------------------------------------
// Author: rewritten 2026 for the new ESP32-C6 mini board.
// 2026 revision: split the single shared I2C bus into TWO dedicated buses.
//
// 2026-06 power revision: the IR emitters are now PULSED instead of left on
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
//      Outer IR sensor) through three MCP23017 I2C port expanders.
//   2. Detects directional bee crossings:
//        Outer-then-Inner = bee entered the hive   -> "in" counter++
//        Inner-then-Outer = bee left the hive      -> "out" counter++
//   3. Maintains lifetime totals, per-interval totals, and per-gate detail.
//   4. Acts as an I2C slave at address 0x30 on a SEPARATE bus so the main
//      HiveScale ESP32 can poll it at any time without any timing constraints.
//
// Dual-bus hardware layout (NEW)
// ------------------------------
// The ESP32-C6 has TWO independent I2C controllers, so we now dedicate one to
// each role and never switch modes:
//
//   Wire   (bus 0): MASTER only, GPIO4/GPIO5  -> the 3x MCP23017s.
//   Wire1  (bus 1): SLAVE  only, GPIO2/GPIO3  -> the HiveScale link.
//
// The external HiveScale connector is wired to GPIO2 (SDA_HiveScale) and
// GPIO3 (SCL_HiveScale). Pull-ups for that bus are provided by the HiveScale
// I2C network. The old J1<->GPIO4/5 SDA/SCL traces are cut.
//
// Consequences vs. the old single-bus firmware:
//   - No master/slave time-multiplexing, no "slave window" scheduler.
//   - The HiveScale can start a transaction at any instant; it can never
//     collide with an MCP read, so no NACK/stretch retry is required.
//   - REG_BUSY_RETRIES is retained in the protocol for compatibility but is
//     now always 0.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Update.h>          // ESP32 OTA writer (esp_ota under the hood)

#include "pins.h"
#include "i2c_slave_protocol.h"

// Optional BLE/GATT transport (compile-time alternative to the I2C slave; see
// ble_link.h and platformio.ini). When -DBEECOUNTER_BLE is set the Wire1 slave
// and the OTA-over-I2C path are compiled out and replaced by a connectable
// NimBLE GATT server, mirroring HiveInside's wireless transfer.
#ifdef BEECOUNTER_BLE
#include "ble_link.h"
#endif

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

// I2C bus speed for the master (MCP) bus. 400 kHz is the MCP23017 spec limit;
// 100 kHz is safer over long cabling. The on-board MCP traces are short, but
// 100 kHz is left as the conservative default.
//
// NOTE on pulsed LEDs: the emitters stay lit for the whole three-chip read, so
// the emitter ON-time (and therefore the duty cycle / average current) scales
// with the read time. At 100 kHz the read is ~1.5 ms; bumping this to 400000
// shortens it to ~0.4 ms and cuts emitter duty roughly 3-4x with no loss of
// detection quality on the short on-board traces. Worth doing if you want lower
// power without touching POLL_INTERVAL_MS.
static constexpr uint32_t I2C_MASTER_HZ = 100000;

// I2C clock for the HiveScale slave bus. As a slave the C6 follows the
// master's clock; this value is only the controller's configured rate.
static constexpr uint32_t I2C_SLAVE_HZ = 100000;

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
//     1. turn BOTH emitter banks ON
//     2. busy-wait LED_SETTLE_US for the phototransistors to settle
//     3. read all three MCP23017s (emitters stay on across the whole read)
//     4. turn BOTH emitter banks OFF
//
// Both banks are pulsed together (not per-bank) so that a single readGPIOAB()
// sweep of all three chips sees every gate correctly lit; gates on U3 straddle
// the two banks, so splitting the read per bank would needlessly double the
// emitter on-time. Average emitter current ≈ peak * (settle + read) /
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

// Per-gate live (currently filling) and shadow (last-latched, served to
// HiveScale) counters for IN/OUT. uint8_t per gate clipped at 255 — over a
// 10-minute interval, 255 bees per gate is already an extreme rate.
struct GateCounters {
    uint8_t in  = 0;
    uint8_t out = 0;
};

static GateRuntime  g_gate_rt[gates::NUM_GATES];
static GateCounters g_gate_live[gates::NUM_GATES];
static GateCounters g_gate_shadow[gates::NUM_GATES];

// Aggregate counters.
//   *_live   = accumulating right now; reset on CMD_LATCH
//   *_shadow = what we hand to the HiveScale; updated atomically on LATCH
//   *_total  = lifetime, never reset (except on CMD_CLEAR_TOTALS)
static volatile uint32_t g_interval_in_live  = 0;
static volatile uint32_t g_interval_out_live = 0;
static volatile uint32_t g_interval_in_shadow  = 0;
static volatile uint32_t g_interval_out_shadow = 0;
static volatile uint32_t g_total_in  = 0;
static volatile uint32_t g_total_out = 0;

static volatile uint16_t g_glitch_count  = 0;
static volatile uint16_t g_busy_retries  = 0;   // retained, always 0 now
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
// I2C slave-callback state (shared with ISR context)
// ============================================================================
//
// Wire1 invokes onReceive()/onRequest() in an ISR-ish callback context. We
// keep the work tiny: stash the register pointer on receive, and on request
// stream bytes from a locally-built response buffer. Counter reads are of
// volatile snapshots updated by the main loop; a torn multi-byte read is
// harmless here because the HiveScale re-reads each interval and we only ever
// hand out the latched *shadow* values, which the main loop updates only
// inside a noInterrupts() block on CMD_LATCH.
// ============================================================================

#ifndef BEECOUNTER_BLE
static volatile uint8_t  g_reg_pointer = beecounter_proto::REG_STATUS;

// ----------------------------------------------------------------------------
// OTA-over-I2C state (PROTOCOL_VERSION >= 2). Written from the Wire1 slave
// callback context, read by loop(). g_ota_active pauses gate polling for the
// duration of a transfer; g_ota_reboot defers the restart until loop() runs,
// so the HiveScale gets a chance to read OTA_STATE_DONE first.
//
// In the BLE build (-DBEECOUNTER_BLE) this whole block is replaced by the
// firmware-over-BLE state machine in ble_link.cpp.
// ----------------------------------------------------------------------------
static volatile bool     g_ota_active       = false;
static volatile bool     g_ota_reboot       = false;
static volatile uint8_t  g_ota_state        = beecounter_proto::OTA_STATE_IDLE;
static volatile uint8_t  g_ota_err          = beecounter_proto::OTA_ERR_NONE;
static volatile uint32_t g_ota_size         = 0;            // declared image size
static volatile uint32_t g_ota_expected_crc = 0;            // declared CRC32
static volatile uint32_t g_ota_received     = 0;            // bytes written so far
static volatile uint32_t g_ota_running_crc  = 0xFFFFFFFFu;  // CRC accumulator

// Response buffer: enough room for the largest single-read register block.
// The biggest is REG_PER_GATE_IN (24 bytes) or REG_PER_GATE_OUT (24 bytes).
static constexpr size_t SLAVE_TX_BUF_SIZE = 32;
#endif  // !BEECOUNTER_BLE

// ============================================================================
// Low-level helpers
// ============================================================================

// Drive both emitter-bank MOSFETs to the same state and update the status bit.
// This is the raw control used by FORCE_ON/FORCE_OFF and by the pulsed sampler.
// It does NOT consult g_led_mode (the caller decides), so the pulsed sampler
// can momentarily turn the LEDs on/off within AUTO mode without fighting the
// mode gate.
static void driveIrLeds(bool on) {
    digitalWrite(pins::IR_LED_BANK_1_EN, on ? HIGH : LOW);
    digitalWrite(pins::IR_LED_BANK_2_EN, on ? HIGH : LOW);
    if (on) g_status_flags |=  beecounter_proto::STATUS_IR_LEDS_ON;
    else    g_status_flags &= ~beecounter_proto::STATUS_IR_LEDS_ON;
}

// Apply a steady LED state honouring the mode overrides. Used when the mode
// changes (e.g. CMD_LEDS_ON/OFF) and at boot. In AUTO mode the steady state is
// OFF — the emitters are only lit transiently by the pulsed sampler — so a
// call here with on=true while in AUTO is treated as "leave pulsing to the
// sampler" and forces the steady level OFF.
static void setIrLeds(bool on) {
    if (g_led_mode == LedMode::FORCE_OFF) on = false;
    else if (g_led_mode == LedMode::FORCE_ON) on = true;
    else /* AUTO */ on = false;   // pulsed: steady level is OFF between samples
    driveIrLeds(on);
}

#ifndef BEECOUNTER_BLE
// Big-endian encode helpers (I2C slave response framing).
static void writeU16BE(uint8_t* buf, uint16_t v) {
    buf[0] = (uint8_t)(v >> 8);
    buf[1] = (uint8_t)(v & 0xFF);
}
static void writeU32BE(uint8_t* buf, uint32_t v) {
    buf[0] = (uint8_t)(v >> 24);
    buf[1] = (uint8_t)(v >> 16);
    buf[2] = (uint8_t)(v >> 8);
    buf[3] = (uint8_t)(v & 0xFF);
}

// Incremental CRC-32 (IEEE 802.3, poly 0xEDB88320). Seed 0xFFFFFFFF, finalize
// by XOR 0xFFFFFFFF. Must match the HiveScale and backend (zlib.crc32).
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return crc;
}
#endif  // !BEECOUNTER_BLE

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
            g_interval_out_live += 1;
            if (g_gate_live[gate_idx].out < 255) g_gate_live[gate_idx].out++;
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
            g_interval_in_live += 1;
            if (g_gate_live[gate_idx].in < 255) g_gate_live[gate_idx].in++;
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
        // Pulsed path: light both banks, let the phototransistors settle, read
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
// production build — none of this code is compiled unless -DIR_DEBUG is set
// (see the esp32-c6-devkitc-1-irdebug env in platformio.ini).
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
        Serial.printf("  %-8s inner:%-5s outer:%-5s\n", loc.tag,
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

// ============================================================================
// Shared control-command handling (REG_CTRL / BLE CTRL characteristic)
// ============================================================================
//
// One implementation for the CMD_* opcodes, called from BOTH the I2C slave
// receive callback and the BLE control characteristic, so the two transports
// stay byte-for-byte equivalent. CMD_LATCH copies the live counters into the
// readout shadow inside a noInterrupts() block; the others toggle totals /
// faults / LED mode.
static void applyCtrlCommand(uint8_t cmd) {
    switch (cmd) {
    case beecounter_proto::CMD_LATCH:
        // Atomically copy live -> shadow, then zero live.
        noInterrupts();
        g_interval_in_shadow  = g_interval_in_live;
        g_interval_out_shadow = g_interval_out_live;
        g_interval_in_live  = 0;
        g_interval_out_live = 0;
        for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
            g_gate_shadow[i] = g_gate_live[i];
            g_gate_live[i]   = {0, 0};
        }
        interrupts();
        break;
    case beecounter_proto::CMD_CLEAR_TOTALS:
        g_total_in  = 0;
        g_total_out = 0;
        break;
    case beecounter_proto::CMD_CLEAR_FAULTS:
        g_status_flags &= ~(beecounter_proto::STATUS_SENSOR_FAULT_FLAG |
                            beecounter_proto::STATUS_OVERFLOW_FLAG);
        break;
    case beecounter_proto::CMD_LEDS_OFF:  g_led_mode = LedMode::FORCE_OFF; break;
    case beecounter_proto::CMD_LEDS_ON:   g_led_mode = LedMode::FORCE_ON;  break;
    case beecounter_proto::CMD_LEDS_AUTO: g_led_mode = LedMode::AUTO;       break;
    default: break;
    }
}

#ifdef BEECOUNTER_BLE
// ----------------------------------------------------------------------------
// BLE transport callbacks (defined here because the counter state lives in this
// translation unit). ble_link.cpp drives the GATT server and calls into these.
// ----------------------------------------------------------------------------
namespace ble {

void applyCtrl(uint8_t cmd) {
    applyCtrlCommand(cmd);
}

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
    t.interval_in      = g_interval_in_shadow;
    t.interval_out     = g_interval_out_shadow;
    t.glitch_count     = g_glitch_count;
}

}  // namespace ble
#endif  // BEECOUNTER_BLE

#ifndef BEECOUNTER_BLE
// ============================================================================
// I2C slave (Wire1) callbacks — registered once in setup(), never torn down
// ============================================================================

static void onSlaveReceive(int n_bytes) {
    // The HiveScale wrote n_bytes to us. The first byte is the register
    // pointer; any further bytes are command payload (only REG_CTRL takes a
    // payload byte).
    if (n_bytes <= 0) return;
    uint8_t reg = (uint8_t)Wire1.read();
    g_reg_pointer = reg;
    if (reg == beecounter_proto::REG_CTRL && n_bytes >= 2) {
        uint8_t cmd = (uint8_t)Wire1.read();
        applyCtrlCommand(cmd);
    }
    // ---- OTA-over-I2C frames (PROTOCOL_VERSION >= 2) ----
    else if (reg == beecounter_proto::REG_OTA_BEGIN) {
        // 8 payload bytes: size(4 BE) + crc32(4 BE).
        if (n_bytes >= 9) {
            uint8_t b[8];
            for (uint8_t i = 0; i < 8; i++) b[i] = (uint8_t)Wire1.read();
            uint32_t size = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                            ((uint32_t)b[2] << 8)  | (uint32_t)b[3];
            uint32_t crc  = ((uint32_t)b[4] << 24) | ((uint32_t)b[5] << 16) |
                            ((uint32_t)b[6] << 8)  | (uint32_t)b[7];
            g_ota_size         = size;
            g_ota_expected_crc = crc;
            g_ota_received     = 0;
            g_ota_running_crc  = 0xFFFFFFFFu;
            if (Update.begin(size)) {
                g_ota_active = true;
                g_ota_state  = beecounter_proto::OTA_STATE_RECEIVING;
                g_ota_err    = beecounter_proto::OTA_ERR_NONE;
            } else {
                g_ota_active = false;
                g_ota_state  = beecounter_proto::OTA_STATE_ERR_BEGIN;
            }
        }
    }
    else if (reg == beecounter_proto::REG_OTA_DATA) {
        // offset(4 BE) + data bytes.
        if (g_ota_active && n_bytes >= 5) {
            uint8_t ob[4];
            for (uint8_t i = 0; i < 4; i++) ob[i] = (uint8_t)Wire1.read();
            uint32_t offset = ((uint32_t)ob[0] << 24) | ((uint32_t)ob[1] << 16) |
                              ((uint32_t)ob[2] << 8)  | (uint32_t)ob[3];
            if (offset != g_ota_received) {
                Update.abort();
                g_ota_active = false;
                g_ota_state  = beecounter_proto::OTA_STATE_ERR_SEQ;
            } else {
                uint8_t data[beecounter_proto::OTA_CHUNK_MAX];
                int dn = 0;
                while (Wire1.available() && dn < (int)sizeof(data)) {
                    data[dn++] = (uint8_t)Wire1.read();
                }
                size_t w = Update.write(data, dn);
                if ((int)w != dn) {
                    Update.abort();
                    g_ota_active = false;
                    g_ota_state  = beecounter_proto::OTA_STATE_ERR_WRITE;
                } else {
                    g_ota_running_crc = crc32_update(g_ota_running_crc, data, dn);
                    g_ota_received += dn;
                }
            }
        }
    }
    else if (reg == beecounter_proto::REG_OTA_END) {
        if (g_ota_active) {
            uint32_t final_crc = g_ota_running_crc ^ 0xFFFFFFFFu;
            if (g_ota_received != g_ota_size) {
                Update.abort();
                g_ota_state = beecounter_proto::OTA_STATE_ERR_SIZE;
            } else if (final_crc != g_ota_expected_crc) {
                Update.abort();
                g_ota_state = beecounter_proto::OTA_STATE_ERR_CRC;
            } else if (!Update.end(true)) {
                g_ota_state = beecounter_proto::OTA_STATE_ERR_END;
            } else {
                g_ota_state  = beecounter_proto::OTA_STATE_DONE;
                g_ota_reboot = true;   // loop() performs the actual restart
            }
            g_ota_active = false;
        }
    }
    else if (reg == beecounter_proto::REG_OTA_ABORT) {
        if (g_ota_active) {
            Update.abort();
            g_ota_active = false;
            g_ota_state  = beecounter_proto::OTA_STATE_IDLE;
        }
    }
}

static void onSlaveRequest() {
    // Build a response for whatever register the master last pointed at.
    uint8_t buf[SLAVE_TX_BUF_SIZE];
    size_t n = 0;

    switch (g_reg_pointer) {
    case beecounter_proto::REG_PROTOCOL_VERSION:
        buf[n++] = beecounter_proto::PROTOCOL_VERSION;
        break;
    case beecounter_proto::REG_STATUS:
        buf[n++] = g_status_flags;
        break;
    case beecounter_proto::REG_UPTIME_S: {
        uint32_t up = millis() / 1000;
        if (up > 0xFFFF) up = 0xFFFF;
        writeU16BE(buf, (uint16_t)up);
        n = 2;
        break;
    }
    case beecounter_proto::REG_NUM_GATES:
        buf[n++] = gates::NUM_GATES;
        break;
    case beecounter_proto::REG_GATES_HEALTHY:
        buf[n++] = (uint8_t)((g_mcp_u2_ok ? 1 : 0) +
                             (g_mcp_u3_ok ? 1 : 0) +
                             (g_mcp_u4_ok ? 1 : 0));
        break;
    case beecounter_proto::REG_TOTAL_IN:
        writeU32BE(buf, g_total_in);  n = 4; break;
    case beecounter_proto::REG_TOTAL_OUT:
        writeU32BE(buf, g_total_out); n = 4; break;
    case beecounter_proto::REG_INTERVAL_IN:
        writeU32BE(buf, g_interval_in_shadow);  n = 4; break;
    case beecounter_proto::REG_INTERVAL_OUT:
        writeU32BE(buf, g_interval_out_shadow); n = 4; break;
    case beecounter_proto::REG_GLITCH_COUNT:
        writeU16BE(buf, g_glitch_count); n = 2; break;
    case beecounter_proto::REG_BUSY_RETRIES:
        writeU16BE(buf, g_busy_retries); n = 2; break;
    case beecounter_proto::REG_PER_GATE_IN:
        for (uint8_t i = 0; i < gates::NUM_GATES && n < sizeof(buf); i++)
            buf[n++] = g_gate_shadow[i].in;
        break;
    case beecounter_proto::REG_PER_GATE_OUT:
        for (uint8_t i = 0; i < gates::NUM_GATES && n < sizeof(buf); i++)
            buf[n++] = g_gate_shadow[i].out;
        break;
    case beecounter_proto::REG_OTA_STATUS: {
        // state(1) + received(4 BE) + err(1)
        buf[0] = g_ota_state;
        writeU32BE(buf + 1, g_ota_received);
        buf[5] = g_ota_err;
        n = 6;
        break;
    }
    default:
        buf[n++] = 0xFF;   // unknown register
        break;
    }

    Wire1.write(buf, n);
}
#endif  // !BEECOUNTER_BLE

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

#ifndef BEECOUNTER_BLE
static void initHiveSlaveBus() {
    Wire1.begin((uint8_t)i2c_addr::BEECOUNTER_SLAVE,
                pins::I2C_HIVE_SDA, pins::I2C_HIVE_SCL,
                (uint32_t)I2C_SLAVE_HZ);
    Wire1.onReceive(onSlaveReceive);
    Wire1.onRequest(onSlaveRequest);
    Serial.printf("[I2C] HiveScale slave bus up on GPIO%d/%d @ 0x%02X\n",
                  pins::I2C_HIVE_SDA, pins::I2C_HIVE_SCL,
                  (unsigned)i2c_addr::BEECOUNTER_SLAVE);
}
#endif  // !BEECOUNTER_BLE

// ============================================================================
// Arduino setup() / loop()
// ============================================================================

static uint32_t g_last_poll_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("==============================================");
#ifdef BEECOUNTER_BLE
    Serial.println("Easy Bee Counter 2026 — firmware booting (BLE/GATT link)");
#else
    Serial.println("Easy Bee Counter 2026 — firmware booting (dual-I2C)");
#endif
    Serial.println("==============================================");

    // Configure LED enable pins -- start with LEDs off so we can verify
    // the MCP23017 pull-up baseline before powering the emitters.
    pinMode(pins::IR_LED_BANK_1_EN, OUTPUT);
    pinMode(pins::IR_LED_BANK_2_EN, OUTPUT);
    digitalWrite(pins::IR_LED_BANK_1_EN, LOW);
    digitalWrite(pins::IR_LED_BANK_2_EN, LOW);

    // Bus 0 (Wire): permanent MASTER for the MCP23017s.
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);

    g_mcp_u2_ok = initMcp(g_mcp_u2, i2c_addr::MCP_GATES_00_07, "U2 (gates 00..07)");
    g_mcp_u3_ok = initMcp(g_mcp_u3, i2c_addr::MCP_GATES_10_17, "U3 (gates 10..17)");
    g_mcp_u4_ok = initMcp(g_mcp_u4, i2c_addr::MCP_GATES_20_27, "U4 (gates 20..27)");

    if (g_mcp_u2_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U2_OK;
    if (g_mcp_u3_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U3_OK;
    if (g_mcp_u4_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U4_OK;

    // HiveScale link: either the wired I2C slave (default) or the wireless BLE
    // GATT server (-DBEECOUNTER_BLE). Exactly one is compiled in.
#ifdef BEECOUNTER_BLE
    ble::begin();   // connectable NimBLE GATT server on GPIO-less radio
#else
    // Bus 1 (Wire1): permanent SLAVE for the HiveScale on GPIO2/GPIO3.
    initHiveSlaveBus();
#endif

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
#ifndef BEECOUNTER_BLE
    // Deferred reboot: the HiveScale has had a chance to read OTA_STATE_DONE
    // from REG_OTA_STATUS, so it's safe to restart into the new firmware.
    if (g_ota_reboot) {
        Serial.println("[OTA] verified — rebooting into new firmware");
        delay(50);            // let any in-flight I2C ack settle
        ESP.restart();
    }
#else
    // BLE build: the firmware-over-BLE state machine lives in ble_link.cpp;
    // pump its deferred reboot once verified.
    ble::loopOta();
#endif

    const uint32_t now = millis();

#ifdef IR_DEBUG
    // Service the USB serial sensor console (bench bring-up builds only).
    irDebugPoll();
#endif

    // ---- Master work: poll the MCP23017s every POLL_INTERVAL_MS ----
    // The HiveScale slave bus (Wire1) is serviced asynchronously by its
    // onReceive/onRequest callbacks, so the loop body only does master work.
    // While an OTA transfer is in progress we skip polling so the CPU is
    // dedicated to servicing inbound firmware frames; bee counts during the
    // ~20 s update are intentionally sacrificed. We also force the emitters
    // dark for the duration so they aren't left lit by a mid-pulse abort.
#ifdef BEECOUNTER_BLE
    const bool ota_busy = ble::isOtaActive();
#else
    const bool ota_busy = g_ota_active;
#endif
    if (ota_busy) {
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

#ifdef BEECOUNTER_BLE
    // Refresh the BLE measurement characteristic periodically so a connected
    // central that subscribes to notifications sees live totals, and an
    // on-demand read always returns fresh values. Control writes (e.g. LATCH)
    // re-publish immediately from the CTRL callback, so this is just a
    // heartbeat — 2 s is plenty given the ~10-minute upload cadence.
    static uint32_t last_ble_publish_ms = 0;
    if (now - last_ble_publish_ms >= 2000) {
        last_ble_publish_ms = now;
        ble::publish();
    }
#endif

    // Periodic debug dump on serial -- once every 30 s.
    static uint32_t last_dump_ms = 0;
    if (now - last_dump_ms > 30000) {
        last_dump_ms = now;
        Serial.printf(
            "[STAT] uptime=%lus total_in=%lu total_out=%lu "
            "interval_in_live=%lu interval_out_live=%lu glitches=%u status=0x%02X\n",
            (unsigned long)(now / 1000),
            (unsigned long)g_total_in,
            (unsigned long)g_total_out,
            (unsigned long)g_interval_in_live,
            (unsigned long)g_interval_out_live,
            (unsigned)g_glitch_count,
            (unsigned)g_status_flags
        );
    }
}
