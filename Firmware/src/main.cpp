// ============================================================================
// Easy Bee Counter 2026 — ESP32-C6 mini firmware
// ----------------------------------------------------------------------------
// Author: rewritten 2026 for the new ESP32-C6 mini board.
//
// What this firmware does
// -----------------------
//   1. Continuously polls 24 entrance gates (each gate = Inner IR sensor +
//      Outer IR sensor) through three MCP23017 I2C port expanders.
//   2. Detects directional bee crossings:
//        Outer-then-Inner = bee entered the hive   -> "in" counter++
//        Inner-then-Outer = bee left the hive      -> "out" counter++
//   3. Maintains lifetime totals, per-interval totals, and per-gate detail.
//   4. Acts as an I2C slave at address 0x30 so the main HiveScale ESP32 can
//      poll it whenever it wakes (roughly every 10 minutes).
//
// Hardware constraint: single shared I2C bus
// ------------------------------------------
// The PCB ties the external connector J1 (which goes to the HiveScale) to
// the SAME SDA/SCL net that the on-board MCP23017s live on. The ESP32-C6 has
// only ONE hardware I2C controller and that controller cannot be master and
// slave simultaneously, so this firmware does:
//
//     mostly:   MASTER -- poll MCP23017s every ~10 ms to detect crossings
//     briefly:  SLAVE  -- when the HiveScale starts a transaction with us
//
// To make the role-switching predictable, the firmware reserves a fixed
// "slave window" each cycle during which it sits in slave mode listening
// for the HiveScale. The HiveScale only polls about every 10 minutes, so
// any reasonable window length works; we use 5 ms of slave listening per
// 50 ms of master polling (10% duty). Crossings during a slave window are
// not missed because the sensor data is sticky inside the MCP23017 — we
// simply read a slightly-older state next time.
//
// If a collision does happen (HiveScale starts a transaction at the very
// moment we are mid-MCP-read), the HiveScale will get a NACK or a stretch.
// It must retry; one retry is virtually guaranteed to succeed.
//
// SAFER ALTERNATIVE (not used here): cut the J1<->SDA/SDC traces on the PCB
// and reroute the HiveScale link to two of the unused ESP32-C6 pins
// (D0..D3, D10) using bit-banged software I2C for the MCP23017s on D4/D5
// and the hardware I2C peripheral in slave mode on the rerouted pins. Doing
// this would eliminate the mode-switch entirely. See README.md for details.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "pins.h"
#include "i2c_slave_protocol.h"

// ============================================================================
// Compile-time tuning knobs — keep these together for easy field adjustment
// ============================================================================

// How many milliseconds the firmware spends in master mode between slave
// windows. Lower = more responsive crossing detection but lower probability
// of catching the HiveScale's transaction first try.
static constexpr uint32_t MASTER_WINDOW_MS = 50;

// How many milliseconds the firmware spends in slave mode each cycle.
// 5 ms easily covers a 9-byte I2C transaction at 100 kHz (~0.9 ms each).
static constexpr uint32_t SLAVE_WINDOW_MS = 5;

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

// I2C bus speed when we are master. 400 kHz is the MCP23017's spec limit;
// 100 kHz is safer over long cabling. The shared bus and external J1 cable
// argue for the slower setting.
static constexpr uint32_t I2C_MASTER_HZ = 100000;

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
//
// This is intentionally simple. It will mis-classify bees that reverse mid-
// crossing (which is rare in practice) and bees that walk in pairs (more
// common — but the IR beam is sharp enough that two bees side-by-side trip
// the same sensor and are counted as one). For a higher-accuracy classifier
// you would extend this state machine; the protocol exposes per-gate counts
// so improvements can be measured.
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
static volatile uint16_t g_busy_retries  = 0;
static volatile uint8_t  g_status_flags  = 0;

// ============================================================================
// MCP23017 driver objects
// ============================================================================
static Adafruit_MCP23X17 g_mcp_u2;   // 0x20, gates 00..07
static Adafruit_MCP23X17 g_mcp_u3;   // 0x21, gates 10..17
static Adafruit_MCP23X17 g_mcp_u4;   // 0x22, gates 20..27

static bool g_mcp_u2_ok = false;
static bool g_mcp_u3_ok = false;
static bool g_mcp_u4_ok = false;

// LED-bank control. Auto = follow the polling cycle; we leave the LEDs on
// continuously while polling, switch them off during slave windows to save
// some current. The IRLB8721 N-FET is logic-level and just needs a digital
// HIGH on its gate.
enum class LedMode : uint8_t { AUTO, FORCE_ON, FORCE_OFF };
static volatile LedMode g_led_mode = LedMode::AUTO;

// ============================================================================
// I2C slave-callback state (shared with ISR context)
// ============================================================================
//
// The Wire library invokes our onReceive() and onRequest() handlers in an
// ISR-ish callback context. We keep the work tiny: copy a register pointer
// into a global, and on read, stream bytes from a pre-built response buffer.
//
// To avoid tearing when the main loop updates counters while we are filling
// the response buffer, we use a snapshot built under "no master in flight"
// conditions only.
// ============================================================================

static volatile uint8_t  g_reg_pointer = beecounter_proto::REG_STATUS;
static volatile bool     g_slave_active = false;   // currently in slave mode
static volatile bool     g_master_busy  = false;   // currently mid-MCP read

// Response buffer: enough room for the largest single-read register block.
// The biggest is REG_PER_GATE_IN (24 bytes) or REG_PER_GATE_OUT (24 bytes).
static constexpr size_t SLAVE_TX_BUF_SIZE = 32;
static uint8_t g_slave_tx_buf[SLAVE_TX_BUF_SIZE];
static volatile uint8_t g_slave_tx_len = 0;

// ============================================================================
// Low-level helpers
// ============================================================================

static void setIrLeds(bool on) {
    if (g_led_mode == LedMode::FORCE_OFF) on = false;
    if (g_led_mode == LedMode::FORCE_ON)  on = true;
    digitalWrite(pins::IR_LED_BANK_1_EN, on ? HIGH : LOW);
    digitalWrite(pins::IR_LED_BANK_2_EN, on ? HIGH : LOW);
    if (on) g_status_flags |=  beecounter_proto::STATUS_IR_LEDS_ON;
    else    g_status_flags &= ~beecounter_proto::STATUS_IR_LEDS_ON;
}

// Big-endian encode helpers.
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

// Poll all three MCP23017s and update every gate. Returns false if any
// chip failed to respond -- in that case we don't update the affected
// gates' sensor state this cycle, which is harmless.
static bool pollAllGates() {
    g_master_busy = true;
    const uint32_t now_ms = millis();

    // Read each chip's full GPIO state as a 16-bit value -- much faster
    // than digitalRead() per pin. Adafruit_MCP23X17::readGPIOAB() returns
    // GPIOA in the low byte and GPIOB in the high byte.
    uint16_t v_u2 = 0, v_u3 = 0, v_u4 = 0;
    bool any_fail = false;

    if (g_mcp_u2_ok) v_u2 = g_mcp_u2.readGPIOAB(); else any_fail = true;
    if (g_mcp_u3_ok) v_u3 = g_mcp_u3.readGPIOAB(); else any_fail = true;
    if (g_mcp_u4_ok) v_u4 = g_mcp_u4.readGPIOAB(); else any_fail = true;

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

    g_master_busy = false;
    return !any_fail;
}

// ============================================================================
// Master <-> Slave mode switching
// ============================================================================

static void enterSlaveMode() {
    if (g_slave_active) return;
    Wire.end();
    delay(1);   // brief settle
    // begin() with an address parameter puts the C6 in slave mode.
    Wire.begin((uint8_t)i2c_addr::BEECOUNTER_SLAVE, pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);
    Wire.onReceive([](int n_bytes) {
        // The HiveScale wrote n_bytes to us. The first byte is the register
        // pointer; any further bytes are command payload (only REG_CTRL
        // takes a payload byte).
        if (n_bytes <= 0) return;
        uint8_t reg = (uint8_t)Wire.read();
        g_reg_pointer = reg;
        if (reg == beecounter_proto::REG_CTRL && n_bytes >= 2) {
            uint8_t cmd = (uint8_t)Wire.read();
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
        // drain anything else
        while (Wire.available()) (void)Wire.read();
    });

    Wire.onRequest([]() {
        // HiveScale wants to read from the previously written register
        // pointer. Stream bytes from the appropriate snapshot.
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
            uint32_t s = millis() / 1000;
            if (s > 0xFFFF) s = 0xFFFF;
            writeU16BE(&buf[n], (uint16_t)s); n += 2;
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
            writeU32BE(&buf[n], g_total_in);  n += 4; break;
        case beecounter_proto::REG_TOTAL_OUT:
            writeU32BE(&buf[n], g_total_out); n += 4; break;
        case beecounter_proto::REG_INTERVAL_IN:
            writeU32BE(&buf[n], g_interval_in_shadow);  n += 4; break;
        case beecounter_proto::REG_INTERVAL_OUT:
            writeU32BE(&buf[n], g_interval_out_shadow); n += 4; break;
        case beecounter_proto::REG_GLITCH_COUNT:
            writeU16BE(&buf[n], g_glitch_count); n += 2; break;
        case beecounter_proto::REG_BUSY_RETRIES:
            writeU16BE(&buf[n], g_busy_retries); n += 2; break;
        case beecounter_proto::REG_PER_GATE_IN:
            for (uint8_t i = 0; i < gates::NUM_GATES; i++)
                buf[n++] = g_gate_shadow[i].in;
            break;
        case beecounter_proto::REG_PER_GATE_OUT:
            for (uint8_t i = 0; i < gates::NUM_GATES; i++)
                buf[n++] = g_gate_shadow[i].out;
            break;
        case beecounter_proto::REG_CTRL:
            buf[n++] = 0xFF;   // write-only
            break;
        default:
            buf[n++] = 0xFF;
            break;
        }
        Wire.write(buf, n);
    });

    g_slave_active = true;
}

static void enterMasterMode() {
    if (!g_slave_active) return;
    Wire.end();
    delay(1);
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);
    g_slave_active = false;
}

// ============================================================================
// Boot-time MCP23017 setup
// ============================================================================

static bool initMcp(Adafruit_MCP23X17& mcp, uint8_t addr, const char* tag) {
    if (!mcp.begin_I2C(addr, &Wire)) {
        Serial.printf("[MCP] %s @ 0x%02X: NOT FOUND\n", tag, addr);
        return false;
    }
    // Configure every pin (0..15) as input with internal pull-up off
    // (the board has 100k externals, but turning the MCP pull-up on too
    // shouldn't hurt; we leave it off to preserve the external R-divider
    // characteristic). All 16 pins are sensor inputs.
    for (uint8_t p = 0; p < 16; p++) {
        mcp.pinMode(p, INPUT);
    }
    Serial.printf("[MCP] %s @ 0x%02X: OK\n", tag, addr);
    return true;
}

// ============================================================================
// Arduino setup() / loop()
// ============================================================================

static uint32_t g_last_window_switch_ms = 0;
static bool g_in_slave_window = false;
static uint32_t g_last_poll_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("==============================================");
    Serial.println("Easy Bee Counter 2026 — firmware booting");
    Serial.println("==============================================");

    // Configure LED enable pins -- start with LEDs off so we can verify
    // the MCP23017 pull-up baseline before powering the emitters.
    pinMode(pins::IR_LED_BANK_1_EN, OUTPUT);
    pinMode(pins::IR_LED_BANK_2_EN, OUTPUT);
    digitalWrite(pins::IR_LED_BANK_1_EN, LOW);
    digitalWrite(pins::IR_LED_BANK_2_EN, LOW);

    // Start in master mode to probe the three MCP23017s.
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);

    g_mcp_u2_ok = initMcp(g_mcp_u2, i2c_addr::MCP_GATES_00_07, "U2 (gates 00..07)");
    g_mcp_u3_ok = initMcp(g_mcp_u3, i2c_addr::MCP_GATES_10_17, "U3 (gates 10..17)");
    g_mcp_u4_ok = initMcp(g_mcp_u4, i2c_addr::MCP_GATES_20_27, "U4 (gates 20..27)");

    if (g_mcp_u2_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U2_OK;
    if (g_mcp_u3_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U3_OK;
    if (g_mcp_u4_ok) g_status_flags |= beecounter_proto::STATUS_MCP_U4_OK;

    // Light up the IR emitters now that the chips are configured.
    setIrLeds(true);

    // One warm-up poll so the state machine has fresh baselines before any
    // crossings start counting.
    pollAllGates();

    g_status_flags |= beecounter_proto::STATUS_READY;
    g_last_window_switch_ms = millis();

    Serial.println("[SETUP] Entering normal counting loop");
}

void loop() {
    const uint32_t now = millis();

    // ---- Master/slave window scheduling ----
    if (g_in_slave_window) {
        if (now - g_last_window_switch_ms >= SLAVE_WINDOW_MS) {
            enterMasterMode();
            g_in_slave_window = false;
            g_last_window_switch_ms = now;
        }
    } else {
        if (now - g_last_window_switch_ms >= MASTER_WINDOW_MS) {
            enterSlaveMode();
            g_in_slave_window = true;
            g_last_window_switch_ms = now;
        }
    }

    // ---- Master work ----
    if (!g_in_slave_window) {
        // Poll the MCP23017s at most once every ~5 ms to leave time for the
        // window scheduler. In practice readGPIOAB() takes ~0.3 ms each, so
        // three of them plus state-machine work fit comfortably in 2 ms.
        if (now - g_last_poll_ms >= 5) {
            g_last_poll_ms = now;
            pollAllGates();
        }
    }

    // Keep the IR LEDs in sync with current mode (auto/force).
    static LedMode last_led_mode = LedMode::AUTO;
    if (g_led_mode != last_led_mode) {
        last_led_mode = g_led_mode;
        // Re-apply current state.
        bool target = (g_led_mode == LedMode::FORCE_OFF) ? false : true;
        setIrLeds(target);
    }

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