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
//   5. Suspends 1-3 on request (night mode), so the emitters — which dominate
//      this board's power draw by an order of magnitude — are dark through the
//      hours European honey bees do not fly. See the section below.
//   6. Runs on a subset of its three emitter banks on request, so an entrance
//      narrower than 24 gates — or a supply that will not carry 24 — costs only
//      the banks it uses. See the section below.
//
// Emitter bank enables (protocol v5)
// ----------------------------------
// The 2026-08 revision put one IRLB8721 behind each MCP23017, which makes each
// bank independently switchable: bank 1 = U2 = gates 00..07, bank 2 = U3 =
// 10..17, bank 3 = U4 = 20..27. Measured on the 3.3 V rail with the pulsed
// sampler at its defaults, one bank draws ~0.14 A, two ~0.22 A and three
// ~0.30 A, so dropping a bank is worth roughly 80 mA continuously — about what
// a quarter of a night of night mode saves, except it applies all day.
//
// This is a configuration rather than a deadline, so unlike night mode it has
// no expiry; like night mode it is deliberately NOT persisted, and HiveHub
// re-asserts it every upload cycle. A reset therefore comes back counting on
// all 24 gates. The mask arithmetic lives in include/bank_state.h.
//
// The gates of a dark bank are SKIPPED, not merely unlit. An unpowered QRE1113
// is a bare phototransistor under a 100k pull-up, and direct sun into a hive
// entrance can pull one low; feeding those readings to the state machine would
// invent crossings on gates the operator switched off.
//
// Night mode (protocol v4)
// ------------------------
// Honey bees are diurnal: flight needs light, and stops below roughly 10 C
// regardless. Sampling 24 gates through the night therefore burns the largest
// item in the power budget to count nothing. HiveHub — which knows the time,
// and this board does not — writes a suspension DURATION to the control
// characteristic once per upload cycle for as long as its configured night
// window lasts; idle_state.h turns that into a deadline and this file stops
// pulsing and polling until it expires.
//
// It is deliberately NOT deep sleep. The emitters are >90% of the draw, so
// parking them captures essentially the whole saving, while staying awake keeps
// the counter readable, updatable and cancellable all night — none of which
// survives deep sleep — and keeps the lifetime totals in RAM where they belong.
// See docs/ble-mode.md for the reasoning and the numbers.
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
#include "gate_logic.h"
#include "idle_state.h"
#include "bank_state.h"

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

// Minimum time a sensor must hold a NEW value before that value is accepted as
// the debounced state. See gate_logic.h for how the debounce is implemented and
// why the previous version did not actually enforce this.
//
// This has to be read together with SENSOR_DEBOUNCE_SAMPLES below and with
// POLL_INTERVAL_MS: a threshold shorter than one poll period cannot reject
// anything, because the first differing sample already satisfies it. The old
// 3 ms value was exactly that case (3 < 5), which is why the sample count is
// the load-bearing half of the rule at the current poll rate.
static constexpr uint32_t SENSOR_DEBOUNCE_MS = 5;

// How many consecutive polls must agree before a sensor change is accepted.
//
// Two is the most the beam physics allow here. A bee crossing at ~25 cm/s
// dwells ~12 ms in a 3 mm beam, i.e. 2-3 samples at POLL_INTERVAL_MS=5, so
// requiring three consecutive agreeing samples (~10 ms) would start dropping
// real crossings. Two rejects the single corrupted or noisy word — the failure
// mode the old code was blind to — while accepting a genuine block ~5 ms after
// it first appears, comfortably inside the dwell.
//
// If you ever raise POLL_INTERVAL_MS, this must stay at 2 (or the debounce
// window grows past the dwell and crossings are silently lost); if you lower it
// well below 5 ms, raise this instead of SENSOR_DEBOUNCE_MS.
static constexpr uint8_t SENSOR_DEBOUNCE_SAMPLES = 2;

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
// returned to CLEAR (each having passed the debounce rule above).
//
// The debounce and the state machine themselves live in include/gate_logic.h,
// free of Arduino dependencies, so the timing rules can be exercised by the
// host-side tests in test/test_gate_logic/ instead of only on a live hive.
// ============================================================================

static gatelogic::GateRuntime g_gate_rt[gates::NUM_GATES];

static constexpr gatelogic::Tuning GATE_TUNING = {
    SENSOR_DEBOUNCE_MS,
    SENSOR_DEBOUNCE_SAMPLES,
    GATE_PAIRING_WINDOW_MS,
    SENSOR_STUCK_MS,
};

// Aggregate counters. LIFETIME totals only: the BLE contract is totals-only and
// HiveHub derives each interval by differencing consecutive reads, so a missed
// connection can never lose traffic. (The wired path additionally kept
// per-interval and per-gate counters, zeroed by CMD_LATCH; both went with it.)
static volatile uint32_t g_total_in  = 0;
static volatile uint32_t g_total_out = 0;

// 32-bit since protocol v3. It saturates rather than wrapping either way, but
// as a uint16_t a noisy device pinned at 65535 was reporting a bounded number
// that said nothing about how bad things had got.
static volatile uint32_t g_glitch_count  = 0;
static volatile uint8_t  g_status_flags  = 0;

// Night-mode suspension. Owned here because this is where the emitters and the
// poll loop live; armed from the BLE control characteristic through
// ble::applyIdleRequest() below. Never persisted: a reset resumes counting.
static idlestate::State g_idle;

// Which emitter banks are allowed to light. Owned here for the same reason
// g_idle is — this file drives the FET gates — and armed from the BLE control
// characteristic through ble::applyBankMask() below. Never persisted: a reset
// comes back with all three banks counting, and HiveHub re-asserts the mask on
// its next upload cycle. See bank_state.h.
static bankstate::State g_banks;

// ============================================================================
// MCP23017 channels + runtime health  (all on Wire / bus 0)
// ============================================================================
//
// Health used to be decided once, in setup(), and never revisited. That made
// two failure modes invisible:
//
//   * a chip absent at boot could never come back without a power cycle, and
//   * a chip that died AFTER boot kept being reported healthy, while its port
//     word read as all-zeros — which this firmware interprets as "every beam
//     blocked" — so its eight gates silently pinned into PAIRED, inflated the
//     glitch tally and eventually latched the sensor-fault flag.
//
// So each chip now carries a small health record: consecutive failures (one
// transient NAK is not a dead chip), a validity flag for the current poll's
// snapshot, and a retry deadline. Gates are only fed from a snapshot marked
// valid, and the STATUS_MCP_U*_OK bits plus the mcps_healthy telemetry now track
// the live state instead of boot-time discovery.
static constexpr uint8_t NUM_MCP = 3;

// Consecutive failed reads before a chip is declared unhealthy. Its gates are
// skipped from the first failure regardless; this only controls when we stop
// trusting the chip altogether, tear its gates down and start re-probing.
static constexpr uint8_t MCP_FAIL_THRESHOLD = 3;

// How often an unhealthy chip is re-probed. Each attempt is one addressed
// transaction that a missing chip NAKs immediately, so this is cheap; it runs
// before the emitters are lit so a failed probe never extends the LED pulse.
static constexpr uint32_t MCP_RETRY_INTERVAL_MS = 5000;

// MCP23017 register address of GPIOA with IOCON.BANK=0 (the power-on default,
// which the Adafruit driver leaves alone). GPIOB follows at 0x13 and the chip's
// sequential-read mode returns both from one 2-byte read — the same access
// readGPIOAB() performs, but issued directly so the I2C transaction result is
// actually visible to us. readGPIOAB() returns a bare uint16_t and reports a
// failed bus transaction as 0x0000, i.e. as "all sensors blocked".
static constexpr uint8_t MCP23X17_REG_GPIOA = 0x12;

static Adafruit_MCP23X17 g_mcp_dev[NUM_MCP];

struct McpHealth {
    uint8_t     addr;
    uint8_t     status_bit;
    const char* tag;
    bool        ok;             // chip is believed present and answering
    uint8_t     fail_streak;    // consecutive failed reads
    uint32_t    next_retry_ms;  // when to re-probe while !ok
    uint16_t    value;          // this poll's port word (GPIOA | GPIOB << 8)
    bool        valid;          // is `value` from a successful read this poll?
};

static McpHealth g_mcp[NUM_MCP] = {
    { i2c_addr::MCP_GATES_00_07, beecounter_proto::STATUS_MCP_U2_OK,
      "U2 (gates 00..07)", false, 0, 0, 0, false },
    { i2c_addr::MCP_GATES_10_17, beecounter_proto::STATUS_MCP_U3_OK,
      "U3 (gates 10..17)", false, 0, 0, 0, false },
    { i2c_addr::MCP_GATES_20_27, beecounter_proto::STATUS_MCP_U4_OK,
      "U4 (gates 20..27)", false, 0, 0, 0, false },
};

// gates::TABLE identifies a chip by I2C address; the health records are indexed.
static int8_t mcpIndexForAddress(uint8_t addr) {
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        if (g_mcp[i].addr == addr) return (int8_t)i;
    }
    return -1;
}

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
    // A disabled bank's FET gate is held LOW whatever `on` says: this is the
    // one place that decides whether an emitter rail is ever energised, so
    // enforcing the mask here means no other path — the pulsed sampler, the
    // FORCE_ON debug mode, the night-mode backstop — can light a bank the
    // operator switched off.
    for (uint8_t b = 0; b < pins::NUM_LED_BANKS; b++) {
        const bool live = on && bankstate::enabled(g_banks, (uint8_t)(b + 1));
        digitalWrite(pins::IR_LED_BANK_EN[b], live ? HIGH : LOW);
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
// MCP23017 acquisition + health
// ============================================================================

static bool initMcp(Adafruit_MCP23X17& mcp, uint8_t addr, const char* tag,
                    bool quiet = false) {
    if (!mcp.begin_I2C(addr, &Wire)) {
        if (!quiet) Serial.printf("[MCP] %s @ 0x%02X: NOT FOUND\n", tag, addr);
        return false;
    }
    // All 16 pins are sensor inputs. The board provides 100k pull-ups, so we
    // use plain INPUT (not INPUT_PULLUP) to keep the MCP's weak internal
    // pull-ups out of the picture.
    for (uint8_t p = 0; p < 16; p++) {
        mcp.pinMode(p, INPUT);
    }
    if (!quiet) Serial.printf("[MCP] %s @ 0x%02X: OK\n", tag, addr);
    return true;
}

// Drop every gate on this chip back to a clean IDLE. Called whenever a chip
// changes health in either direction, so that a half-finished pairing from
// before an outage cannot combine with a sensor reading from after it and
// fabricate a crossing. The counters are lifetime totals and are untouched.
static void resetGatesForMcp(uint8_t addr) {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        if (gates::TABLE[i].mcp_address == addr) {
            g_gate_rt[i] = gatelogic::GateRuntime();
        }
    }
}

// Drop every gate on one emitter bank back to IDLE. Called when a bank is
// switched off (its gates stop being sampled, so a half-finished pairing would
// otherwise sit there indefinitely) and when one is switched back on (the
// pairing predates however long the bank was dark). Same reasoning as
// resetGatesForMcp() across a chip outage; the lifetime totals are untouched.
static void resetGatesForBank(uint8_t bank) {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        if (gates::TABLE[i].led_bank == bank) {
            g_gate_rt[i] = gatelogic::GateRuntime();
        }
    }
}

// Drop every gate back to IDLE. Used on both edges of a night-mode suspension:
// a gate that was half-way through a pairing when sensing stopped must not
// combine that stale half with the first sample taken hours later and fabricate
// a crossing. Same reasoning as resetGatesForMcp() across a chip outage — the
// lifetime totals are untouched either way.
static void resetAllGates() {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        g_gate_rt[i] = gatelogic::GateRuntime();
    }
}

// One checked 2-byte read of GPIOA/GPIOB. Unlike readGPIOAB() this reports bus
// failures instead of silently returning 0x0000 ("all blocked").
static bool readMcpChecked(uint8_t idx, uint16_t& out) {
    const uint8_t addr = g_mcp[idx].addr;
    Wire.beginTransmission(addr);
    Wire.write(MCP23X17_REG_GPIOA);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    if (Wire.requestFrom((int)addr, (int)2) != 2) return false;
    const uint8_t a = (uint8_t)Wire.read();   // GPIOA -> inner sensors
    const uint8_t b = (uint8_t)Wire.read();   // GPIOB -> outer sensors
    out = (uint16_t)a | ((uint16_t)b << 8);
    return true;
}

// Fold one poll's read result into a chip's health record.
static void noteMcpResult(uint8_t idx, bool read_ok, uint32_t now_ms) {
    McpHealth& m = g_mcp[idx];
    if (read_ok) {
        m.fail_streak = 0;
        return;
    }
    if (m.fail_streak < 0xFF) m.fail_streak++;
    if (m.ok && m.fail_streak >= MCP_FAIL_THRESHOLD) {
        m.ok = false;
        m.valid = false;
        g_status_flags &= (uint8_t)~m.status_bit;
        m.next_retry_ms = now_ms + MCP_RETRY_INTERVAL_MS;
        resetGatesForMcp(m.addr);
        Serial.printf("[MCP] %s @ 0x%02X: lost after %u failed reads\n",
                      m.tag, m.addr, (unsigned)m.fail_streak);
    }
}

// Does anything ACK at this address? One addressed byte, no data phase.
static bool mcpPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Re-probe an unhealthy chip, at most once per MCP_RETRY_INTERVAL_MS. Runs with
// the emitters dark, before the sampling window.
static void retryMcp(uint8_t idx, uint32_t now_ms) {
    McpHealth& m = g_mcp[idx];
    if (m.ok) return;
    if ((int32_t)(now_ms - m.next_retry_ms) < 0) return;
    m.next_retry_ms = now_ms + MCP_RETRY_INTERVAL_MS;

    // Cheap ACK probe first, so the full re-init only runs when the chip is
    // genuinely back. begin_I2C() reallocates the driver's bus object; gating
    // it on presence keeps a permanently absent chip from churning that
    // allocation every retry for the life of the device.
    if (!mcpPresent(m.addr)) return;

    // quiet: a chip that ACKs but fails to configure would otherwise log on
    // every retry.
    if (!initMcp(g_mcp_dev[idx], m.addr, m.tag, /*quiet=*/true)) return;

    m.ok = true;
    m.fail_streak = 0;
    g_status_flags |= m.status_bit;
    resetGatesForMcp(m.addr);
    Serial.printf("[MCP] %s @ 0x%02X: recovered\n", m.tag, m.addr);
}

// ============================================================================
// Crossing detection
// ============================================================================

// Saturating lifetime totals: see gatelogic::countSaturating. The flag is
// raised as soon as a counter reaches its maximum, and it stays pinned there
// rather than wrapping to zero behind HiveHub's back.
static void countCrossing(volatile uint32_t& total) {
    if (gatelogic::countSaturating(total)) {
        g_status_flags |= beecounter_proto::STATUS_OVERFLOW_FLAG;
    }
}

// Read every healthy MCP23017 into its health record. The emitters must
// already be lit and settled before this is called.
//
// Each record's `valid` flag says whether THIS poll's word can be trusted. A
// chip that is unhealthy, or that failed its read this cycle, is left invalid
// and its gates are skipped entirely — the old code left the word at zero and
// fed it to the state machine anyway, which reads as "all 16 beams blocked".
//
// Returns false if any chip did not produce a usable word.
static bool readAllMcp(uint32_t now_ms) {
    bool all_ok = true;
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        McpHealth& m = g_mcp[i];
        m.valid = false;
        m.value = 0;
        if (!m.ok) {
            all_ok = false;
            continue;
        }
        uint16_t v = 0;
        const bool read_ok = readMcpChecked(i, v);
        if (read_ok) {
            m.value = v;
            m.valid = true;
        } else {
            all_ok = false;
        }
        noteMcpResult(i, read_ok, now_ms);
    }
    return all_ok;
}

// Acquire one fresh set of sensor readings with the emitters pulsed on only for
// the settle + read window. In FORCE_ON the emitters are already steady-on, so
// we skip the extra toggling. In FORCE_OFF we never light them (readings will
// look "clear"), preserving the diagnostic meaning of that mode.
static bool sampleGates(uint32_t now_ms) {
    switch (g_led_mode) {
    case LedMode::FORCE_OFF:
        // Emitters stay dark; read whatever the sensors show (nominally clear).
        return readAllMcp(now_ms);

    case LedMode::FORCE_ON:
        // Emitters are already steady-on (set when the mode was entered); just
        // read. No per-sample toggling so a scope trace shows a clean DC level.
        return readAllMcp(now_ms);

    case LedMode::AUTO:
    default:
        // Pulsed path: light all banks, let the phototransistors settle, read
        // across the lit window, then black the emitters out again.
        driveIrLeds(true);
        delayMicroseconds(LED_SETTLE_US);
        {
            bool ok = readAllMcp(now_ms);
            driveIrLeds(false);
            return ok;
        }
    }
}

// Poll all three MCP23017s and update every gate whose snapshot is trustworthy.
// Returns false if any chip did not produce a usable word this cycle; gates on
// such a chip are skipped rather than fed a fabricated all-blocked reading.
static bool pollAllGates() {
    const uint32_t now_ms = millis();

    // Re-probe anything currently marked unhealthy. Done first, with the
    // emitters still dark, so a probe never lengthens the LED pulse.
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        retryMcp(i, now_ms);
    }

    // Acquire a fresh sensor snapshot with the emitters pulsed (AUTO) or steady
    // (FORCE_ON) per the current LED mode. Each chip's word is GPIOA in the low
    // byte and GPIOB in the high byte.
    const bool ok = sampleGates(now_ms);

    // Named getBit to avoid clashing with Arduino.h's bit(b) macro.
    auto getBit = [](uint16_t v, uint8_t pin) -> bool {
        return (v >> pin) & 0x1;
    };

    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        const auto& loc = gates::TABLE[i];
        // A gate on a disabled bank is skipped, not read as "clear": its
        // emitters are dark, so the phototransistor is only reporting ambient
        // light, and sun into the entrance would fabricate crossings on gates
        // the operator deliberately switched off. The chip itself is still read
        // above, so mcps_healthy keeps meaning "expanders answering".
        if (!bankstate::enabled(g_banks, loc.led_bank)) continue;
        const int8_t ch = mcpIndexForAddress(loc.mcp_address);
        if (ch < 0 || !g_mcp[ch].valid) continue;   // no trustworthy sample
        const uint16_t v = g_mcp[ch].value;

        // QRE1113 phototransistor: BLOCKED -> sensor line LOW (bit=0).
        // We pass "blocked = (bit == 0)" into the state machine.
        const bool inner_blocked = !getBit(v, loc.inner_pin);
        const bool outer_blocked = !getBit(v, loc.outer_pin);

        const gatelogic::GateUpdate u = gatelogic::updateGate(
            g_gate_rt[i], inner_blocked, outer_blocked, now_ms, GATE_TUNING);

        switch (u.event) {
        case gatelogic::GateEvent::BEE_IN:  countCrossing(g_total_in);  break;
        case gatelogic::GateEvent::BEE_OUT: countCrossing(g_total_out); break;
        case gatelogic::GateEvent::GLITCH:
            // Saturating: a wrapping diagnostic counter can make a badly
            // unhealthy device look healthier than it did on the last read.
            gatelogic::countSaturating(g_glitch_count);
            break;
        case gatelogic::GateEvent::NONE:
        default:
            break;
        }

        if (u.sensor_stuck) {
            g_status_flags |= beecounter_proto::STATUS_SENSOR_FAULT_FLAG;
        }
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
//   n  arm a 60 s night-mode suspension (press again to resume)
//   4  toggle emitter bank 1 (gates 00..07)
//   5  toggle emitter bank 2 (gates 10..17)
//   6  toggle emitter bank 3 (gates 20..27)
//   h  print the command list
//
// The bank keys are 4/5/6 because the schematic calls those rails /GPIO4,
// /GPIO5 and /GPIO6 — misleading net names (they are physically GPIO19/20/18,
// see pins.h) but the labels silkscreened next to the FETs, which is what
// someone with a probe in one hand is actually reading.
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
    Serial.println(F("  n  arm/clear a 60 s night-mode suspension"));
    Serial.println(F("  4  toggle emitter bank 1 (gates 00..07)"));
    Serial.println(F("  5  toggle emitter bank 2 (gates 10..17)"));
    Serial.println(F("  6  toggle emitter bank 3 (gates 20..27)"));
    Serial.println(F("  h  show this help"));
    Serial.println();
}

// Take one fresh reading of all 24 gates and print it. The emitters are pulsed
// on for the settle+read window regardless of the current LED mode, so the
// readout is always valid on the bench even in AUTO/FORCE_OFF; afterwards the
// mode-correct steady level is restored.
static void irDebugReadAndPrint() {
    const uint32_t now_ms = millis();
    driveIrLeds(true);
    delayMicroseconds(LED_SETTLE_US);
    bool ok = readAllMcp(now_ms);
    setIrLeds(true);   // restore mode-correct steady level (OFF in AUTO)

    auto getBit = [](uint16_t v, uint8_t pin) -> bool { return (v >> pin) & 0x1; };

    Serial.printf("[IR] t=%lus raw U2=0x%04X U3=0x%04X U4=0x%04X read_ok=%d "
                  "banks=0x%02X\n",
                  (unsigned long)(now_ms / 1000), g_mcp[0].value, g_mcp[1].value,
                  g_mcp[2].value, ok ? 1 : 0, (unsigned)g_banks.mask);
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        const auto& loc = gates::TABLE[i];
        const int8_t ch = mcpIndexForAddress(loc.mcp_address);
        if (ch < 0) continue;
        if (!g_mcp[ch].valid) {
            // Printing "clear" here would be a lie: an unread chip has no state.
            Serial.printf("  %-8s bank:%u  <no reading: %s>\n", loc.tag,
                          (unsigned)loc.led_bank,
                          g_mcp[ch].ok ? "read failed" : "chip unhealthy");
            continue;
        }
        const uint16_t v = g_mcp[ch].value;
        if (!bankstate::enabled(g_banks, loc.led_bank)) {
            // The chip answered, but this gate's emitters are switched off, so
            // whatever the phototransistor says is ambient light and not a
            // beam state. Saying "clear" here would be the same lie as saying
            // it for a chip that failed to read.
            Serial.printf("  %-8s bank:%u  <bank disabled>\n", loc.tag,
                          (unsigned)loc.led_bank);
            continue;
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
        case 'n': {
            // Exercise the night-mode path without a HiveHub: 60 s is long
            // enough to watch the emitters go dark and the counts freeze, short
            // enough that a forgotten bench board resumes on its own.
            const bool idle_now = !idlestate::sensing(g_idle);
            const uint32_t granted = ble::applyIdleRequest(idle_now ? 0 : 60);
            Serial.printf("[IR-DEBUG] night mode %s (%lus)\n",
                          granted ? "ARMED" : "cleared",
                          (unsigned long)granted);
            break;
        }
        case '4':
        case '5':
        case '6': {
            // Toggle one bank. A refused request (the last bank going off)
            // reports itself from applyBankMask(), so nothing extra is needed
            // here to explain why the mask did not move.
            const uint8_t bank = (uint8_t)(c - '3');   // '4' -> 1
            const uint8_t bit  = bankstate::bankBit(bank);
            const uint8_t want = (uint8_t)(g_banks.mask ^ bit);
            const uint8_t got  = ble::applyBankMask(want);
            Serial.printf("[IR-DEBUG] bank %u %s — mask 0x%02X\n",
                          (unsigned)bank,
                          (got & bit) ? "ENABLED" : "disabled",
                          (unsigned)got);
            break;
        }
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
    // No clamp since protocol v3: uptime_s is a uint32_t on both sides of the
    // link. millis() itself rolls over at ~49.7 days, so this still restarts
    // then — the field reports time since the last millis() epoch, which is
    // what an unexpected reset shows up in. (A 32-bit second counter would run
    // ~136 years; the rollover, not the width, is now the limit.)
    const uint32_t up = millis() / 1000;
    // Live health, not the boot-time snapshot this used to report: a chip that
    // died after setup() now shows up here (and in the STATUS_MCP_U*_OK bits),
    // and one that recovers is counted again.
    uint8_t healthy = 0;
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        if (g_mcp[i].ok) healthy++;
    }
    // Derived here, not read straight out of g_status_flags, so the bit and the
    // countdown can never disagree in a published document: the deadline can
    // pass between a GATT read and the loop() pass that services the expiry,
    // and "STATUS_NIGHT_IDLE set, idle_s 0" is a contradiction HiveHub would
    // have to guess its way out of.
    const uint32_t idle_left = idlestate::remainingSeconds(g_idle, millis());
    const uint8_t status = idle_left
        ? (uint8_t)(g_status_flags | beecounter_proto::STATUS_NIGHT_IDLE)
        : (uint8_t)(g_status_flags & ~beecounter_proto::STATUS_NIGHT_IDLE);

    t.protocol_version = beecounter_proto::PROTOCOL_VERSION;
    t.status_flags     = status;
    t.uptime_s         = up;
    t.num_gates        = gates::NUM_GATES;
    t.mcps_healthy     = healthy;
    t.total_in         = g_total_in;
    t.total_out        = g_total_out;
    t.glitch_count     = g_glitch_count;
    t.idle_s           = idle_left;
    // Reported unconditionally, including the 0x07 of a counter nobody has
    // reconfigured. A consumer that only saw the field when it was interesting
    // could not tell "all banks on" from "counter too old to say", and those
    // are opposite readings of the same flat totals.
    t.bank_mask        = g_banks.mask;
}

uint32_t applyIdleRequest(uint32_t duration_s) {
    const bool was_sensing = idlestate::sensing(g_idle);
    const idlestate::Request r =
        idlestate::request(g_idle, millis(), duration_s);

    if (idlestate::sensing(g_idle)) {
        // Resumed (or refused to start): clear the flag and drop any pairing
        // state that predates the gap before the next poll can build on it.
        if (!was_sensing) {
            resetAllGates();
            Serial.println(F("[IDLE] sensing resumed on request"));
        }
        g_status_flags &= (uint8_t)~beecounter_proto::STATUS_NIGHT_IDLE;
    } else {
        // Entering. Park the emitters immediately rather than waiting for the
        // next poll to not run — the whole point of the feature is the current
        // they draw. Re-arming an already-idle counter is the common case and
        // must not reset the gates again: nothing has sampled in between.
        if (was_sensing) {
            resetAllGates();
            driveIrLeds(false);
        }
        g_status_flags |= beecounter_proto::STATUS_NIGHT_IDLE;
    }
    return r.granted_s;
}

uint32_t idleRemainingSeconds() {
    return idlestate::remainingSeconds(g_idle, millis());
}

uint8_t applyBankMask(uint8_t mask) {
    const bankstate::Request r = bankstate::request(g_banks, mask);
    if (!r.accepted) {
        // bank_state.h refuses an all-off mask rather than blinding the
        // counter on one byte. Say so: the alternative is a HiveHub that
        // believes it switched everything off and a counter that did not.
        Serial.printf("[BANKS] request 0x%02X refused; still 0x%02X\n",
                      (unsigned)mask, (unsigned)g_banks.mask);
        return g_banks.mask;
    }
    if (r.changed) {
        // Both edges matter. A bank going dark leaves any half-finished
        // pairing on its gates unfinishable; a bank coming back would otherwise
        // combine a pairing from before the gap with a sample from after it.
        for (uint8_t b = 1; b <= pins::NUM_LED_BANKS; b++) {
            resetGatesForBank(b);
        }
        // Park the FETs on the new mask immediately rather than waiting for the
        // next poll — drawing the current is the whole thing being switched off.
        driveIrLeds(false);
        Serial.printf("[BANKS] mask 0x%02X — %u of %u banks, %u gates active\n",
                      (unsigned)r.granted,
                      (unsigned)bankstate::enabledCount(g_banks),
                      (unsigned)pins::NUM_LED_BANKS,
                      (unsigned)(bankstate::enabledCount(g_banks) * 8u));
    }
    return r.granted;
}

uint8_t bankMask() {
    return g_banks.mask;
}

}  // namespace ble


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

    // Boot-time discovery. A chip that is missing here is NOT written off: it
    // is left unhealthy, its gates are skipped, and pollAllGates() re-probes it
    // every MCP_RETRY_INTERVAL_MS until it answers.
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        g_mcp[i].ok = initMcp(g_mcp_dev[i], g_mcp[i].addr, g_mcp[i].tag);
        g_mcp[i].fail_streak = 0;
        if (g_mcp[i].ok) {
            g_status_flags |= g_mcp[i].status_bit;
        } else {
            g_mcp[i].next_retry_ms = millis() + MCP_RETRY_INTERVAL_MS;
        }
    }

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

    Serial.printf("[SETUP] emitter banks 0x%02X (%u of %u, %u gates active)\n",
                  (unsigned)g_banks.mask,
                  (unsigned)bankstate::enabledCount(g_banks),
                  (unsigned)pins::NUM_LED_BANKS,
                  (unsigned)(bankstate::enabledCount(g_banks) * 8u));
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
    // Night mode ends on its own deadline, checked here rather than inside the
    // poll branch so it still expires while an OTA is running — otherwise a
    // relay that straddles sunrise would leave the counter suspended until the
    // next HiveHub cycle re-armed or cleared it.
    if (idlestate::serviceExpiry(g_idle, now)) {
        // Nothing has sampled since the suspension began, so any pairing state
        // still on a gate is hours stale.
        resetAllGates();
        g_status_flags &= (uint8_t)~beecounter_proto::STATUS_NIGHT_IDLE;
        Serial.println(F("[IDLE] suspension expired; counting again"));
    }

    if (ble::isOtaActive()) {
        driveIrLeds(false);
    } else if (!idlestate::sensing(g_idle)) {
        // Night mode: emitters dark, gates unpolled, totals frozen.
        //
        // The re-assert runs on the poll cadence rather than every loop pass.
        // It is here as a backstop — applyIdleRequest() already parked the
        // emitters on entry — so that nothing else (a FORCE_ON left set from a
        // bench session, a future code path) can leave a bank lit through the
        // night; doing it at full loop speed would spend the CPU cycles this
        // mode exists to save.
        if (now - g_last_poll_ms >= POLL_INTERVAL_MS) {
            g_last_poll_ms = now;
            driveIrLeds(false);
        }
        // Yield: with no gates to poll there is nothing here to be responsive
        // to except BLE, which runs on its own task. A bare spin would burn the
        // MCU's share of the budget for the whole night.
        delay(1);
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
        // Suspended: leave the emitters dark whatever the mode says. FORCE_ON
        // exists for bench work and must not be able to undo night mode from a
        // debug console that was left in that state.
        if (idlestate::sensing(g_idle)) {
            setIrLeds(true);   // setIrLeds() applies the mode-correct steady level
        } else {
            driveIrLeds(false);
        }
    }

    // Periodic debug dump on serial -- once every 30 s.
    static uint32_t last_dump_ms = 0;
    if (now - last_dump_ms > 30000) {
        last_dump_ms = now;
        Serial.printf(
            "[STAT] uptime=%lus total_in=%lu total_out=%lu "
            "glitches=%lu status=0x%02X idle=%lus banks=0x%02X\n",
            (unsigned long)(now / 1000),
            (unsigned long)g_total_in,
            (unsigned long)g_total_out,
            (unsigned long)g_glitch_count,
            (unsigned)g_status_flags,
            (unsigned long)idlestate::remainingSeconds(g_idle, now),
            (unsigned)g_banks.mask
        );
    }
}
