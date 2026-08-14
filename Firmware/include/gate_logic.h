// ============================================================================
// gate_logic.h — sensor debouncing + per-gate crossing state machine
// ============================================================================
//
// Pure logic, deliberately free of Arduino/I2C/hardware dependencies: it takes
// raw sensor levels plus a timestamp and returns what happened. main.cpp owns
// the hardware (pulsing the emitters, reading the MCP23017s, deciding which
// gates have a trustworthy sample this cycle) and the global counters.
//
// Splitting it out this way is what makes the debounce testable on a host
// compiler — see test/test_gate_logic/. The timing rules below cannot be
// validated on hardware without a bee, an oscilloscope and a lot of patience.
//
// ---------------------------------------------------------------------------
// Why the debounce is written the way it is
// ---------------------------------------------------------------------------
// The earlier implementation kept only the accepted state plus a "last change"
// timestamp, and refreshed that timestamp on every poll where the raw reading
// still agreed with the accepted state. That made the elapsed-time test at the
// moment of a change measure the POLL PERIOD rather than how long the new value
// had persisted, so with a 5 ms poll and a 3 ms threshold the very first
// differing sample was always accepted: a one-sample transition detector, not a
// debounce. One corrupted MCP23017 word could move the state machine.
//
// The fix is to track the CANDIDATE value separately from the accepted one:
//
//   * a raw reading that differs from the current candidate restarts the
//     candidate, its first-seen timestamp and its sample count;
//   * a raw reading that agrees extends the candidate's run;
//   * the candidate is promoted to accepted only once it has been seen on
//     `debounce_samples` consecutive polls AND has persisted for at least
//     `debounce_ms`.
//
// Both conditions matter. The sample count is what rejects a single bad word
// (the failure the old code could not see); the elapsed-time rule is what keeps
// the guarantee meaningful if the poll interval is ever shortened.
//
// All time comparisons use unsigned subtraction, so millis() rollover at ~49.7
// days is handled without special cases.
// ============================================================================

#pragma once

#include <stdint.h>

namespace gatelogic {

// ---------------------------------------------------------------------------
// Timing rules, supplied by the caller so tests can drive the logic at
// timescales that would be impractical on hardware.
// ---------------------------------------------------------------------------
struct Tuning {
    uint32_t debounce_ms;        // minimum persistence before a change counts
    uint8_t  debounce_samples;   // minimum consecutive polls agreeing
    uint32_t pairing_window_ms;  // max inner<->outer gap for a directional count
    uint32_t stuck_ms;           // continuously blocked longer than this = fault
};

enum class GateState : uint8_t {
    IDLE,           // both sensors clear, no event in progress
    OUTER_FIRST,    // outer triggered first, awaiting inner
    INNER_FIRST,    // inner triggered first, awaiting outer
    PAIRED,         // both triggered, waiting for both to clear
};

// What one update() call concluded. At most one event per gate per poll.
enum class GateEvent : uint8_t {
    NONE,
    BEE_IN,     // outer-then-inner: a bee entered the hive
    BEE_OUT,    // inner-then-outer: a bee left the hive
    GLITCH,     // ambiguous or aborted pairing, counted for diagnostics only
};

struct GateUpdate {
    GateEvent event = GateEvent::NONE;
    bool sensor_stuck = false;   // a sensor has been blocked past stuck_ms
};

// ---------------------------------------------------------------------------
// One debounced sensor line.
// ---------------------------------------------------------------------------
// `accepted` is the debounced state the state machine sees; everything else
// describes the candidate currently being timed. true == beam blocked.
struct SensorDebounce {
    bool     accepted = false;
    bool     candidate = false;
    uint8_t  candidate_samples = 0;
    uint32_t candidate_since_ms = 0;
    uint32_t blocked_since_ms = 0;   // when `accepted` last became true
};

// Feed one raw sample. Returns true if the debounced state changed this call.
inline bool debounceSensor(SensorDebounce& s, bool raw, uint32_t now_ms,
                           const Tuning& cfg) {
    if (raw != s.candidate) {
        // A new candidate: restart its run. Note this deliberately also fires
        // when the raw value returns to the accepted state after a partial run,
        // which is what discards noise that never made it to promotion.
        s.candidate = raw;
        s.candidate_since_ms = now_ms;
        s.candidate_samples = 1;
        return false;
    }

    if (s.candidate_samples < 0xFF) s.candidate_samples++;

    if (s.candidate == s.accepted) return false;              // nothing to do
    if (s.candidate_samples < cfg.debounce_samples) return false;
    if (now_ms - s.candidate_since_ms < cfg.debounce_ms) return false;

    s.accepted = s.candidate;
    if (s.accepted) s.blocked_since_ms = now_ms;
    return true;
}

// ---------------------------------------------------------------------------
// Per-gate runtime state.
// ---------------------------------------------------------------------------
struct GateRuntime {
    GateState state = GateState::IDLE;
    SensorDebounce inner;
    SensorDebounce outer;
    uint32_t event_start_ms = 0;   // when the current pairing began
};

// Process one gate after its two sensors have been freshly read.
// raw_inner / raw_outer: true means "beam blocked" (sensor line reads LOW).
//
// The caller must NOT call this for a gate whose sensor word is untrustworthy
// (an MCP23017 that failed to answer this cycle). A missing chip reads as all
// zeros, which is indistinguishable from "every beam blocked".
inline GateUpdate updateGate(GateRuntime& rt, bool raw_inner, bool raw_outer,
                             uint32_t now_ms, const Tuning& cfg) {
    GateUpdate out;

    debounceSensor(rt.inner, raw_inner, now_ms, cfg);
    debounceSensor(rt.outer, raw_outer, now_ms, cfg);

    // ----- stuck-sensor health check -----
    // A bee cannot physically block a beam for stuck_ms; something is wrong
    // with the emitter, the phototransistor or the tunnel.
    if (rt.inner.accepted && now_ms - rt.inner.blocked_since_ms > cfg.stuck_ms) {
        out.sensor_stuck = true;
    }
    if (rt.outer.accepted && now_ms - rt.outer.blocked_since_ms > cfg.stuck_ms) {
        out.sensor_stuck = true;
    }

    const bool inner = rt.inner.accepted;
    const bool outer = rt.outer.accepted;

    // ----- state machine -----
    switch (rt.state) {
    case GateState::IDLE:
        if (inner && !outer) {
            rt.state = GateState::INNER_FIRST;
            rt.event_start_ms = now_ms;
        } else if (outer && !inner) {
            rt.state = GateState::OUTER_FIRST;
            rt.event_start_ms = now_ms;
        } else if (inner && outer) {
            // both blocked at the same poll -- ambiguous direction, treat
            // as a glitch and stay paired until both clear.
            rt.state = GateState::PAIRED;
            out.event = GateEvent::GLITCH;
        }
        break;

    case GateState::INNER_FIRST:
        // expecting outer to block next -> OUTGOING bee
        if (now_ms - rt.event_start_ms > cfg.pairing_window_ms) {
            // timeout: bee stayed inside or backed off. Reset on clear.
            if (!inner && !outer) rt.state = GateState::IDLE;
        } else if (outer) {
            rt.state = GateState::PAIRED;
            out.event = GateEvent::BEE_OUT;
        } else if (!inner) {
            // inner cleared before outer ever blocked: false start
            rt.state = GateState::IDLE;
            out.event = GateEvent::GLITCH;
        }
        break;

    case GateState::OUTER_FIRST:
        // expecting inner to block next -> INCOMING bee
        if (now_ms - rt.event_start_ms > cfg.pairing_window_ms) {
            if (!inner && !outer) rt.state = GateState::IDLE;
        } else if (inner) {
            rt.state = GateState::PAIRED;
            out.event = GateEvent::BEE_IN;
        } else if (!outer) {
            rt.state = GateState::IDLE;
            out.event = GateEvent::GLITCH;
        }
        break;

    case GateState::PAIRED:
        // wait until both sensors clear, then return to IDLE
        if (!inner && !outer) rt.state = GateState::IDLE;
        break;
    }

    return out;
}

// ---------------------------------------------------------------------------
// Saturating counters
// ---------------------------------------------------------------------------
// The BLE contract (docs/ble-mode.md) promises totals that are monotonic until
// 32-bit saturation. An unconditional increment breaks that promise at the very
// end of the range: the counter wraps to zero, which a client that only sees
// the totals cannot tell apart from a reboot. Saturating instead keeps the
// sequence monotonic forever, and the caller raises STATUS_OVERFLOW_FLAG so the
// pinned value is not mistaken for a device that stopped counting.
//
// The reference is volatile-qualified because the firmware's counters are read
// from the NimBLE host task; binding a plain counter to it (as the tests do) is
// a legal qualification conversion.
//
// Returns true once the counter is pinned at its maximum.
inline bool countSaturating(volatile uint32_t& total) {
    if (total == UINT32_MAX) return true;
    total = total + 1;
    return total == UINT32_MAX;
}

// The diagnostic glitch tally uses the same 32-bit counter as of protocol v3.
// It was a uint16_t that first wrapped every 65,536 events — making a badly
// unhealthy device look like it had FEWER glitches than on the previous read —
// and then, once saturating, pinned at 65535, which a noisy device can reach in
// a day. Widening it needed HiveHub's parser to change in step, so it happened
// as part of the v3 revision rather than on the counter alone.

}  // namespace gatelogic
