// ============================================================================
// idle_state.h — night-mode suspension, as pure deadline arithmetic
// ============================================================================
//
// Arduino-free for the same reason gate_logic.h and measurement_json.h are: the
// interesting part of "stop counting until told otherwise" is entirely about
// clock arithmetic that is awkward to reproduce on a bench and impossible to
// reproduce on a hive. It is exercised by test/test_idle_state/ on a host
// compiler; src/main.cpp owns the emitters and the poll loop and calls in here
// to decide whether to run them.
//
// The model
// ---------
// The counter never learns what time it is. HiveHub writes SET_IDLE with a
// DURATION, this header turns that into a millis() deadline, and every poll
// asks whether the deadline has passed. Three properties fall out of that and
// they are the whole design:
//
//   1. **It expires.** A HiveHub that crashes, loses power, or is carried away
//      cannot leave a counter suspended: the deadline runs out and sensing
//      resumes on its own. Compare a stored 20:00-06:00 schedule, which stays
//      wrong until someone walks to the hive.
//   2. **It is bounded.** Requests are clamped to MAX_IDLE_SECONDS rather than
//      refused, so a malformed or over-eager duration costs a re-arm next
//      cycle, not a night of running emitters.
//   3. **It is not persistent.** There is no NVS write and no RTC-memory copy.
//      Any reset — brownout, OTA, watchdog — comes back counting.
//
// millis() rollover
// -----------------
// Deadlines are compared with signed differences, like the rest of this
// firmware, so the ~49.7-day millis() wrap is a non-event: (int32_t)(now -
// deadline) >= 0 stays correct across it as long as the interval itself is
// shorter than half the counter's range, which MAX_IDLE_SECONDS (1 h) is by
// four orders of magnitude.
// ============================================================================

#pragma once

#include <stdint.h>

#include "counter_protocol.h"

namespace idlestate {

// Suspension state. Default-constructed is "sensing", which is what a freshly
// booted counter must always be.
struct State {
    bool     active   = false;
    uint32_t deadline_ms = 0;   // only meaningful while active
};

// Result of a SET_IDLE request, so the caller can log what it actually did
// rather than what it was asked to do.
struct Request {
    uint32_t granted_s = 0;   // duration actually applied, after clamping
    bool     clamped   = false;
};

// Arm (or re-arm) the suspension for `duration_s` seconds from `now_ms`.
//
// A zero duration resumes sensing immediately — that is the same thing
// CTRL_OP_RESUME does, and accepting it here means HiveHub can express "not
// tonight" by re-arming with 0 rather than needing a second opcode on a path
// where it already has one.
//
// Re-arming while already idle is the normal case, not an edge case: HiveHub
// pushes a fresh deadline every upload cycle for as long as the night window
// lasts, so the deadline moves forward roughly every 10 minutes and the counter
// stays suspended without any single request having to cover the whole night.
inline Request request(State& s, uint32_t now_ms, uint32_t duration_s) {
    Request r;
    r.clamped = duration_s > beecounter_proto::MAX_IDLE_SECONDS;
    r.granted_s = r.clamped ? beecounter_proto::MAX_IDLE_SECONDS : duration_s;

    if (r.granted_s == 0) {
        s.active = false;
        s.deadline_ms = 0;
        return r;
    }
    s.active = true;
    s.deadline_ms = now_ms + r.granted_s * 1000UL;
    return r;
}

// Resume sensing now, discarding any deadline.
inline void resume(State& s) {
    s.active = false;
    s.deadline_ms = 0;
}

// Has an armed suspension run out? False when not suspended at all.
inline bool expired(const State& s, uint32_t now_ms) {
    if (!s.active) return false;
    return (int32_t)(now_ms - s.deadline_ms) >= 0;
}

// Clear the suspension if its deadline has passed. Returns true exactly on the
// poll that ends it, so the caller can do its one-off resume work (resetting
// the gate state machines) without tracking the edge itself.
inline bool serviceExpiry(State& s, uint32_t now_ms) {
    if (!expired(s, now_ms)) return false;
    resume(s);
    return true;
}

// Whether sensing should run right now.
inline bool sensing(const State& s) {
    return !s.active;
}

// Seconds left on the suspension, rounded UP so a live suspension never reports
// zero (which reads as "sensing" to HiveHub and would make it re-arm a beat
// early). Zero means not suspended.
inline uint32_t remainingSeconds(const State& s, uint32_t now_ms) {
    if (!s.active) return 0;
    const int32_t left_ms = (int32_t)(s.deadline_ms - now_ms);
    if (left_ms <= 0) return 0;
    return ((uint32_t)left_ms + 999UL) / 1000UL;
}

}  // namespace idlestate
