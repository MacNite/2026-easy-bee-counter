// ============================================================================
// Host-side tests for include/gate_logic.h
// ============================================================================
//
// These exist because the debounce cannot be validated on hardware without a
// bee, an oscilloscope and a great deal of patience — and because the bug they
// pin down (a "debounce" that accepted the first differing sample) survived
// review precisely by looking correct in a code read.
//
// Run them with PlatformIO:
//
//     pio test -e native
//
// or directly with any host compiler:
//
//     c++ -std=c++11 -I include test/test_gate_logic/test_gate_logic.cpp -o /tmp/t && /tmp/t
//
// The environment is `test_framework = custom`, so this file owns main() and
// signals failure with a non-zero exit status.
// ============================================================================

#include "gate_logic.h"

#include <cstdio>
#include <cstdlib>

using namespace gatelogic;

// --------------------------------------------------------------------------
// Tiny assertion harness (no dependency on <cassert>'s NDEBUG behaviour).
// --------------------------------------------------------------------------
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAIL %s:%d  [%s]  %s\n", __FILE__, __LINE__,      \
                        g_case, #cond);                                      \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

// The firmware's production tuning: 5 ms polling, so a change must hold for
// 2 consecutive samples and at least 5 ms.
static const Tuning kCfg = { /*debounce_ms=*/5, /*debounce_samples=*/2,
                             /*pairing_window_ms=*/2000, /*stuck_ms=*/30000 };

static const uint32_t kPoll = 5;   // POLL_INTERVAL_MS

// --------------------------------------------------------------------------
// A gate driven at the real poll rate, tallying events the way main.cpp does.
// --------------------------------------------------------------------------
struct Sim {
    GateRuntime rt;
    uint32_t now;
    uint32_t in = 0, out = 0, glitch = 0, stuck = 0;

    explicit Sim(uint32_t start_ms = 1000) : now(start_ms) {}

    // Advance one poll with the given raw levels (true == beam blocked).
    void poll(bool inner, bool outer) {
        const GateUpdate u = updateGate(rt, inner, outer, now, kCfg);
        switch (u.event) {
        case GateEvent::BEE_IN:  ++in; break;
        case GateEvent::BEE_OUT: ++out; break;
        case GateEvent::GLITCH:  ++glitch; break;
        case GateEvent::NONE:    break;
        }
        if (u.sensor_stuck) ++stuck;
        now += kPoll;
    }

    void pollN(int n, bool inner, bool outer) {
        for (int i = 0; i < n; i++) poll(inner, outer);
    }

    bool quiet() const { return in == 0 && out == 0 && glitch == 0; }
};

// --------------------------------------------------------------------------
// Debounce
// --------------------------------------------------------------------------

// A steady clear line must never produce an event, and must never drift into a
// state where the next single sample is trusted.
static void test_steady_clear_is_silent() {
    g_case = "steady clear";
    Sim s;
    s.pollN(200, false, false);
    CHECK(s.quiet());
    CHECK(s.rt.state == GateState::IDLE);
    CHECK(!s.rt.inner.accepted);
    CHECK(!s.rt.outer.accepted);
}

// THE regression test. One corrupted MCP23017 word — a single sample in which
// a sensor reads blocked — must not move the state machine. Before the fix the
// elapsed-time check measured the poll period rather than the candidate's age,
// so this promoted immediately and pushed the gate into INNER_FIRST.
static void test_single_sample_spike_rejected() {
    g_case = "single-sample spike";
    Sim s;
    s.pollN(10, false, false);
    s.poll(true, false);          // one bad word
    s.pollN(10, false, false);
    CHECK(s.quiet());
    CHECK(s.rt.state == GateState::IDLE);
    CHECK(!s.rt.inner.accepted);
}

// A whole corrupted snapshot (both sensors of the gate reading blocked for one
// sample) is the shape a failed I2C read used to take. It must not register as
// the ambiguous-simultaneous-block glitch either.
static void test_single_sample_both_blocked_rejected() {
    g_case = "single-sample both blocked";
    Sim s;
    s.pollN(10, false, false);
    s.poll(true, true);
    s.pollN(10, false, false);
    CHECK(s.quiet());
    CHECK(s.rt.state == GateState::IDLE);
}

// Alternating noise never holds a value for two consecutive polls, so nothing
// is ever promoted no matter how long it goes on.
static void test_alternating_noise_rejected() {
    g_case = "alternating noise";
    Sim s;
    for (int i = 0; i < 500; i++) s.poll(i % 2 == 0, false);
    CHECK(s.quiet());
    CHECK(!s.rt.inner.accepted);
    CHECK(s.rt.state == GateState::IDLE);
}

// Two consecutive agreeing samples promote — and not one sooner.
static void test_two_samples_promote() {
    g_case = "two samples promote";
    Sim s;
    s.pollN(5, false, false);
    s.poll(true, false);
    CHECK(!s.rt.inner.accepted);   // first sight: candidate only
    s.poll(true, false);
    CHECK(s.rt.inner.accepted);    // second sight: accepted
    CHECK(s.rt.state == GateState::INNER_FIRST);
}

// A partial run that is interrupted must restart from scratch, not accumulate.
static void test_interrupted_run_restarts() {
    g_case = "interrupted run restarts";
    Sim s;
    s.pollN(5, false, false);
    s.poll(true, false);           // candidate blocked, 1 sample
    s.poll(false, false);          // back to clear: candidate discarded
    s.poll(true, false);           // candidate blocked again, 1 sample
    CHECK(!s.rt.inner.accepted);
    CHECK(s.rt.state == GateState::IDLE);
    CHECK(s.quiet());
}

// --------------------------------------------------------------------------
// Crossings
// --------------------------------------------------------------------------

// A bee dwells ~12 ms in the beam, i.e. 2-3 samples at a 5 ms poll. The
// debounce must still be loose enough to see that. This is the constraint that
// stops SENSOR_DEBOUNCE_SAMPLES from simply being raised to 3.
static void test_realistic_incoming_crossing() {
    g_case = "incoming crossing";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(3, false, true);       // ~15 ms: outer beam broken
    s.pollN(3, true, true);        // bee spans both
    s.pollN(3, true, false);       // outer clears, inner still blocked
    s.pollN(10, false, false);     // both clear
    CHECK(s.in == 1);
    CHECK(s.out == 0);
    CHECK(s.rt.state == GateState::IDLE);
}

static void test_realistic_outgoing_crossing() {
    g_case = "outgoing crossing";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(3, true, false);
    s.pollN(3, true, true);
    s.pollN(3, false, true);
    s.pollN(10, false, false);
    CHECK(s.out == 1);
    CHECK(s.in == 0);
    CHECK(s.rt.state == GateState::IDLE);
}

// The minimum credible dwell: exactly two samples per phase.
static void test_minimum_dwell_crossing() {
    g_case = "minimum dwell crossing";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(2, false, true);
    s.pollN(2, true, true);
    s.pollN(2, true, false);
    s.pollN(5, false, false);
    CHECK(s.in == 1);
}

// Ten crossings in a row must count ten times and leave the gate idle.
static void test_repeated_crossings() {
    g_case = "repeated crossings";
    Sim s;
    for (int i = 0; i < 10; i++) {
        s.pollN(3, false, true);
        s.pollN(3, true, true);
        s.pollN(3, true, false);
        s.pollN(5, false, false);
    }
    CHECK(s.in == 10);
    CHECK(s.out == 0);
    CHECK(s.rt.state == GateState::IDLE);
}

// Both sensors genuinely blocking together (held long enough to be real) is
// direction-ambiguous and is tallied as a glitch, exactly once.
static void test_simultaneous_block_is_one_glitch() {
    g_case = "simultaneous block";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(20, true, true);
    CHECK(s.glitch == 1);
    CHECK(s.in == 0 && s.out == 0);
    CHECK(s.rt.state == GateState::PAIRED);
    s.pollN(5, false, false);
    CHECK(s.rt.state == GateState::IDLE);
}

// A bee that pokes the inner beam and retreats is a false start, not a count.
static void test_false_start_is_glitch_not_count() {
    g_case = "false start";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(4, true, false);
    s.pollN(10, false, false);
    CHECK(s.in == 0 && s.out == 0);
    CHECK(s.glitch == 1);
    CHECK(s.rt.state == GateState::IDLE);
}

// A bee that sits in the tunnel past the pairing window must not later be
// paired with an unrelated block on the other sensor.
static void test_pairing_window_expires() {
    g_case = "pairing window expiry";
    Sim s;
    s.pollN(5, false, false);
    s.pollN(2, false, true);                    // outer blocks
    CHECK(s.rt.state == GateState::OUTER_FIRST);
    s.now += kCfg.pairing_window_ms + 100;      // long pause, beam still broken
    s.pollN(4, false, true);
    s.pollN(4, false, false);                   // clears -> back to IDLE
    CHECK(s.rt.state == GateState::IDLE);
    s.pollN(4, true, false);                    // inner blocks much later
    CHECK(s.in == 0);                           // must not pair across the gap
}

// --------------------------------------------------------------------------
// Stuck sensors and rollover
// --------------------------------------------------------------------------

static void test_stuck_sensor_reported() {
    g_case = "stuck sensor";
    Sim s;
    s.pollN(4, true, false);
    CHECK(s.stuck == 0);
    s.now += kCfg.stuck_ms + 1;
    s.poll(true, false);
    CHECK(s.stuck == 1);
}

// millis() wraps every ~49.7 days. Every comparison in gate_logic.h is an
// unsigned difference, so a crossing straddling the wrap must behave normally.
static void test_millis_rollover() {
    g_case = "millis rollover";
    Sim s(0xFFFFFFFFu - 20);       // wraps partway through the crossing
    s.pollN(2, false, false);
    s.pollN(3, false, true);
    s.pollN(3, true, true);
    s.pollN(3, true, false);
    s.pollN(5, false, false);
    CHECK(s.in == 1);
    CHECK(s.glitch == 0);
    CHECK(s.rt.state == GateState::IDLE);
}

// A gate blocked across the wrap must not report a spurious stuck sensor.
static void test_no_false_stuck_across_rollover() {
    g_case = "no false stuck across rollover";
    Sim s(0xFFFFFFFFu - 100);
    s.pollN(30, true, false);      // blocked across the wrap, well under 30 s
    CHECK(s.stuck == 0);
}

// --------------------------------------------------------------------------
// Saturating counters
// --------------------------------------------------------------------------

static void test_totals_saturate() {
    g_case = "32-bit totals saturate";
    uint32_t total = UINT32_MAX - 2;
    CHECK(!countSaturating(total));
    CHECK(total == UINT32_MAX - 1);
    CHECK(countSaturating(total));      // reaching the max reports saturation
    CHECK(total == UINT32_MAX);
    CHECK(countSaturating(total));      // and never wraps past it
    CHECK(total == UINT32_MAX);
    for (int i = 0; i < 100; i++) countSaturating(total);
    CHECK(total == UINT32_MAX);
}

static void test_glitches_saturate() {
    g_case = "16-bit glitches saturate";
    uint16_t glitches = UINT16_MAX - 1;
    CHECK(countSaturating(glitches));
    CHECK(glitches == UINT16_MAX);
    CHECK(countSaturating(glitches));
    CHECK(glitches == UINT16_MAX);      // must not wrap to 0
}

// --------------------------------------------------------------------------

int main() {
    std::printf("gate_logic tests\n");
    test_steady_clear_is_silent();
    test_single_sample_spike_rejected();
    test_single_sample_both_blocked_rejected();
    test_alternating_noise_rejected();
    test_two_samples_promote();
    test_interrupted_run_restarts();
    test_realistic_incoming_crossing();
    test_realistic_outgoing_crossing();
    test_minimum_dwell_crossing();
    test_repeated_crossings();
    test_simultaneous_block_is_one_glitch();
    test_false_start_is_glitch_not_count();
    test_pairing_window_expires();
    test_stuck_sensor_reported();
    test_millis_rollover();
    test_no_false_stuck_across_rollover();
    test_totals_saturate();
    test_glitches_saturate();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
