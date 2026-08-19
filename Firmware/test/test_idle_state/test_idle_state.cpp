// ============================================================================
// Host-side tests for include/idle_state.h
// ============================================================================
//
// Night mode is a feature whose failure mode is silent: a counter that stays
// suspended when it should be counting reports perfectly well-formed documents
// full of zeros, and nothing about a hive in August distinguishes that from a
// stretch of bad weather until someone reads the totals a week later. The
// deadline arithmetic is therefore pinned here rather than trusted to a bench
// session, including the two cases that are impractical to reproduce on real
// hardware: the millis() rollover, and an idle window that outlives HiveHub.
//
//     c++ -std=c++11 -I include
//         test/test_idle_state/test_idle_state.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

// ---------------------------------------------------------------------------
// Arduino's macro soup, reproduced before the include on purpose.
// ---------------------------------------------------------------------------
// idle_state.h and counter_protocol.h are built two ways: here, against a bare
// host compiler, and in the firmware, after Arduino.h has defined a few hundred
// all-caps macros. The second is far more hostile, and a test that only sees
// the first cannot catch a name collision — HiveHub hit exactly that, where an
// enumerator named DISABLED met esp32-hal-gpio.h's `#define DISABLED 0x00` and
// broke the build while every host check passed.
//
// So define the ones that actually bite, with their real Arduino values, before
// the include. Any constant added to the shared protocol header that strays
// into the all-caps macro namespace now fails on a host compiler in seconds
// rather than partway through a firmware build. Deliberately NOT #undef'd.
#define DISABLED 0x00
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define PULLDOWN 0x08
#define HIGH 0x1
#define LOW 0x0
#define ANALOG 0xC0
#define OPEN_DRAIN 0x10

#include "idle_state.h"

#include <cstdio>
#include <cstdlib>

using namespace idlestate;
using beecounter_proto::MAX_IDLE_SECONDS;

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

// --------------------------------------------------------------------------

static void test_default_is_sensing() {
    // A counter that boots suspended is a counter that never counts again if
    // the request that suspended it came from a HiveHub now out of range.
    g_case = "a fresh state senses";
    State s;
    CHECK(sensing(s));
    CHECK(remainingSeconds(s, 0) == 0);
    CHECK(!expired(s, 0));
    CHECK(!expired(s, 0xFFFFFFFFUL));
}

static void test_request_suspends_until_the_deadline() {
    g_case = "a request suspends for exactly its duration";
    State s;
    const Request r = request(s, 1000, 600);
    CHECK(r.granted_s == 600);
    CHECK(!r.clamped);
    CHECK(!sensing(s));
    CHECK(remainingSeconds(s, 1000) == 600);

    CHECK(!expired(s, 1000 + 599999UL));      // one ms short
    CHECK(remainingSeconds(s, 1000 + 599999UL) == 1);
    CHECK(expired(s, 1000 + 600000UL));       // exactly on it
    CHECK(expired(s, 1000 + 600001UL));
}

static void test_zero_duration_resumes() {
    // HiveHub expresses "not tonight after all" by re-arming with 0 on the
    // connection it already has, rather than needing a second round trip.
    g_case = "a zero-second request resumes sensing";
    State s;
    request(s, 1000, 600);
    CHECK(!sensing(s));
    const Request r = request(s, 2000, 0);
    CHECK(r.granted_s == 0);
    CHECK(!r.clamped);
    CHECK(sensing(s));
    CHECK(remainingSeconds(s, 2000) == 0);
}

static void test_explicit_resume() {
    g_case = "resume() clears an armed suspension";
    State s;
    request(s, 1000, 600);
    resume(s);
    CHECK(sensing(s));
    CHECK(remainingSeconds(s, 1000) == 0);
    CHECK(!expired(s, 0xFFFFFFFFUL));
}

static void test_oversized_request_is_clamped_not_refused() {
    // Refusing would leave the emitters running all night because one field was
    // too large — the opposite of what the request was for. Clamping costs one
    // re-arm at the cap.
    g_case = "an over-long request is clamped to the cap";
    State s;
    const Request r = request(s, 0, 86400);       // a whole day
    CHECK(r.clamped);
    CHECK(r.granted_s == MAX_IDLE_SECONDS);
    CHECK(!sensing(s));
    CHECK(remainingSeconds(s, 0) == MAX_IDLE_SECONDS);
    CHECK(expired(s, MAX_IDLE_SECONDS * 1000UL));
}

static void test_cap_is_shorter_than_a_night() {
    // The cap is load-bearing: it is what bounds how long a counter stays blind
    // after HiveHub falls off the air. If it ever grew past a night, a single
    // request could strand a counter until morning with nothing to re-arm it.
    g_case = "the cap cannot cover a whole night";
    CHECK(MAX_IDLE_SECONDS <= 3600);
    CHECK(MAX_IDLE_SECONDS > 0);
}

static void test_rearming_extends_the_window() {
    // The normal night: HiveHub pushes a fresh deadline every upload cycle, so
    // no single request ever has to cover more than one cycle.
    g_case = "re-arming each cycle keeps a long night suspended";
    State s;
    uint32_t now = 5000;
    request(s, now, 900);                 // 15 min, HiveHub's 10 min + margin
    for (int cycle = 0; cycle < 40; ++cycle) {   // ~6.7 h of 10-min cycles
        now += 600000UL;                  // one upload cycle later
        CHECK(!expired(s, now));          // the old deadline has not run out
        request(s, now, 900);             // re-armed
        CHECK(!sensing(s));
    }
    // HiveHub stops re-arming (sunrise, or it died). The counter frees itself.
    now += 900000UL;
    CHECK(expired(s, now));
    CHECK(serviceExpiry(s, now));
    CHECK(sensing(s));
}

static void test_service_expiry_fires_once() {
    // The caller uses the return value as a one-shot edge to reset the gate
    // state machines. A second true would reset gates mid-count.
    g_case = "serviceExpiry is an edge, not a level";
    State s;
    request(s, 0, 10);
    CHECK(!serviceExpiry(s, 5000));       // still inside the window
    CHECK(!sensing(s));
    CHECK(serviceExpiry(s, 10000));       // the one poll that ends it
    CHECK(sensing(s));
    CHECK(!serviceExpiry(s, 10001));      // and never again
    CHECK(!serviceExpiry(s, 999999UL));
}

static void test_remaining_rounds_up() {
    // A live suspension reporting 0 would read as "sensing" to HiveHub and to
    // anyone reading idle_s out of stored history.
    g_case = "remaining never rounds a live suspension down to zero";
    State s;
    request(s, 0, 10);
    CHECK(remainingSeconds(s, 9999) == 1);    // 1 ms left
    CHECK(remainingSeconds(s, 9001) == 1);    // 999 ms left
    CHECK(remainingSeconds(s, 9000) == 1);
    CHECK(remainingSeconds(s, 8999) == 2);
    CHECK(remainingSeconds(s, 10000) == 0);   // genuinely over
    CHECK(remainingSeconds(s, 20000) == 0);   // and past it
}

static void test_millis_rollover() {
    // millis() wraps every ~49.7 days. An unsigned comparison would make a
    // suspension armed just before the wrap look already-expired (resuming the
    // emitters in the middle of the night) or never-expiring, depending on
    // which side of the subtraction wrapped. Signed differences hold.
    g_case = "a suspension armed across the millis() wrap behaves";
    const uint32_t near_wrap = 0xFFFFFF00UL;   // 256 ms before the wrap
    State s;
    request(s, near_wrap, 600);                // deadline is past the wrap
    CHECK(!sensing(s));
    CHECK(!expired(s, near_wrap));
    CHECK(!expired(s, 0));                     // the wrap itself
    CHECK(!expired(s, 100000));                // 100 s in, still suspended
    CHECK(remainingSeconds(s, 0) == 600);      // 256 ms rounds up
    // Written as runtime arithmetic on a uint32_t: the wrap is the point of the
    // test, and spelling it as a constant expression is a compile-time overflow.
    uint32_t past_deadline = near_wrap;
    past_deadline += 600000UL;
    CHECK(expired(s, past_deadline));          // wrapped, and past the deadline
}

int main() {
    std::printf("idle_state tests\n");
    test_default_is_sensing();
    test_request_suspends_until_the_deadline();
    test_zero_duration_resumes();
    test_explicit_resume();
    test_oversized_request_is_clamped_not_refused();
    test_cap_is_shorter_than_a_night();
    test_rearming_extends_the_window();
    test_service_expiry_fires_once();
    test_remaining_rounds_up();
    test_millis_rollover();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
