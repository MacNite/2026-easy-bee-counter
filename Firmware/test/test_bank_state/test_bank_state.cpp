// ============================================================================
// Host-side tests for include/bank_state.h
// ============================================================================
//
// The emitter-bank mask decides whether eight gates are counted at all, and
// gets it wrong silently in both directions: a bank that should be on but is
// off produces a permanently flat third of the totals, which reads as a dead
// FET, and a bank that should be off but is on just quietly costs ~80 mA on a
// supply that was sized without it. Neither shows up in a code read and neither
// is fun to reproduce on a hive, so the rules are pinned here.
//
//     c++ -std=c++11 -I include
//         test/test_bank_state/test_bank_state.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

#include "bank_state.h"

#include <cstdio>
#include <cstdlib>

using namespace beecounter_proto;

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

static void test_default_is_everything_on() {
    // A counter that boots, resets after a brownout, or reboots out of an OTA
    // must come back counting all 24 gates. Nothing about this feature is
    // persisted, so this default IS the post-reset behaviour.
    g_case = "a fresh counter runs every bank";
    bankstate::State s;
    CHECK(s.mask == BANK_MASK_ALL);
    CHECK(bankstate::enabledCount(s) == 3);
    CHECK(bankstate::enabled(s, 1));
    CHECK(bankstate::enabled(s, 2));
    CHECK(bankstate::enabled(s, 3));
}

static void test_bank_numbering_is_one_based() {
    // gates::GateLocation::led_bank is 1..3; a 0-based read of it would map
    // every gate one bank to the left and light the wrong eight.
    g_case = "bank numbers are 1..3, and 0 is not a bank";
    CHECK(bankstate::bankBit(1) == 0x01);
    CHECK(bankstate::bankBit(2) == 0x02);
    CHECK(bankstate::bankBit(3) == 0x04);
    CHECK(bankstate::bankBit(0) == 0x00);

    bankstate::State s;
    CHECK(!bankstate::enabled(s, 0));
    CHECK(!bankstate::enabled(s, 4));   // no such FET on this board
}

static void test_single_bank_leaves_the_others_dark() {
    g_case = "one bank on means exactly one bank on";
    bankstate::State s;
    const bankstate::Request r = bankstate::request(s, 0x02);
    CHECK(r.accepted);
    CHECK(r.changed);
    CHECK(r.granted == 0x02);
    CHECK(!bankstate::enabled(s, 1));
    CHECK(bankstate::enabled(s, 2));
    CHECK(!bankstate::enabled(s, 3));
    CHECK(bankstate::enabledCount(s) == 1);
}

static void test_all_off_is_refused() {
    // The rule that keeps one corrupted byte from blinding a counter until
    // someone walks to the hive. A counter that should count nothing is
    // unpaired in HiveHub; it is never masked to zero here.
    g_case = "a mask of zero is refused, not applied";
    bankstate::State s;
    bankstate::request(s, 0x05);          // banks 1 and 3
    const bankstate::Request r = bankstate::request(s, 0x00);
    CHECK(!r.accepted);
    CHECK(!r.changed);
    CHECK(r.granted == 0x05);
    CHECK(s.mask == 0x05);                // unchanged: still counting
    CHECK(bankstate::enabledCount(s) == 2);
}

static void test_bits_above_the_last_bank_are_ignored() {
    // A four-FET board's mask must not conjure a bank 4 whose GPIO does not
    // exist on this one — and 0xF8 alone must not read as "some banks on".
    g_case = "phantom banks are masked off";
    bankstate::State s;
    bankstate::Request r = bankstate::request(s, 0xFF);
    CHECK(r.accepted);
    CHECK(r.granted == BANK_MASK_ALL);
    CHECK(s.mask == BANK_MASK_ALL);
    CHECK(bankstate::enabledCount(s) == 3);

    // Only phantom bits set is the same as asking for nothing, and is refused
    // the same way rather than applied as an empty mask.
    r = bankstate::request(s, 0xF8);
    CHECK(!r.accepted);
    CHECK(s.mask == BANK_MASK_ALL);
}

static void test_reasserting_the_same_mask_is_not_a_change() {
    // HiveHub re-writes the mask every upload cycle. `changed` is what the
    // firmware uses to decide whether to tear down the gate state machines, so
    // a steady-state re-assert must not reset a pairing in progress every ten
    // minutes.
    g_case = "an unchanged re-assert is accepted but not a change";
    bankstate::State s;
    bankstate::request(s, 0x03);
    const bankstate::Request r = bankstate::request(s, 0x03);
    CHECK(r.accepted);
    CHECK(!r.changed);
    CHECK(s.mask == 0x03);
}

static void test_a_bank_can_come_back() {
    g_case = "switching a bank back on is just another request";
    bankstate::State s;
    bankstate::request(s, 0x01);
    CHECK(bankstate::enabledCount(s) == 1);
    const bankstate::Request r = bankstate::request(s, BANK_MASK_ALL);
    CHECK(r.accepted);
    CHECK(r.changed);
    CHECK(bankstate::enabledCount(s) == 3);
}

static void test_mask_matches_the_protocol_constant() {
    // BANK_MASK_ALL and the three physical FETs have to agree; if a board
    // revision adds one, this is the line that fails first.
    g_case = "BANK_MASK_ALL covers exactly three banks";
    CHECK(BANK_MASK_ALL == 0x07);
    CHECK(CTRL_OP_SET_BANKS == 0x03);
    CHECK(CTRL_SET_BANKS_LENGTH == 2);
    // The opcode space is shared with night mode; a collision would let a
    // suspension request switch banks or vice versa.
    CHECK(CTRL_OP_SET_BANKS != CTRL_OP_SET_IDLE);
    CHECK(CTRL_OP_SET_BANKS != CTRL_OP_RESUME);
}

int main() {
    std::printf("bank_state tests\n");
    test_default_is_everything_on();
    test_bank_numbering_is_one_based();
    test_single_bank_leaves_the_others_dark();
    test_all_off_is_refused();
    test_bits_above_the_last_bank_are_ignored();
    test_reasserting_the_same_mask_is_not_a_change();
    test_a_bank_can_come_back();
    test_mask_matches_the_protocol_constant();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
