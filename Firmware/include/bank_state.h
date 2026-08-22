// ============================================================================
// bank_state.h — which emitter banks (MOSFETs) are enabled
// ============================================================================
//
// Arduino-free, like idle_state.h and gate_logic.h, and for the same reason:
// the interesting part of "run only some of the counter" is a handful of
// bitmask rules that decide whether eight gates get counted at all. Getting one
// wrong is silent — the totals for those gates simply stay flat, which looks
// exactly like a dead FET — so the rules are pinned by
// test/test_bank_state/ on a host compiler. src/main.cpp owns the GPIO and the
// poll loop and calls in here for the verdict.
//
// The model
// ---------
// Since the 2026-08 hardware revision there is one IRLB8721 per MCP23017, so a
// bank IS a chip is eight gates:
//
//     bank 1 (bit 0) -> U2 @ 0x20, gates 00..07
//     bank 2 (bit 1) -> U3 @ 0x21, gates 10..17
//     bank 3 (bit 2) -> U4 @ 0x22, gates 20..27
//
// HiveHub writes a mask; this header decides what is actually applied. Three
// rules, each of which exists because the alternative fails quietly:
//
//   1. **Bits above the last bank are ignored.** A four-bank board's mask
//      arriving at a three-bank counter must not conjure a bank 4 whose GPIO
//      does not exist.
//   2. **A mask of 0 is refused, not applied.** Every other decision in this
//      firmware is arranged so that a bad write costs a cycle rather than a
//      deployment; accepting 0 would let one corrupted byte blind a counter
//      until someone walks to the hive. A counter that should count nothing is
//      unpaired in HiveHub, not masked to zero here.
//   3. **It is not persisted.** A reset comes back with every bank enabled and
//      counting, and HiveHub re-asserts the mask on its next upload cycle. The
//      worst case is one cycle of drawing more current than asked for, which is
//      the same failure direction night mode chose.
//
// Why not just leave the FET off and read the chip anyway
// ------------------------------------------------------
// That is exactly what this does — the chip is still read and still health-
// checked, so `mcps_healthy` keeps meaning what it has always meant. What the
// caller must additionally do is SKIP the gates on a dark bank, which is not
// paranoia: an unlit QRE1113 is not a sensor that reads "clear", it is a bare
// phototransistor under a 100k pull-up, and direct sun through a hive entrance
// is quite capable of pulling one low. Counting those would invent crossings on
// gates the operator deliberately switched off.
// ============================================================================

#pragma once

#include <stdint.h>

#include "counter_protocol.h"

namespace bankstate {

// Enabled-bank bitmask. Default-constructed is "everything on", which is what a
// freshly booted counter must always be.
struct State {
    uint8_t mask = beecounter_proto::BANK_MASK_ALL;
};

// Result of a SET_BANKS request, so the caller can log what it actually did
// rather than what it was asked to do.
struct Request {
    uint8_t granted  = beecounter_proto::BANK_MASK_ALL;
    bool    accepted = false;   // false: refused, `granted` is the unchanged mask
    bool    changed  = false;   // did the applied mask actually move?
};

// Bit for bank number 1..NUM banks. Bank 0 does not exist and yields 0, so a
// caller that mixes up 0- and 1-based numbering gets "no bank" rather than a
// silently shifted-by-one map.
inline uint8_t bankBit(uint8_t bank) {
    if (bank == 0 || bank > 8) return 0;
    return (uint8_t)(1u << (bank - 1));
}

// Is this bank's emitter rail allowed to light?
inline bool enabled(const State& s, uint8_t bank) {
    const uint8_t bit = bankBit(bank);
    return bit != 0 && (s.mask & bit) != 0;
}

// How many banks the mask turns on. Drives the reported active gate count and
// the log line; also the cheapest way to say "this counter is running on a
// third of its entrance".
inline uint8_t enabledCount(const State& s) {
    uint8_t n = 0;
    for (uint8_t bit = 1; bit; bit = (uint8_t)(bit << 1)) {
        if (s.mask & bit) n++;
    }
    return n;
}

// Apply a requested mask, honouring the three rules above.
inline Request request(State& s, uint8_t requested) {
    Request r;
    const uint8_t sane = (uint8_t)(requested & beecounter_proto::BANK_MASK_ALL);
    if (sane == 0) {
        // Refused. Leave the counter counting on whatever it already had.
        r.granted  = s.mask;
        r.accepted = false;
        r.changed  = false;
        return r;
    }
    r.accepted = true;
    r.changed  = sane != s.mask;
    s.mask     = sane;
    r.granted  = sane;
    return r;
}

}  // namespace bankstate
