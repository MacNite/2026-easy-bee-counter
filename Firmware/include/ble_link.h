#pragma once

#include <stdint.h>

namespace ble {

// One measurement snapshot, as reported by the JSON characteristic. Field
// widths are the wire contract's, not an implementation detail — see
// docs/ble-mode.md.
struct Telemetry {
    uint8_t protocol_version;
    uint8_t status_flags;
    // 32-bit since protocol v3. As a uint16_t this clamped at 65535 s
    // (18 h 12 min), so an always-on counter reported one constant number for
    // its whole deployment and the field could not do the only job it has:
    // showing that the device restarted unexpectedly. 32 bits is ~136 years.
    uint32_t uptime_s;
    uint8_t num_gates;
    // MCP23017 port expanders currently answering on the I2C bus, 0..3 — NOT
    // gates, of which there are num_gates (24). Each healthy expander covers
    // eight gates. Named "gates_healthy" on the wire until protocol v3, which
    // is exactly the misreading the rename fixes.
    uint8_t mcps_healthy;
    uint32_t total_in;
    uint32_t total_out;
    // 32-bit since protocol v3; saturating, never wrapping (gate_logic.h).
    uint32_t glitch_count;
    // Seconds of night-mode suspension still to run, 0 when counting. New in
    // protocol v4, alongside STATUS_NIGHT_IDLE. It is what lets HiveHub — and
    // anyone reading the stored history later — tell "no bees flew" from "this
    // counter was deliberately not looking", which the totals alone cannot say.
    uint32_t idle_s;
    // Enabled emitter banks, one bit per MOSFET (bit 0 = bank 1 = gates
    // 00..07, and so on). New in protocol v5, reported as "banks". A cleared
    // bit means those eight gates are dark and deliberately not counted, so
    // their share of the totals stays flat — which without this field is
    // indistinguishable from the FET having died. 0x07 on any counter that
    // has not been told otherwise.
    uint8_t bank_mask;
};

void getTelemetry(Telemetry& out);

// Apply a night-mode request written to the control characteristic.
// Implemented in main.cpp, where the suspension state and the emitters live;
// declared here because ble_link.cpp is what receives the write.
// `duration_s` of 0 resumes sensing. Returns the duration actually granted
// after clamping to beecounter_proto::MAX_IDLE_SECONDS.
uint32_t applyIdleRequest(uint32_t duration_s);

// Seconds of suspension left, for the control characteristic's read-back.
uint32_t idleRemainingSeconds();

// Apply an emitter-bank enable mask written to the control characteristic.
// Implemented in main.cpp alongside applyIdleRequest(), for the same reason:
// the FET pins and the gate state machines live there. Returns the mask
// actually in force afterwards, which is the unchanged one if the request was
// refused (see bank_state.h — a mask of 0 is never applied).
uint8_t applyBankMask(uint8_t mask);

// The enabled-bank mask currently in force, for the control characteristic's
// read-back.
uint8_t bankMask();

void begin();
bool isOtaActive();
void loopOta();

}  // namespace ble
