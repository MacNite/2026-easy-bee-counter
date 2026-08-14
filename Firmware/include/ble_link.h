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
};

void getTelemetry(Telemetry& out);
void begin();
bool isOtaActive();
void loopOta();

}  // namespace ble
