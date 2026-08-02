#pragma once

#include <stdint.h>

namespace ble {

struct Telemetry {
    uint8_t protocol_version;
    uint8_t status_flags;
    uint16_t uptime_s;
    uint8_t num_gates;
    uint8_t gates_healthy;
    uint32_t total_in;
    uint32_t total_out;
    uint16_t glitch_count;
};

void getTelemetry(Telemetry& out);
void begin();
bool isOtaActive();
void loopOta();

}  // namespace ble
