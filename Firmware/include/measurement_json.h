// ============================================================================
// measurement_json.h — serialization of the BLE measurement characteristic
// ============================================================================
//
// Pure logic, deliberately free of Arduino/NimBLE dependencies, for the same
// reason gate_logic.h is: this is the wire contract HiveHub parses, and a
// contract that can only be exercised by flashing an ESP32 and connecting a
// second ESP32 to it does not get exercised. src/ble_link.cpp owns the GATT
// plumbing and calls in here for the bytes.
//
// The format is documented in docs/ble-mode.md and versioned by
// beecounter_proto::PROTOCOL_VERSION. Keep the three in step: a field added
// here is a field HiveHub's parser has to learn, in the same revision.
//
// snprintf rather than a JSON library: the document is a flat object of ten
// fixed keys with no nesting, no arrays and no escaping to do, so there is
// nothing for a parser-builder to buy us at the cost of heap churn on every
// GATT read.
// ============================================================================

#pragma once

#include <stdint.h>
#include <stdio.h>

#include "ble_link.h"
#include "counter_protocol.h"

namespace beecounter_proto {

// Buffer the measurement document must be built into.
//
// The worst case is protocol v4 with every field saturated and a long version
// string. Counting the fixed punctuation and the maximum decimal widths:
//
//   {"fw":255,"ver":"<15>","uptime_s":4294967295,"status":255,"num_gates":255,
//    "mcps_healthy":255,"total_in":4294967295,"total_out":4294967295,
//    "glitches":4294967295,"idle_s":4294967295,"banks":255}
//
// is 203 bytes, plus the NUL — measured, not estimated, by
// test/test_measurement_json/ (which prints the number and fails if it grows
// past the buffer). 240 leaves room for a version string longer than any
// version.h has carried without another audit of this number; the truncation
// check in buildMeasurementJson() is what actually guarantees a malformed
// document is never published, so this is headroom, not a promise.
//
// (v2 fitted in ~155 bytes. Widening uptime_s and glitches to 32 bits and
// renaming gates_healthy -> mcps_healthy added ~16 bytes; v4's idle_s added
// ~21 more, both absorbed by the old 224-byte buffer. v5's "banks" added ~12
// and is what finally moved it.)
constexpr unsigned MEASUREMENT_JSON_CAPACITY = 240;

// Serialize `t` plus the image version string into `out`.
//
// Returns the number of bytes written (excluding the NUL), or a negative value
// if the document did not fit `capacity` or the underlying snprintf failed. A
// caller must treat any non-positive result as "publish nothing": a truncated
// document is not valid JSON and HiveHub would drop the whole read anyway,
// having lost the totals in it.
//
// `fw_version` is HIVETRAFFIC_FW_VERSION (version.h); it is passed in rather
// than included so this header stays testable against arbitrary version
// strings, including the empty one.
inline int buildMeasurementJson(char* out, unsigned capacity,
                                const ble::Telemetry& t,
                                const char* fw_version) {
    if (out == 0 || capacity == 0) return -1;
    const int length = snprintf(
        out, capacity,
        "{\"fw\":%u,\"ver\":\"%s\",\"uptime_s\":%lu,\"status\":%u,"
        "\"num_gates\":%u,\"mcps_healthy\":%u,\"total_in\":%lu,"
        "\"total_out\":%lu,\"glitches\":%lu,\"idle_s\":%lu,"
        "\"banks\":%u}",
        static_cast<unsigned>(t.protocol_version),
        fw_version ? fw_version : "",
        static_cast<unsigned long>(t.uptime_s),
        static_cast<unsigned>(t.status_flags),
        static_cast<unsigned>(t.num_gates),
        static_cast<unsigned>(t.mcps_healthy),
        static_cast<unsigned long>(t.total_in),
        static_cast<unsigned long>(t.total_out),
        static_cast<unsigned long>(t.glitch_count),
        static_cast<unsigned long>(t.idle_s),
        static_cast<unsigned>(t.bank_mask));
    if (length <= 0 || static_cast<unsigned>(length) >= capacity) return -1;
    return length;
}

}  // namespace beecounter_proto
