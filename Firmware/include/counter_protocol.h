// ============================================================================
// counter_protocol.h — HiveTraffic counter wire contract
// ============================================================================
//
// What the counter reports over BLE, and the states its OTA target moves
// through. Both are shared between main.cpp (which owns the counter state) and
// ble_link.cpp (which serializes it), which is why they live in a header rather
// than in either translation unit.
//
// This file replaces i2c_slave_protocol.h. That header described a register map
// the HiveScale polled over a dedicated I2C slave bus — big-endian registers, a
// CMD_LATCH interval/per-gate readout, and an OTA-over-I2C block. That whole
// transport is gone: BLE/GATT is the only link, and HiveHub dropped its wired
// counter client. What survived is the handful of values that were never
// really about I2C.
//
// The BLE wire format itself (field names, characteristic UUIDs, frame layout)
// is defined in docs/ble-mode.md and implemented in src/ble_link.cpp.
// ============================================================================

#pragma once

#include <stdint.h>

namespace beecounter_proto {

// Revision of the reported measurement format, sent as the JSON "fw" field.
// Increment when the set or meaning of reported fields changes.
//
// NOT the firmware version — that is HIVETRAFFIC_FW_VERSION in version.h, is
// sent as "ver", and moves independently. HiveHub gates OTA relays on "ver" and
// uses this only to understand the document it is parsing.
//
// v2 = the last wired revision (it numbered an OTA-over-I2C register block).
//      The value was kept rather than reset so a HiveHub that has seen both
//      generations never sees the format revision go backwards.
// v3 = uptime_s and glitches widened to 32 bits, and "gates_healthy" renamed to
//      "mcps_healthy" to say what it has always counted. See the revision
//      history in docs/ble-mode.md for the full delta.
//
// A counter in the field keeps emitting v2 until it is updated over the air,
// and the OTA relay has to read this very characteristic before it can update
// anything — so HiveHub's parser reads "fw" first and accepts both revisions.
// Its tolerant parser must be deployed BEFORE any counter emitting v3.
constexpr uint8_t PROTOCOL_VERSION = 3;

// --------------------------------------------------------------------------
// Status bitfield — reported as the JSON "status" field
// --------------------------------------------------------------------------
// The three MCP bits track LIVE health, not boot-time discovery: a chip that
// stops answering clears its bit within MCP_FAIL_THRESHOLD polls, and one that
// starts answering again (or was missing at boot) sets it on the next
// successful re-probe. A cleared bit means that chip's eight gates are not
// being counted at all — they are skipped rather than read as all-blocked.
constexpr uint8_t STATUS_READY              = 0x01;   // boot complete, polling
constexpr uint8_t STATUS_MCP_U2_OK          = 0x02;
constexpr uint8_t STATUS_MCP_U3_OK          = 0x04;
constexpr uint8_t STATUS_MCP_U4_OK          = 0x08;
constexpr uint8_t STATUS_IR_LEDS_ON         = 0x10;
constexpr uint8_t STATUS_SENSOR_FAULT_FLAG  = 0x20;   // a gate is stuck low/high
// A lifetime total has reached UINT32_MAX. The counters saturate rather than
// wrap, so the reported value stays pinned at the maximum and stays monotonic;
// this flag is what distinguishes "pinned" from "stopped counting".
constexpr uint8_t STATUS_OVERFLOW_FLAG      = 0x40;

// --------------------------------------------------------------------------
// OTA state machine — byte 0 of the OTA status characteristic
// --------------------------------------------------------------------------
// Any value >= the first error code (0x10) means the transfer failed and is
// dead; the running image is untouched. These values are deliberately identical
// to HiveInside's, so HiveHub drives both devices with one state machine.
constexpr uint8_t OTA_STATE_IDLE      = 0x00;
constexpr uint8_t OTA_STATE_RECEIVING = 0x01;
constexpr uint8_t OTA_STATE_DONE      = 0x02;   // verified, will reboot
constexpr uint8_t OTA_STATE_ERR_BEGIN = 0x10;   // Update.begin() failed
constexpr uint8_t OTA_STATE_ERR_SEQ   = 0x11;   // out-of-sequence frame
constexpr uint8_t OTA_STATE_ERR_WRITE = 0x12;   // Update.write() short/failed
constexpr uint8_t OTA_STATE_ERR_CRC   = 0x13;   // final CRC mismatch
constexpr uint8_t OTA_STATE_ERR_SIZE  = 0x14;   // received != declared size
constexpr uint8_t OTA_STATE_ERR_END   = 0x15;   // Update.end() failed

constexpr uint8_t OTA_ERR_NONE = 0x00;

}  // namespace beecounter_proto
