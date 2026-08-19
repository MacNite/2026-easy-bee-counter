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
// v4 = adds the "idle_s" field and the STATUS_NIGHT_IDLE bit, so a document
//      says whether the counter is deliberately not sensing (night mode) and
//      for how much longer. Without it a night of zero crossings is
//      indistinguishable from a counter whose emitters have failed.
//
// A counter in the field keeps emitting an older revision until it is updated
// over the air, and the OTA relay has to read this very characteristic before
// it can update anything — so HiveHub's parser reads "fw" first and accepts
// every revision. Its tolerant parser must be deployed BEFORE any counter
// emitting the new one.
constexpr uint8_t PROTOCOL_VERSION = 4;

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
// Sensing is deliberately suspended (night mode): the emitters are dark, the
// gates are not polled and the totals are frozen. This is the bit that keeps a
// night of zero crossings from reading as a dead counter — see idle_state.h for
// the deadline that clears it, and the control characteristic below for who
// sets it.
constexpr uint8_t STATUS_NIGHT_IDLE         = 0x80;

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

// --------------------------------------------------------------------------
// Control characteristic — night mode / sensing suspension
// --------------------------------------------------------------------------
// The counter has never had an input other than OTA. This is the second one,
// and it is deliberately the smallest thing that can express "stop sensing":
// HiveHub writes a DURATION, never a schedule and never a wall-clock time.
//
// Why a deadline rather than a schedule
// -------------------------------------
// The counter has no RTC, no NVS and no idea what time it is; giving it a
// 20:00-06:00 window would mean teaching it all three, and every one of them is
// a way for a counter to end up permanently blind on its own. A duration cannot
// do that: it expires. HiveHub knows the time (NTP + a DS3231 at +/-2 ppm) and
// re-arms the idle window once per upload cycle, so the counter's own clock
// only has to be right for one cycle at a time and nothing accumulates.
//
// Everything about this is fail-open. An idle request is capped at
// MAX_IDLE_SECONDS; the state is never persisted, so any reset resumes
// counting; and a HiveHub that stops calling simply lets the deadline run out.
// The failure mode of the whole feature is "the counter counts", which is the
// behaviour it had before this existed.
constexpr uint8_t CTRL_OP_SET_IDLE = 0x01;   // + duration_s (4 LE)
constexpr uint8_t CTRL_OP_RESUME   = 0x02;   // no payload: sense again now

// Longest suspension the counter will accept, whatever HiveHub asks for. One
// hour is several times HiveHub's default 10-minute upload cycle — enough that
// a couple of missed cycles do not wake the emitters up in the middle of the
// night — while bounding how long a counter can stay blind after HiveHub falls
// off the air entirely. A longer request is CLAMPED to this rather than
// refused: refusing would leave the emitters running all night because one
// field was too large.
constexpr uint32_t MAX_IDLE_SECONDS = 3600;

// Control status, as read back from the control characteristic:
//     state(1) + remaining_s(4 LE)
// state is one of the two below; remaining_s is 0 unless idle.
constexpr uint8_t CTRL_STATE_SENSING = 0x00;
constexpr uint8_t CTRL_STATE_IDLE    = 0x01;

// Bytes in that read-back value.
constexpr uint8_t CTRL_STATUS_LENGTH = 5;

// Bytes in a well-formed SET_IDLE write (opcode + uint32 LE).
constexpr uint8_t CTRL_SET_IDLE_LENGTH = 5;

}  // namespace beecounter_proto
