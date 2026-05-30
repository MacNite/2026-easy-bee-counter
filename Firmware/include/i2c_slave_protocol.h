// ============================================================================
// i2c_slave_protocol.h — Bee counter <-> HiveScale wire format
// ============================================================================
//
// The HiveScale ESP32 polls this bee counter on the shared I2C bus whenever
// it wakes from deep sleep (every ~10 minutes). This file defines the
// register-style protocol both sides agree on.
//
// Slave address: 0x30  (see pins.h -> i2c_addr::BEECOUNTER_SLAVE)
//
// Transaction style
// -----------------
// Standard "register-pointer" pattern:
//   1. master WRITE 1 byte (the register address to read from)
//   2. master READ  N bytes (the contents of the register, MSB first)
//
// The bee counter implements this in software (Wire.onReceive + Wire.onRequest)
// because the ESP32-C6 has only one hardware I2C controller; it spends most
// of its time as MASTER polling the MCP23017s and switches into SLAVE mode
// to answer the HiveScale's transaction. See main.cpp for the mode-switch
// state machine and the inevitable race-window mitigation.
//
// All multi-byte values are sent BIG-ENDIAN on the wire.
// Counters wrap at 2^32 - 1 (they cannot be negative).
// ============================================================================

#pragma once

#include <stdint.h>

namespace beecounter_proto {

// Protocol revision. Increment whenever a register layout changes.
// v2 = added the OTA-over-I2C register block (REG_OTA_*, 0x90..0x94).
constexpr uint8_t PROTOCOL_VERSION = 2;

// --------------------------------------------------------------------------
// Register map
// --------------------------------------------------------------------------
// Single-byte register addresses written by the master before a read.
// Multi-byte registers MUST be read in a single transaction; the slave
// streams them MSB-first.
//
//  Addr  Width   Name                  Description
//  ----  -----   --------------------- ------------------------------------
//  0x00   1      REG_PROTOCOL_VERSION  Protocol revision (see above)
//  0x01   1      REG_STATUS            Bitfield, see STATUS_* below
//  0x02   2      REG_UPTIME_S          Uptime in seconds (clipped at 0xFFFF)
//  0x04   1      REG_NUM_GATES         Always equals gates::NUM_GATES (24)
//  0x05   1      REG_GATES_HEALTHY     Count of MCP chips that ACK'd at boot
//                                      (range 0..3)
//
//  0x10   4      REG_TOTAL_IN          Bees that crossed inbound since boot
//                                      OR since last REG_CTRL clear
//  0x14   4      REG_TOTAL_OUT         Bees that crossed outbound likewise
//  0x18   4      REG_INTERVAL_IN       Bees inbound since last successful
//                                      poll-and-clear (REG_CTRL=CMD_LATCH)
//  0x1C   4      REG_INTERVAL_OUT      Bees outbound since last poll-and-clear
//  0x20   2      REG_GLITCH_COUNT      Number of detected pattern glitches
//                                      since boot (sensor noise events)
//  0x22   2      REG_BUSY_RETRIES      Number of times the slave callback
//                                      had to NACK / clock-stretch because
//                                      it was mid-master-cycle
//
//  0x30  24      REG_PER_GATE_IN       For each of 24 gates: 1 byte count
//                                      of inbound crossings since last
//                                      LATCH (clipped at 255). Order is
//                                      the gates::TABLE[] order.
//  0x48  24      REG_PER_GATE_OUT      Same, outbound.
//
//  0x80   1      REG_CTRL              WRITE-ONLY. Send a CMD_* value.
//                                      Reading returns 0xFF.
// --------------------------------------------------------------------------

constexpr uint8_t REG_PROTOCOL_VERSION = 0x00;
constexpr uint8_t REG_STATUS           = 0x01;
constexpr uint8_t REG_UPTIME_S         = 0x02;   // 2 bytes
constexpr uint8_t REG_NUM_GATES        = 0x04;
constexpr uint8_t REG_GATES_HEALTHY    = 0x05;

constexpr uint8_t REG_TOTAL_IN         = 0x10;   // 4 bytes
constexpr uint8_t REG_TOTAL_OUT        = 0x14;   // 4 bytes
constexpr uint8_t REG_INTERVAL_IN      = 0x18;   // 4 bytes
constexpr uint8_t REG_INTERVAL_OUT     = 0x1C;   // 4 bytes
constexpr uint8_t REG_GLITCH_COUNT     = 0x20;   // 2 bytes
constexpr uint8_t REG_BUSY_RETRIES     = 0x22;   // 2 bytes

constexpr uint8_t REG_PER_GATE_IN      = 0x30;   // 24 bytes
constexpr uint8_t REG_PER_GATE_OUT     = 0x48;   // 24 bytes

constexpr uint8_t REG_CTRL             = 0x80;

// --------------------------------------------------------------------------
// REG_STATUS bitfield
// --------------------------------------------------------------------------
constexpr uint8_t STATUS_READY              = 0x01;   // boot complete, polling
constexpr uint8_t STATUS_MCP_U2_OK          = 0x02;
constexpr uint8_t STATUS_MCP_U3_OK          = 0x04;
constexpr uint8_t STATUS_MCP_U4_OK          = 0x08;
constexpr uint8_t STATUS_IR_LEDS_ON         = 0x10;
constexpr uint8_t STATUS_SENSOR_FAULT_FLAG  = 0x20;   // a gate is stuck low/high
constexpr uint8_t STATUS_OVERFLOW_FLAG      = 0x40;   // a totals counter wrapped

// --------------------------------------------------------------------------
// REG_CTRL commands (write to REG_CTRL)
// --------------------------------------------------------------------------
constexpr uint8_t CMD_NOP            = 0x00;
constexpr uint8_t CMD_LATCH          = 0x01;   // Atomically copy live interval
                                               // counters into the readout
                                               // shadow and zero the live
                                               // ones. The HiveScale should
                                               // send this AFTER a successful
                                               // read of the interval/per-gate
                                               // registers so the next read
                                               // gets a fresh interval.
constexpr uint8_t CMD_CLEAR_TOTALS   = 0x02;   // Zero the lifetime totals
                                               // (REG_TOTAL_IN/OUT). Rarely
                                               // used; intended for testing.
constexpr uint8_t CMD_CLEAR_FAULTS   = 0x04;   // Clear STATUS_SENSOR_FAULT_FLAG
                                               // and STATUS_OVERFLOW_FLAG.
constexpr uint8_t CMD_LEDS_OFF       = 0x10;   // Force IR LEDs off (debug).
constexpr uint8_t CMD_LEDS_ON        = 0x11;   // Force IR LEDs on  (debug).
constexpr uint8_t CMD_LEDS_AUTO      = 0x12;   // Restore normal LED management.

// --------------------------------------------------------------------------
// OTA-over-I2C register block (PROTOCOL_VERSION >= 2)
// --------------------------------------------------------------------------
// The HiveScale relays a firmware image to this BeeCounter over I2C. The
// HiveScale downloads the image over WiFi, then streams it here in frames.
// All multi-byte fields are BIG-ENDIAN, consistent with the rest of the map.
//
//  Addr  Dir    Name              Payload / meaning
//  ----  -----  ----------------- --------------------------------------------
//  0x90  WRITE  REG_OTA_BEGIN     8 bytes: size(4) + crc32(4). Slave prepares
//                                  the OTA partition (Update.begin) and enters
//                                  OTA_STATE_RECEIVING. Gate polling pauses.
//  0x91  WRITE  REG_OTA_DATA      offset(4) + data(1..OTA_CHUNK_MAX). Slave
//                                  Update.write()s the payload if offset
//                                  matches bytes-received-so-far.
//  0x92  WRITE  REG_OTA_END       0 bytes. Slave verifies size + CRC, calls
//                                  Update.end(true), enters OTA_STATE_DONE,
//                                  and reboots shortly after.
//  0x93  WRITE  REG_OTA_ABORT     0 bytes. Slave aborts and returns to IDLE.
//  0x94  READ   REG_OTA_STATUS    6 bytes: state(1) + received(4) + err(1).
// --------------------------------------------------------------------------

constexpr uint8_t REG_OTA_BEGIN  = 0x90;   // write 8 bytes (size + crc32)
constexpr uint8_t REG_OTA_DATA   = 0x91;   // write offset(4) + data
constexpr uint8_t REG_OTA_END    = 0x92;   // write 0 bytes -> finalize
constexpr uint8_t REG_OTA_ABORT  = 0x93;   // write 0 bytes -> cancel
constexpr uint8_t REG_OTA_STATUS = 0x94;   // read 6 bytes

// Max data-payload bytes per REG_OTA_DATA frame. The ESP32 Wire RX buffer is
// 128 bytes by default; 64 keeps reg(1)+offset(4)+data inside one callback
// with comfortable headroom.
constexpr uint8_t OTA_CHUNK_MAX = 64;

// OTA state machine values (REG_OTA_STATUS byte 0). Any value >= the first
// error code (0x10) means a fatal error occurred and the transfer is dead.
constexpr uint8_t OTA_STATE_IDLE      = 0x00;
constexpr uint8_t OTA_STATE_RECEIVING = 0x01;
constexpr uint8_t OTA_STATE_DONE      = 0x02;   // verified, will reboot
constexpr uint8_t OTA_STATE_ERR_BEGIN = 0x10;   // Update.begin() failed
constexpr uint8_t OTA_STATE_ERR_SEQ   = 0x11;   // offset mismatch
constexpr uint8_t OTA_STATE_ERR_WRITE = 0x12;   // Update.write() short/failed
constexpr uint8_t OTA_STATE_ERR_CRC   = 0x13;   // final CRC mismatch
constexpr uint8_t OTA_STATE_ERR_SIZE  = 0x14;   // received != declared size
constexpr uint8_t OTA_STATE_ERR_END   = 0x15;   // Update.end() failed

constexpr uint8_t OTA_ERR_NONE = 0x00;

// --------------------------------------------------------------------------
// Convenience: total size of the per-gate arrays, kept in sync with the
// hardware gate count in pins.h.
// --------------------------------------------------------------------------
constexpr uint8_t PER_GATE_ARRAY_LEN = 24;

}  // namespace beecounter_proto
