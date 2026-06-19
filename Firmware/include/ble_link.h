// ============================================================================
// ble_link.h — optional BLE/GATT transport for the Easy Bee Counter 2026
// ----------------------------------------------------------------------------
// This is a COMPILE-TIME ALTERNATIVE to the I2C slave link. When the firmware
// is built with -DBEECOUNTER_BLE the device drops the Wire1 slave entirely and
// instead runs a connectable NimBLE GATT server, mirroring HiveInside's
// wireless transfer (see HiveInside/firmware/src/ble_link.cpp). The default
// build keeps the wired I2C protocol untouched — see platformio.ini.
//
// Why this is cheap for the bee counter specifically: its power budget is
// dominated by pulsing the 48 IR emitters, so the BLE radio staying
// advertising-connectable adds negligible average draw. We therefore skip ALL
// of HiveInside's deep-sleep / wake-sync machinery: the device just keeps
// counting and stays connectable forever, exactly like the I2C slave is always
// available on the wire.
//
// GATT layout
// -----------
//   * Standard Battery service is intentionally omitted (no battery sense).
//   * Custom BeeCounter service:
//       - Measurement characteristic (READ + NOTIFY): a compact JSON document
//         with the aggregate IN/OUT counts the HiveScale actually uploads —
//         totals, the latched interval, plus status/health/uptime for
//         diagnostics. Per-gate detail (debug-only) is deliberately NOT sent,
//         keeping the JSON far under the 512-byte ATT attribute cap.
//       - Control characteristic (WRITE): one CMD_* byte (LATCH / CLEAR_* /
//         LEDS_*), same opcodes as the I2C REG_CTRL register.
//       - OTA characteristics (firmware-over-BLE): control + payload (WRITE)
//         and a readable/notifiable status, identical scheme to HiveInside.
//
// The counter state itself lives in main.cpp; this module pulls a telemetry
// snapshot and applies control commands through the small callback API below,
// which main.cpp implements.
// ============================================================================
#pragma once

#ifdef BEECOUNTER_BLE

#include <stdint.h>
#include <stddef.h>

namespace ble {

// ── Telemetry snapshot ─────────────────────────────────────────────────────
// main.cpp fills this from its (volatile) counters when we publish. It carries
// only the aggregate figures the backend consumes — no per-gate arrays.
struct Telemetry {
    uint8_t  protocol_version;
    uint8_t  status_flags;     // beecounter_proto::STATUS_* bitfield
    uint16_t uptime_s;
    uint8_t  num_gates;
    uint8_t  gates_healthy;
    uint32_t total_in;
    uint32_t total_out;
    uint32_t interval_in;      // latched (shadow) interval counters
    uint32_t interval_out;
    uint16_t glitch_count;
};

// ── Provided by main.cpp (the counter owns this state) ──────────────────────
// Populate a fresh telemetry snapshot for the measurement characteristic.
void getTelemetry(Telemetry& out);
// Apply a REG_CTRL-style command byte (CMD_LATCH / CMD_CLEAR_* / CMD_LEDS_*).
void applyCtrl(uint8_t cmd);

// ── GATT server lifecycle (implemented here, called by main.cpp) ────────────
// Initialise the radio and start the connectable GATT server. Call once in
// setup() (only in the BLE build).
void begin();
// Refresh the measurement characteristic from getTelemetry() and notify any
// subscribers. Call periodically from loop().
void publish();

// ── Firmware-over-BLE (OTA) ─────────────────────────────────────────────────
// True while an image is actively being received, or a verified image is staged
// and a reboot is pending. main.cpp uses this to pause gate polling and hold the
// emitters dark for the duration (mirrors the I2C OTA behaviour).
bool isOtaActive();
// Pump the OTA state machine: when a verified image is staged, reboot into it.
// Call once per loop() iteration; no-op when no OTA is in progress.
void loopOta();

}  // namespace ble

#endif  // BEECOUNTER_BLE
