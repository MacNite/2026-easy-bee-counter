#pragma once

// Firmware version of this HiveTraffic build.
//
// This is the *image* version, and is deliberately separate from
// beecounter_proto::PROTOCOL_VERSION (counter_protocol.h), which versions the
// reported measurement format. The two move independently: a bug fix that
// changes no reported fields bumps this and leaves the format revision alone.
//
// It is reported over BLE as the measurement JSON's "ver" field so a HiveHub
// relay can tell what a counter is running. HiveHub compares it with
// dotted-numeric semantics (server/firmware.py::parse_version), so keep the
// MAJOR.MINOR.PATCH shape — it decides whether an OTA relay is offered at all,
// and it is the only way to confirm afterwards that an update actually took.
//
// Bump this on every released image.
#define HIVETRAFFIC_FW_VERSION "0.3.0"
