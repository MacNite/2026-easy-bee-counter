#pragma once

// Firmware version of this HiveTraffic build.
//
// This is the *image* version, and is deliberately separate from
// beecounter_proto::PROTOCOL_VERSION (i2c_slave_protocol.h), which versions the
// wire format only. The two move independently: a bug fix that changes no
// registers bumps this and leaves the protocol version alone.
//
// It is reported over BLE as the measurement JSON's "ver" field so a HiveHub
// relay can tell what a counter is running. HiveHub compares it with
// dotted-numeric semantics (server/firmware.py::parse_version), so keep the
// MAJOR.MINOR.PATCH shape — it decides whether an OTA relay is offered at all,
// and it is the only way to confirm afterwards that an update actually took.
//
// Bump this on every released image.
#define HIVETRAFFIC_FW_VERSION "0.1.0"
