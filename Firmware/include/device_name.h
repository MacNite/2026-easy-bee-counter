// ============================================================================
// device_name.h — the advertised BLE name, "HiveTraffic-AB:12"
// ============================================================================
//
// Every counter used to advertise the identical local name, so an apiary with
// several in range produced a scan list of identical rows and the only way to
// tell one entry from another was to open a scanner's detail view and read the
// address. The name now carries the last two bytes of the counter's own BLE
// address, formatted the way a scanner prints them, so the list is
// self-identifying:
//
//     HiveTraffic-AB:12    <- the node whose address ends ...:AB:12
//     HiveTraffic-4F:9C
//
// Nothing on the wire keys off the name. HiveHub connects by the MAC paired in
// its portal (docs/ble-mode.md), so this is a display string for whoever is
// standing at the hive with a phone — during pairing, or when working out
// which of three counters on a bench is the one being flashed.
//
// Arduino-free, like gate_logic.h and bank_state.h, and pinned by
// test/test_device_name/ for the same reason: the failure mode is quiet. A
// wrong byte order or an off-by-one in the buffer produces a name that still
// looks plausible in a scan list while pointing at the wrong device, and the
// only way to notice on hardware is to already know the address you were
// looking for. src/ble_link.cpp owns the radio and calls in here for the
// string.
//
// Byte order
// ----------
// NimBLE stores an address little-endian — val[5] is the byte a scanner prints
// first, val[0] the last — so the two bytes wanted here are val[1] and val[0],
// in that order. Getting this backwards is the mistake this header exists to
// prevent: it yields a suffix that is a real part of the address, just
// reversed, so it passes a glance and fails exactly when someone tries to
// match it against what the scanner shows.
//
// Which address ends up in the name
// ---------------------------------
// The one the counter advertises with. NimBLE settles its own-address type
// while starting up and both the address it reports and the packets it sends
// follow it, so the two always agree. In practice that is the controller's
// public address — on the ESP32-C6 the factory eFuse MAC: unique per unit and
// stable across reboots, reflashes and OTA updates, with nothing to provision
// per device. The suffix is therefore literally the tail of the address the
// scanner shows beside the entry.
// ============================================================================

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace devicename {

// The product half of the name, without the address suffix. Also the fallback
// if the address cannot be read.
constexpr char BASE[] = "HiveTraffic";

// "-AB:12": a separator, two hex digits, a colon, two more.
constexpr size_t SUFFIX_LENGTH = 6;

// Longest name this header produces, NUL included. sizeof(BASE) already counts
// the terminator.
constexpr size_t CAPACITY = sizeof(BASE) + SUFFIX_LENGTH;

// A legacy scan response holds 31 bytes, of which an AD structure spends two on
// its length and type. ble_link.cpp puts the name in the scan response alone,
// so this is the whole budget it has to fit in — checked here rather than in a
// comment there, because the name is what would grow.
static_assert(CAPACITY - 1 + 2 <= 31,
              "the advertised name does not fit a legacy scan response");

// Build the advertised name into `out`, returning its length (excluding the
// NUL) or 0 if the buffer is too small to hold the complete name.
//
// `addr_val` is a NimBLE-order (little-endian) six-byte address, or nullptr
// when none could be read — in which case the bare product name is used. An
// unsuffixed name is a far better failure than no name, or than a plausible
// "HiveTraffic-00:00" that several counters would then share.
//
// Refusing to truncate is deliberate: a half-written suffix is a name that
// identifies the wrong device, which is worse than one that identifies no
// device in particular.
inline size_t build(char* out, size_t capacity, const uint8_t* addr_val) {
    // Lower-case deliberately: this header is compiled after Arduino.h, whose
    // Print.h defines `HEX` as 16. An all-caps name here is not a style choice
    // but a build break — see the macro block in test/test_device_name/.
    static const char hex_digits[] = "0123456789ABCDEF";

    if (out == nullptr) return 0;

    const size_t base_length = sizeof(BASE) - 1;
    const size_t length =
        base_length + (addr_val != nullptr ? SUFFIX_LENGTH : 0);
    if (capacity < length + 1) return 0;

    size_t i = 0;
    for (; i < base_length; ++i) out[i] = BASE[i];

    if (addr_val != nullptr) {
        out[i++] = '-';
        out[i++] = hex_digits[addr_val[1] >> 4];
        out[i++] = hex_digits[addr_val[1] & 0x0F];
        out[i++] = ':';
        out[i++] = hex_digits[addr_val[0] >> 4];
        out[i++] = hex_digits[addr_val[0] & 0x0F];
    }

    out[i] = '\0';
    return i;
}

}  // namespace devicename
