// ============================================================================
// Host-side tests for include/device_name.h
// ============================================================================
//
// The advertised name is how a person standing at a hive tells one counter
// from another, and every way of getting it wrong is quiet: a reversed suffix,
// a dropped leading zero or a truncated buffer all produce a name that looks
// like a name and points at the wrong device. Nothing on the hardware would
// flag it — you would have to already know the address you were looking for —
// so the exact bytes are pinned here.
//
//     c++ -std=c++11 -I include
//         test/test_device_name/test_device_name.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

#include "device_name.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAIL %s:%d  [%s]  %s\n", __FILE__, __LINE__,      \
                        g_case, #cond);                                      \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

#define CHECK_NAME(actual, expected)                                         \
    do {                                                                     \
        if (std::strcmp((actual), (expected)) != 0) {                        \
            std::printf("  FAIL %s:%d  [%s]  got \"%s\", want \"%s\"\n",     \
                        __FILE__, __LINE__, g_case, (actual), (expected));   \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

// A NimBLE-order address: val[5] is the byte a scanner prints first. This one
// is displayed as A4:C1:38:9F:AB:12, so its name must end "AB:12".
static const uint8_t ADDR[6] = {0x12, 0xAB, 0x9F, 0x38, 0xC1, 0xA4};

static void test_name_ends_with_the_displayed_address() {
    g_case = "the suffix is the tail of the address, in display order";
    char name[devicename::CAPACITY];
    const size_t length = devicename::build(name, sizeof(name), ADDR);
    CHECK_NAME(name, "HiveTraffic-AB:12");
    CHECK(length == std::strlen(name));
    // The bytes are the LAST two a scanner prints, not the first two of the
    // little-endian array: "12:AB" here would be the same two bytes reversed,
    // which is exactly the failure that survives a code read.
    CHECK(std::strcmp(name, "HiveTraffic-12:AB") != 0);
}

static void test_leading_zeros_are_kept() {
    // "HiveTraffic-0:5" would be a different length for every device and would
    // not line up with anything a scanner prints. Both digits, always.
    g_case = "each byte is two hex digits";
    const uint8_t addr[6] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
    char name[devicename::CAPACITY];
    devicename::build(name, sizeof(name), addr);
    CHECK_NAME(name, "HiveTraffic-00:05");
}

static void test_hex_is_uppercase() {
    // Scanners print addresses in uppercase; a lowercase suffix reads as a
    // different value at a glance when matching name against address.
    g_case = "hex digits are uppercase";
    const uint8_t addr[6] = {0xEF, 0xCD, 0x00, 0x00, 0x00, 0x00};
    char name[devicename::CAPACITY];
    devicename::build(name, sizeof(name), addr);
    CHECK_NAME(name, "HiveTraffic-CD:EF");
}

static void test_every_byte_value_round_trips() {
    // Both nibbles of both bytes, across the whole range — the table lookup
    // has no arithmetic in it, but a wrong shift or mask would show here.
    g_case = "all 65536 suffixes render as the address does";
    for (unsigned high = 0; high <= 0xFF; ++high) {
        for (unsigned low = 0; low <= 0xFF; ++low) {
            const uint8_t addr[6] = {
                (uint8_t)low, (uint8_t)high, 0x00, 0x00, 0x00, 0x00};
            char name[devicename::CAPACITY];
            char want[devicename::CAPACITY];
            devicename::build(name, sizeof(name), addr);
            std::snprintf(want, sizeof(want), "%s-%02X:%02X",
                          devicename::BASE, high, low);
            if (std::strcmp(name, want) != 0) {
                std::printf("  FAIL %s:%d  [%s]  got \"%s\", want \"%s\"\n",
                            __FILE__, __LINE__, g_case, name, want);
                ++g_failures;
                return;   // one report is enough; 65536 would not help
            }
        }
    }
}

static void test_no_address_falls_back_to_the_product_name() {
    // What ble_link.cpp does when NimBLE cannot hand it an identity address.
    // An unsuffixed name is a far better failure than no advertising, and than
    // a plausible "HiveTraffic-00:00" that every affected unit would share.
    g_case = "without an address the bare product name is used";
    char name[devicename::CAPACITY];
    const size_t length = devicename::build(name, sizeof(name), nullptr);
    CHECK_NAME(name, "HiveTraffic");
    CHECK(length == sizeof(devicename::BASE) - 1);
}

static void test_a_short_buffer_is_refused_not_truncated() {
    // A truncated suffix identifies the WRONG device, which is worse than one
    // that identifies none. The caller gets 0 and can fall back.
    g_case = "too small a buffer yields nothing";
    char name[devicename::CAPACITY];
    std::memset(name, 'x', sizeof(name));
    // One short: room for the name but not its terminator.
    CHECK(devicename::build(name, devicename::CAPACITY - 1, ADDR) == 0);
    CHECK(name[0] == 'x');            // untouched, not half-written
    CHECK(devicename::build(name, 0, ADDR) == 0);
    CHECK(devicename::build(nullptr, devicename::CAPACITY, ADDR) == 0);

    // The unsuffixed form needs less room, and is still written when only that
    // much is available.
    CHECK(devicename::build(name, sizeof(devicename::BASE), nullptr) ==
          sizeof(devicename::BASE) - 1);
    CHECK_NAME(name, "HiveTraffic");
    CHECK(devicename::build(name, sizeof(devicename::BASE) - 1, nullptr) == 0);
}

static void test_capacity_matches_the_longest_name() {
    // CAPACITY sizes every buffer in ble_link.cpp. If the product name ever
    // changes, this is the line that says so before the scan response
    // overflows.
    g_case = "CAPACITY is exactly the suffixed name plus its NUL";
    char name[devicename::CAPACITY];
    const size_t length = devicename::build(name, sizeof(name), ADDR);
    CHECK(length + 1 == devicename::CAPACITY);
    CHECK(devicename::CAPACITY == 18);          // "HiveTraffic-AB:12" + NUL
    // The scan response budget the static_assert in the header guards: two
    // bytes of AD header plus the name, inside 31.
    CHECK(2 + length <= 31);
}

static void test_two_counters_get_two_names() {
    // The entire point: distinct hardware, distinct rows in the scan list.
    g_case = "different addresses give different names";
    const uint8_t other[6] = {0x9C, 0x4F, 0x9F, 0x38, 0xC1, 0xA4};
    char a[devicename::CAPACITY];
    char b[devicename::CAPACITY];
    devicename::build(a, sizeof(a), ADDR);
    devicename::build(b, sizeof(b), other);
    CHECK(std::strcmp(a, b) != 0);
    CHECK_NAME(b, "HiveTraffic-4F:9C");
    // Only the last two bytes are in the name, so units that share them are
    // not told apart by it. This is a known and accepted limit — 1 in 65536
    // for factory-assigned addresses — recorded here so it is a decision
    // rather than a surprise.
    const uint8_t twin[6] = {0x12, 0xAB, 0x11, 0x22, 0x33, 0x44};
    char c[devicename::CAPACITY];
    devicename::build(c, sizeof(c), twin);
    CHECK(std::strcmp(a, c) == 0);
}

int main() {
    std::printf("device_name tests\n");
    test_name_ends_with_the_displayed_address();
    test_leading_zeros_are_kept();
    test_hex_is_uppercase();
    test_every_byte_value_round_trips();
    test_no_address_falls_back_to_the_product_name();
    test_a_short_buffer_is_refused_not_truncated();
    test_capacity_matches_the_longest_name();
    test_two_counters_get_two_names();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
