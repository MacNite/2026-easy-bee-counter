// ============================================================================
// Host-side tests for include/measurement_json.h
// ============================================================================
//
// This file is the counter's half of a two-repo contract: the exact bytes
// HiveHub's bee_counter_wire.h has to parse. The two defects that made the v3
// revision necessary — a uint16_t uptime that clamped at 18 hours and a
// uint16_t glitch tally that pinned at 65535 — were both invisible in a code
// read and both needed a device left running to reproduce, so they are pinned
// down here instead. v4's idle_s and v5's banks are here for the same reason
// from the other direction: both exist to say that a flat stretch of totals is
// deliberate, and both are useless if the field name drifts.
//
// Run them directly with any host compiler:
//
//     c++ -std=c++11 -I include
//         test/test_measurement_json/test_measurement_json.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

#include "measurement_json.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace beecounter_proto;

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

// Assert the document contains `needle` verbatim, printing both on failure —
// a mismatched field name is the failure mode that breaks HiveHub silently.
static void containsField(const char* json, const char* needle) {
    if (std::strstr(json, needle) == 0) {
        std::printf("  FAIL [%s]  missing %s\n     in: %s\n", g_case, needle, json);
        ++g_failures;
    }
}

static void refusesField(const char* json, const char* needle) {
    if (std::strstr(json, needle) != 0) {
        std::printf("  FAIL [%s]  unexpected %s\n     in: %s\n", g_case, needle, json);
        ++g_failures;
    }
}

// A plausible mid-deployment reading.
static ble::Telemetry nominal() {
    ble::Telemetry t;
    t.protocol_version = PROTOCOL_VERSION;
    t.status_flags     = STATUS_READY | STATUS_MCP_U2_OK | STATUS_MCP_U3_OK |
                         STATUS_MCP_U4_OK;
    t.uptime_s         = 1234;
    t.num_gates        = 24;
    t.mcps_healthy     = 3;
    t.total_in         = 100;
    t.total_out        = 95;
    t.glitch_count     = 2;
    t.idle_s           = 0;
    t.bank_mask        = BANK_MASK_ALL;
    return t;
}

// --------------------------------------------------------------------------

static void test_nominal_document() {
    g_case = "nominal v5 document";
    char json[MEASUREMENT_JSON_CAPACITY];
    const int n = buildMeasurementJson(json, sizeof(json), nominal(), "0.1.0");
    CHECK(n > 0);
    CHECK(std::strcmp(
        json,
        "{\"fw\":5,\"ver\":\"0.1.0\",\"uptime_s\":1234,\"status\":15,"
        "\"num_gates\":24,\"mcps_healthy\":3,\"total_in\":100,"
        "\"total_out\":95,\"glitches\":2,\"idle_s\":0,\"banks\":7}") == 0);
    CHECK(n == (int)std::strlen(json));
}

static void test_protocol_version_is_five() {
    // The version byte is what HiveHub branches on. If this changes without the
    // parser learning the new revision, every counter goes unreadable.
    g_case = "fw is the protocol revision, not the image version";
    CHECK(PROTOCOL_VERSION == 5);
    char json[MEASUREMENT_JSON_CAPACITY];
    buildMeasurementJson(json, sizeof(json), nominal(), "9.9.9");
    containsField(json, "\"fw\":5");
    containsField(json, "\"ver\":\"9.9.9\"");
}

static void test_bank_mask_is_always_reported() {
    // Why the field exists: eight gates that are deliberately dark and eight
    // gates whose FET has died produce the same permanently flat share of the
    // totals. Only this byte separates them — and it has to be present even
    // when nothing is switched off, or "all banks on" and "counter too old to
    // say" become the same absence.
    g_case = "banks says which MOSFETs are live";
    ble::Telemetry t = nominal();
    char json[MEASUREMENT_JSON_CAPACITY];
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.3.0") > 0);
    containsField(json, "\"banks\":7");

    t.bank_mask = 0x01;                 // only gates 00..07 counted
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.3.0") > 0);
    containsField(json, "\"banks\":1");

    t.bank_mask = 0x05;                 // banks 1 and 3
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.3.0") > 0);
    containsField(json, "\"banks\":5");

    // num_gates keeps reporting what is WIRED, not what is lit: it is the
    // board's topology, and a consumer derives the active count from the mask.
    // Moving it would silently rewrite the meaning of every stored reading.
    containsField(json, "\"num_gates\":24");
}

static void test_night_mode_is_visible_in_the_document() {
    // The reason idle_s exists: without it, a night of zero crossings and a
    // counter whose emitter FETs have died produce identical documents. The
    // status bit and the countdown are the only things that separate them.
    g_case = "a suspended counter says so";
    ble::Telemetry t = nominal();
    t.status_flags |= STATUS_NIGHT_IDLE;
    t.idle_s = 754;
    char json[MEASUREMENT_JSON_CAPACITY];
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"idle_s\":754");
    // 15 (ready + three expanders) | 0x80 = 143.
    containsField(json, "\"status\":143");

    // And a counting one reports the same fields, saying the opposite.
    t = nominal();
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"idle_s\":0");
    containsField(json, "\"status\":15");
}

static void test_night_idle_bit_does_not_collide() {
    // 0x80 was the last free bit in the status byte. If a future flag reuses
    // it, a suspended counter and whatever that flag means become the same
    // reading on the wire.
    g_case = "STATUS_NIGHT_IDLE owns bit 7 alone";
    CHECK(STATUS_NIGHT_IDLE == 0x80);
    const uint8_t others = STATUS_READY | STATUS_MCP_U2_OK | STATUS_MCP_U3_OK |
                           STATUS_MCP_U4_OK | STATUS_IR_LEDS_ON |
                           STATUS_SENSOR_FAULT_FLAG | STATUS_OVERFLOW_FLAG;
    CHECK((others & STATUS_NIGHT_IDLE) == 0);
}

static void test_uptime_past_the_old_ceiling() {
    // The whole point of widening the field: a counter up for a week used to
    // report 65535 forever, so an unexpected reset was undetectable.
    g_case = "uptime_s survives past 18 h 12 min";
    ble::Telemetry t = nominal();
    t.uptime_s = 65536;                 // one second past the old uint16_t clamp
    char json[MEASUREMENT_JSON_CAPACITY];
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"uptime_s\":65536");
    refusesField(json, "\"uptime_s\":65535");

    t.uptime_s = 604800;                // one week
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"uptime_s\":604800");

    t.uptime_s = UINT32_MAX;
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"uptime_s\":4294967295");
}

static void test_glitches_past_the_old_ceiling() {
    g_case = "glitches survive past 65535";
    ble::Telemetry t = nominal();
    t.glitch_count = 70000;
    char json[MEASUREMENT_JSON_CAPACITY];
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"glitches\":70000");

    t.glitch_count = UINT32_MAX;        // saturated, per gate_logic.h
    CHECK(buildMeasurementJson(json, sizeof(json), t, "0.2.0") > 0);
    containsField(json, "\"glitches\":4294967295");
}

static void test_healthy_count_is_named_for_what_it_counts() {
    // 3 alongside num_gates 24 read as "21 of 24 gates broken" to every
    // consumer that met the old name. The value is unchanged; the name is not.
    g_case = "mcps_healthy replaces gates_healthy";
    char json[MEASUREMENT_JSON_CAPACITY];
    buildMeasurementJson(json, sizeof(json), nominal(), "0.2.0");
    containsField(json, "\"mcps_healthy\":3");
    refusesField(json, "gates_healthy");
    containsField(json, "\"num_gates\":24");
}

static void test_saturated_worst_case_fits_the_buffer() {
    // MEASUREMENT_JSON_CAPACITY is a claim about the widest document this
    // firmware can produce. Widening two fields to 32 bits moved that number,
    // and a buffer that is one byte short publishes nothing at all.
    g_case = "every field saturated still fits";
    ble::Telemetry t;
    t.protocol_version = 255;
    t.status_flags     = 255;
    t.uptime_s         = UINT32_MAX;
    t.num_gates        = 255;
    t.mcps_healthy     = 255;
    t.total_in         = UINT32_MAX;
    t.total_out        = UINT32_MAX;
    t.glitch_count     = UINT32_MAX;
    t.idle_s           = UINT32_MAX;
    t.bank_mask        = 255;
    char json[MEASUREMENT_JSON_CAPACITY];
    const int n = buildMeasurementJson(json, sizeof(json), t, "255.255.255-rc1");
    CHECK(n > 0);
    CHECK(n < (int)MEASUREMENT_JSON_CAPACITY);
    // Headroom, so a longer version string cannot silently start truncating.
    CHECK(n <= 215);
    std::printf("  worst case is %d of %u bytes\n", n, MEASUREMENT_JSON_CAPACITY);
}

static void test_truncation_is_refused_not_published() {
    // A truncated document is not JSON; HiveHub would discard the read and lose
    // the totals in it. Better to publish nothing and log.
    g_case = "a document that does not fit is rejected";
    char json[32];
    CHECK(buildMeasurementJson(json, sizeof(json), nominal(), "0.1.0") < 0);
    CHECK(buildMeasurementJson(json, 0, nominal(), "0.1.0") < 0);
    CHECK(buildMeasurementJson(0, sizeof(json), nominal(), "0.1.0") < 0);
}

static void test_missing_version_string() {
    // version.h has always defined one, but an empty "ver" is what a counter
    // too old to report it looks like to HiveHub, and it must still be JSON.
    g_case = "empty and null version strings";
    char json[MEASUREMENT_JSON_CAPACITY];
    CHECK(buildMeasurementJson(json, sizeof(json), nominal(), "") > 0);
    containsField(json, "\"ver\":\"\"");
    CHECK(buildMeasurementJson(json, sizeof(json), nominal(), 0) > 0);
    containsField(json, "\"ver\":\"\"");
}

int main() {
    std::printf("measurement_json tests\n");
    test_nominal_document();
    test_protocol_version_is_five();
    test_bank_mask_is_always_reported();
    test_night_mode_is_visible_in_the_document();
    test_night_idle_bit_does_not_collide();
    test_uptime_past_the_old_ceiling();
    test_glitches_past_the_old_ceiling();
    test_healthy_count_is_named_for_what_it_counts();
    test_saturated_worst_case_fits_the_buffer();
    test_truncation_is_refused_not_published();
    test_missing_version_string();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
