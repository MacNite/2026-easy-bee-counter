// ============================================================================
// ble_link.cpp — connectable NimBLE GATT server for the Easy Bee Counter 2026
// ----------------------------------------------------------------------------
// Compiled only in the BLE build (-DBEECOUNTER_BLE). A direct sibling of
// HiveInside/firmware/src/ble_link.cpp: same NimBLE 2.x stack, same compact-JSON
// measurement characteristic, and the same firmware-over-BLE OTA scheme (framed
// BEGIN/END/ABORT control + streamed payload + readable/notifiable status, with
// an end-to-end CRC-32 verified before anything is committed).
//
// Unlike HiveInside this device never sleeps (its power is dominated by the IR
// emitters), so there is no wake-sync / deep-sleep machinery here — it just
// stays advertising-connectable while main.cpp keeps counting.
//
// The counter state lives in main.cpp; we reach it only through the
// ble::getTelemetry() / ble::applyCtrl() callbacks declared in ble_link.h.
// ============================================================================
#include "ble_link.h"

#ifdef BEECOUNTER_BLE

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <Update.h>          // ESP32 OTA writer (esp_ota under the hood)

#include "i2c_slave_protocol.h"   // shared CMD_* and OTA_STATE_* opcodes

namespace ble {

// ── Advertised name ─────────────────────────────────────────────────────────
static const char* BLE_DEVICE_NAME = "BeeCounter";

// ── Custom GATT service + characteristic UUIDs ──────────────────────────────
// A distinct 128-bit base from HiveInside's so a HiveScale central can tell the
// two devices apart. (HiveInside uses 8e8b00xx-…; we use 8e8b01xx-… .)
static const char* SVC_BEECOUNTER   = "8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01";
static const char* CHR_MEASUREMENT  = "8e8b0102-7a1c-4b9e-9a2f-1d6e0b9c1a01"; // read/notify JSON
static const char* CHR_CTRL         = "8e8b0103-7a1c-4b9e-9a2f-1d6e0b9c1a01"; // write 1 CMD byte

// Firmware-over-BLE characteristics (same custom service). Mirrors the
// HiveInside OTA UUIDs/semantics so a future HiveScale relay can reuse its code.
static const char* CHR_OTA_CTRL     = "8e8b0110-7a1c-4b9e-9a2f-1d6e0b9c1a01"; // write: framed control
static const char* CHR_OTA_DATA     = "8e8b0111-7a1c-4b9e-9a2f-1d6e0b9c1a01"; // write: payload stream
static const char* CHR_OTA_STATUS   = "8e8b0113-7a1c-4b9e-9a2f-1d6e0b9c1a01"; // read/notify: state+recv+err

// ===========================================================================
// Measurement characteristic
// ===========================================================================
static NimBLECharacteristic* chrMeasurement = nullptr;

// Serialise the aggregate telemetry to JSON. Only the figures the backend
// uploads (totals + latched interval) plus diagnostics — no per-gate arrays —
// so this lands well under the 512-byte ATT cap with room to spare.
static String measurementJson(const Telemetry& t) {
    JsonDocument doc;
    doc["fw"]            = t.protocol_version;   // protocol/firmware revision
    doc["uptime_s"]      = t.uptime_s;
    doc["status"]        = t.status_flags;
    doc["num_gates"]     = t.num_gates;
    doc["gates_healthy"] = t.gates_healthy;
    doc["total_in"]      = t.total_in;
    doc["total_out"]     = t.total_out;
    doc["interval_in"]   = t.interval_in;
    doc["interval_out"]  = t.interval_out;
    doc["glitches"]      = t.glitch_count;
    String out;
    serializeJson(doc, out);
    return out;
}

// ===========================================================================
// Control characteristic — a single REG_CTRL-style command byte
// ===========================================================================
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        NimBLEAttValue v = c->getValue();
        if (v.size() < 1) return;
        uint8_t cmd = v.data()[0];
        Serial.printf("[BLE] CTRL cmd 0x%02X\n", cmd);
        applyCtrl(cmd);
        // Reflect the post-command state immediately (e.g. cleared interval
        // after CMD_LATCH) so a subscriber sees the fresh values.
        publish();
    }
};

// ===========================================================================
// Server callbacks — keep advertising after a disconnect
// ===========================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        Serial.println("[BLE] central connected");
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        Serial.printf("[BLE] central disconnected (reason %d); re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// ===========================================================================
// Firmware-over-BLE (OTA target)
// ===========================================================================
// HiveScale streams a new image into CHR_OTA_DATA in order; we write each chunk
// straight to the inactive OTA slot via the Arduino Update API and keep a
// running CRC-32. CHR_OTA_CTRL frames BEGIN / END / ABORT; CHR_OTA_STATUS lets
// the central poll progress and the final result. Nothing is committed until
// END verifies the end-to-end CRC, so a dropped/corrupted transfer always
// leaves the device on its current image. Byte-for-byte the HiveInside scheme.

// Control opcodes (first byte of a CTRL write) — match HiveInside.
static constexpr uint8_t OTA_OP_BEGIN = 0x01;  // + size(4 LE) + crc32(4 LE)
static constexpr uint8_t OTA_OP_END   = 0x03;  // finalize, verify, reboot
static constexpr uint8_t OTA_OP_ABORT = 0x04;  // cancel

// We report progress using the shared beecounter_proto::OTA_STATE_* values so a
// HiveScale relay can interpret the BLE and I2C OTA paths identically.
using namespace beecounter_proto;

static NimBLECharacteristic* chrOtaStatus = nullptr;
static volatile uint8_t  s_otaState = OTA_STATE_IDLE;
static volatile uint8_t  s_otaErr   = OTA_ERR_NONE;
static uint32_t s_otaSize        = 0;             // expected image size
static volatile uint32_t s_otaRecv = 0;           // bytes written so far
static uint32_t s_otaExpectedCrc = 0;             // end-to-end CRC from BEGIN
static uint32_t s_otaRunningCrc  = 0xFFFFFFFFUL;
static unsigned long s_otaRebootAt = 0;           // 0 = no reboot pending

// CRC-32 (IEEE 802.3, reflected poly 0xEDB88320) — same as zlib.crc32 on the
// backend and the I2C OTA path in main.cpp, so a value verified here matches
// end to end. Used with init/final 0xFFFFFFFF.
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return crc;
}

static void otaPublishStatus() {
    if (!chrOtaStatus) return;
    // state(1) + received(4 BE) + err(1) — same layout as REG_OTA_STATUS.
    uint8_t b[6];
    b[0] = s_otaState;
    uint32_t r = s_otaRecv;
    b[1] = (r >> 24) & 0xFF; b[2] = (r >> 16) & 0xFF;
    b[3] = (r >> 8) & 0xFF;  b[4] = r & 0xFF;
    b[5] = s_otaErr;
    chrOtaStatus->setValue(b, sizeof(b));
    chrOtaStatus->notify();
}

static void otaFail(uint8_t state) {
    if (s_otaState == OTA_STATE_RECEIVING) Update.abort();
    s_otaState = state;
    s_otaErr   = state;
    otaPublishStatus();
}

class OtaCtrlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        NimBLEAttValue v = c->getValue();
        if (v.size() < 1) return;
        const uint8_t* p = v.data();
        switch (p[0]) {
        case OTA_OP_BEGIN: {
            if (v.size() < 9) { otaFail(OTA_STATE_ERR_BEGIN); return; }
            uint32_t size = (uint32_t)p[1] | ((uint32_t)p[2] << 8) |
                            ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
            uint32_t crc  = (uint32_t)p[5] | ((uint32_t)p[6] << 8) |
                            ((uint32_t)p[7] << 16) | ((uint32_t)p[8] << 24);
            if (!Update.begin(size)) {
                Serial.printf("[OTA] Update.begin(%u) failed: %s\n",
                              (unsigned)size, Update.errorString());
                otaFail(OTA_STATE_ERR_BEGIN);
                return;
            }
            s_otaSize        = size;
            s_otaExpectedCrc = crc;
            s_otaRecv        = 0;
            s_otaRunningCrc  = 0xFFFFFFFFUL;
            s_otaErr         = OTA_ERR_NONE;
            s_otaState       = OTA_STATE_RECEIVING;
            Serial.printf("[OTA] BEGIN size=%u crc=0x%08X\n", (unsigned)size, (unsigned)crc);
            otaPublishStatus();
            break;
        }
        case OTA_OP_END: {
            if (s_otaState != OTA_STATE_RECEIVING) return;
            if (s_otaRecv != s_otaSize) {
                Serial.printf("[OTA] size mismatch recv=%u expected=%u\n",
                              (unsigned)s_otaRecv, (unsigned)s_otaSize);
                otaFail(OTA_STATE_ERR_SIZE);
                return;
            }
            uint32_t finalCrc = s_otaRunningCrc ^ 0xFFFFFFFFUL;
            if (s_otaExpectedCrc != 0 && finalCrc != s_otaExpectedCrc) {
                Serial.printf("[OTA] CRC mismatch got=0x%08X expected=0x%08X\n",
                              (unsigned)finalCrc, (unsigned)s_otaExpectedCrc);
                otaFail(OTA_STATE_ERR_CRC);
                return;
            }
            if (!Update.end(true)) {
                Serial.printf("[OTA] Update.end failed: %s\n", Update.errorString());
                s_otaState = OTA_STATE_ERR_END;
                s_otaErr   = OTA_STATE_ERR_END;
                otaPublishStatus();
                return;
            }
            Serial.println("[OTA] END ok — image verified, rebooting shortly");
            s_otaState   = OTA_STATE_DONE;
            otaPublishStatus();
            // Defer the reset so the central can read DONE; loopOta() handles it.
            s_otaRebootAt = millis() + 1500;
            break;
        }
        case OTA_OP_ABORT: {
            if (s_otaState == OTA_STATE_RECEIVING) Update.abort();
            s_otaState = OTA_STATE_IDLE;
            s_otaErr   = OTA_ERR_NONE;
            s_otaRecv  = 0;
            otaPublishStatus();
            Serial.println("[OTA] ABORT");
            break;
        }
        default:
            break;
        }
    }
};

class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        if (s_otaState != OTA_STATE_RECEIVING) { otaFail(OTA_STATE_ERR_SEQ); return; }
        NimBLEAttValue v = c->getValue();
        size_t n = v.size();
        if (n == 0) return;
        const uint8_t* p = v.data();
        // Update.write takes a non-const pointer but does not modify the buffer.
        if (Update.write(const_cast<uint8_t*>(p), n) != n) {
            Serial.printf("[OTA] write failed at %u: %s\n",
                          (unsigned)s_otaRecv, Update.errorString());
            otaFail(OTA_STATE_ERR_WRITE);
            return;
        }
        s_otaRunningCrc = crc32_update(s_otaRunningCrc, p, n);
        s_otaRecv += n;
    }
};

bool isOtaActive() {
    return s_otaState == OTA_STATE_RECEIVING ||
           (s_otaState == OTA_STATE_DONE && s_otaRebootAt != 0);
}

void loopOta() {
    if (s_otaState == OTA_STATE_DONE && s_otaRebootAt != 0 &&
        (long)(millis() - s_otaRebootAt) >= 0) {
        Serial.println("[OTA] rebooting into new image");
        Serial.flush();
        NimBLEDevice::deinit(true);
        delay(50);
        ESP.restart();
    }
}

// ===========================================================================
// Lifecycle
// ===========================================================================
void begin() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(), true);
    server->advertiseOnDisconnect(true);

    NimBLEService* svc = server->createService(SVC_BEECOUNTER);

    chrMeasurement = svc->createCharacteristic(CHR_MEASUREMENT,
                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    NimBLECharacteristic* chrCtrl = svc->createCharacteristic(CHR_CTRL,
                        NIMBLE_PROPERTY::WRITE);
    chrCtrl->setCallbacks(new CtrlCallbacks());

    // Firmware-over-BLE: control + payload (write) and a readable/notifiable status.
    NimBLECharacteristic* chrOtaCtrl = svc->createCharacteristic(CHR_OTA_CTRL,
                        NIMBLE_PROPERTY::WRITE);
    chrOtaCtrl->setCallbacks(new OtaCtrlCallbacks());
    NimBLECharacteristic* chrOtaData = svc->createCharacteristic(CHR_OTA_DATA,
                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    chrOtaData->setCallbacks(new OtaDataCallbacks());
    chrOtaStatus = svc->createCharacteristic(CHR_OTA_STATUS,
                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    otaPublishStatus();

    svc->start();

    // Seed the measurement characteristic so the first read has data.
    publish();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName(BLE_DEVICE_NAME);
    adv->addServiceUUID(svc->getUUID());
    adv->enableScanResponse(true);
    adv->start();
    Serial.println("[BLE] BeeCounter GATT server advertising (connectable)");
}

void publish() {
    if (!chrMeasurement) return;
    Telemetry t;
    getTelemetry(t);
    String json = measurementJson(t);
    if (json.length() > 512) {
        // Should never happen with the aggregate-only payload, but guard like
        // HiveInside does rather than let NimBLE reject an oversized value.
        Serial.printf("[BLE] measurement JSON too large (%u B > 512); not updated\n",
                      (unsigned)json.length());
        return;
    }
    chrMeasurement->setValue((uint8_t*)json.c_str(), json.length());
    chrMeasurement->notify();
}

}  // namespace ble

#endif  // BEECOUNTER_BLE
