// HiveHub-compatible BLE/GATT transport for HiveTraffic.
#include "ble_link.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Update.h>
#include <stdio.h>

#include "counter_protocol.h"
#include "version.h"

namespace ble {
namespace {

constexpr char BLE_DEVICE_NAME[] = "BeeCounter";
constexpr char SVC_BEECOUNTER[] = "8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_MEASUREMENT[] = "8e8b0102-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_CTRL[] = "8e8b0110-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_DATA[] = "8e8b0111-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_STATUS[] = "8e8b0113-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr uint16_t ADV_INTERVAL_UNITS = 1600;  // 1600 * 0.625 ms = 1 second
constexpr uint8_t OTA_OP_BEGIN = 0x01;
constexpr uint8_t OTA_OP_END = 0x03;
constexpr uint8_t OTA_OP_ABORT = 0x04;
constexpr uint32_t OTA_REBOOT_DELAY_MS = 1500;

using namespace beecounter_proto;

NimBLECharacteristic* otaStatus = nullptr;
volatile uint8_t otaState = OTA_STATE_IDLE;
volatile uint8_t otaError = OTA_ERR_NONE;
volatile uint32_t otaReceived = 0;
uint32_t otaSize = 0;
uint32_t otaExpectedCrc = 0;
uint32_t otaRunningCrc = 0xFFFFFFFFUL;
uint32_t otaRebootAt = 0;

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t length) {
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint32_t mask = static_cast<uint32_t>(
                -static_cast<int32_t>(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return crc;
}

static void publishOtaStatus() {
    if (!otaStatus) return;
    const uint32_t received = otaReceived;
    // Same six-byte, little-endian status used by HiveInside.
    uint8_t value[6] = {
        otaState,
        static_cast<uint8_t>(received),
        static_cast<uint8_t>(received >> 8),
        static_cast<uint8_t>(received >> 16),
        static_cast<uint8_t>(received >> 24),
        otaError,
    };
    otaStatus->setValue(value, sizeof(value));
    otaStatus->notify();
}

static void failOta(uint8_t state) {
    if (otaState == OTA_STATE_RECEIVING) Update.abort();
    otaRebootAt = 0;
    otaState = state;
    otaError = state;
    publishOtaStatus();
}

static void refreshMeasurement(NimBLECharacteristic* characteristic) {
    Telemetry t{};
    getTelemetry(t);
    // 224 bytes, not 192: the worst-case document (all fields saturated) is
    // ~155 bytes with "ver" included, and a longer version string must not be
    // able to push it over. snprintf truncation is caught below either way.
    char json[224];
    const int length = snprintf(
        json, sizeof(json),
        "{\"fw\":%u,\"ver\":\"%s\",\"uptime_s\":%u,\"status\":%u,"
        "\"num_gates\":%u,\"gates_healthy\":%u,\"total_in\":%lu,"
        "\"total_out\":%lu,\"glitches\":%u}",
        t.protocol_version, HIVETRAFFIC_FW_VERSION, t.uptime_s, t.status_flags,
        t.num_gates, t.gates_healthy, static_cast<unsigned long>(t.total_in),
        static_cast<unsigned long>(t.total_out), t.glitch_count);
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(json)) {
        Serial.println(F("[BLE] measurement serialization failed"));
        return;
    }
    characteristic->setValue(reinterpret_cast<const uint8_t*>(json), length);
}

class MeasurementCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        // Build the snapshot on demand: no periodic JSON allocation or stale read.
        refreshMeasurement(characteristic);
    }
};

class OtaControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const NimBLEAttValue value = characteristic->getValue();
        if (value.size() == 0) return;
        const uint8_t* data = value.data();

        switch (data[0]) {
        case OTA_OP_BEGIN: {
            if (value.size() != 9) {
                failOta(OTA_STATE_ERR_BEGIN);
                return;
            }
            if (otaState == OTA_STATE_RECEIVING) Update.abort();
            otaSize = static_cast<uint32_t>(data[1]) |
                      (static_cast<uint32_t>(data[2]) << 8) |
                      (static_cast<uint32_t>(data[3]) << 16) |
                      (static_cast<uint32_t>(data[4]) << 24);
            otaExpectedCrc = static_cast<uint32_t>(data[5]) |
                             (static_cast<uint32_t>(data[6]) << 8) |
                             (static_cast<uint32_t>(data[7]) << 16) |
                             (static_cast<uint32_t>(data[8]) << 24);
            otaReceived = 0;
            otaRunningCrc = 0xFFFFFFFFUL;
            otaRebootAt = 0;
            if (otaSize == 0 || !Update.begin(otaSize)) {
                Serial.printf("[BLE-OTA] BEGIN rejected: size=%lu error=%s\n",
                              static_cast<unsigned long>(otaSize),
                              Update.errorString());
                failOta(OTA_STATE_ERR_BEGIN);
                return;
            }
            otaError = OTA_ERR_NONE;
            otaState = OTA_STATE_RECEIVING;
            publishOtaStatus();
            Serial.printf("[BLE-OTA] receiving %lu bytes, crc=0x%08lX\n",
                          static_cast<unsigned long>(otaSize),
                          static_cast<unsigned long>(otaExpectedCrc));
            break;
        }
        case OTA_OP_END: {
            if (otaState != OTA_STATE_RECEIVING) {
                failOta(OTA_STATE_ERR_SEQ);
                return;
            }
            if (otaReceived != otaSize) {
                failOta(OTA_STATE_ERR_SIZE);
                return;
            }
            if ((otaRunningCrc ^ 0xFFFFFFFFUL) != otaExpectedCrc) {
                failOta(OTA_STATE_ERR_CRC);
                return;
            }
            if (!Update.end(true)) {
                failOta(OTA_STATE_ERR_END);
                return;
            }
            otaState = OTA_STATE_DONE;
            otaError = OTA_ERR_NONE;
            otaRebootAt = millis() + OTA_REBOOT_DELAY_MS;
            publishOtaStatus();
            Serial.println(F("[BLE-OTA] verified; reboot scheduled"));
            break;
        }
        case OTA_OP_ABORT:
            if (otaState == OTA_STATE_RECEIVING) Update.abort();
            otaState = OTA_STATE_IDLE;
            otaError = OTA_ERR_NONE;
            otaReceived = 0;
            otaRebootAt = 0;
            publishOtaStatus();
            break;
        default:
            failOta(OTA_STATE_ERR_SEQ);
            break;
        }
    }
};

class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const NimBLEAttValue value = characteristic->getValue();
        const size_t length = value.size();
        if (length == 0) return;
        if (otaState != OTA_STATE_RECEIVING || otaReceived > otaSize ||
            length > otaSize - otaReceived) {
            failOta(OTA_STATE_ERR_SEQ);
            return;
        }
        const uint8_t* data = value.data();
        if (Update.write(const_cast<uint8_t*>(data), length) != length) {
            failOta(OTA_STATE_ERR_WRITE);
            return;
        }
        otaRunningCrc = crc32Update(otaRunningCrc, data, length);
        otaReceived += length;
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        Serial.println(F("[BLE] HiveHub connected"));
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
        // A vanished relay must not leave gate sampling paused indefinitely.
        // The inactive partition is disposable; the running slot is untouched.
        if (otaState == OTA_STATE_RECEIVING) {
            failOta(OTA_STATE_ERR_SEQ);
            Serial.println(F("[BLE-OTA] transfer aborted on disconnect"));
        }
        Serial.printf("[BLE] disconnected (reason %d)\n", reason);
    }
};

}  // namespace

void begin() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(), true);
    server->advertiseOnDisconnect(true);

    NimBLEService* service = server->createService(SVC_BEECOUNTER);
    NimBLECharacteristic* measurement = service->createCharacteristic(
        CHR_MEASUREMENT, NIMBLE_PROPERTY::READ);
    measurement->setCallbacks(new MeasurementCallbacks(), true);
    refreshMeasurement(measurement);

    NimBLECharacteristic* control = service->createCharacteristic(
        CHR_OTA_CTRL, NIMBLE_PROPERTY::WRITE);
    control->setCallbacks(new OtaControlCallbacks(), true);
    NimBLECharacteristic* payload = service->createCharacteristic(
        CHR_OTA_DATA, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    payload->setCallbacks(new OtaDataCallbacks(), true);
    otaStatus = service->createCharacteristic(
        CHR_OTA_STATUS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    publishOtaStatus();
    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    // The name goes in the SCAN RESPONSE, not the advertisement. A legacy
    // advertising PDU holds 31 bytes, and the two elements HiveHub cares about
    // do not both fit alongside a name:
    //
    //     flags                 3   (added by NimBLE at start())
    //     128-bit service UUID  18  (2 + 16)
    //     "BeeCounter"          12  (2 + 10)   -> 33 > 31
    //
    // NimBLE 2.x leaves scan response DISABLED by default and does not silently
    // relocate the name, so setting all three on the advertisement overflows and
    // something is dropped — potentially advertising itself. A counter that does
    // not advertise is invisible to BOTH the measurement read and the OTA relay,
    // which locates it by a scan first (HiveHub ble_sensor.cpp::otaBegin).
    // Splitting them keeps the advertisement at 21 bytes and the scan response
    // at 12, with room to spare on each.
    advertising->addServiceUUID(service->getUUID());
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(BLE_DEVICE_NAME);
    advertising->setScanResponseData(scanResponse);
    advertising->enableScanResponse(true);
    advertising->setMinInterval(ADV_INTERVAL_UNITS);
    advertising->setMaxInterval(ADV_INTERVAL_UNITS);
    if (!advertising->start()) {
        // Never fail silently: without this the only symptom is a counter that
        // HiveHub can never see, with nothing on the serial log to say why.
        Serial.println(F("[BLE] ERROR: advertising failed to start"));
        return;
    }
    Serial.printf("[BLE] HiveTraffic %s advertising for HiveHub\n",
                  HIVETRAFFIC_FW_VERSION);
}

bool isOtaActive() {
    return otaState == OTA_STATE_RECEIVING || otaRebootAt != 0;
}

void loopOta() {
    if (otaRebootAt != 0 &&
        static_cast<int32_t>(millis() - otaRebootAt) >= 0) {
        Serial.println(F("[BLE-OTA] rebooting into updated firmware"));
        Serial.flush();
        delay(50);
        ESP.restart();
    }
}

}  // namespace ble
