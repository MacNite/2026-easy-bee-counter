// HiveHub-compatible BLE/GATT transport for HiveTraffic.
#include "ble_link.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Update.h>
#include <stdio.h>

#include "counter_protocol.h"
#include "measurement_json.h"
#include "version.h"

namespace ble {
namespace {

constexpr char BLE_DEVICE_NAME[] = "BeeCounter";
constexpr char SVC_BEECOUNTER[] = "8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_MEASUREMENT[] = "8e8b0102-7a1c-4b9e-9a2f-1d6e0b9c1a01";
// Night mode: HiveHub writes a suspension DURATION here, and reads back the
// current state. Deliberately a separate characteristic from the OTA control
// one — an image transfer and "stop sensing for a while" have nothing in
// common but a direction, and sharing an opcode space between them would mean
// a malformed OTA frame could suspend counting.
constexpr char CHR_CONTROL[] = "8e8b0103-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_CTRL[] = "8e8b0110-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_DATA[] = "8e8b0111-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr char CHR_OTA_STATUS[] = "8e8b0113-7a1c-4b9e-9a2f-1d6e0b9c1a01";
constexpr uint16_t ADV_INTERVAL_UNITS = 1600;  // 1600 * 0.625 ms = 1 second
constexpr uint8_t OTA_OP_BEGIN = 0x01;
constexpr uint8_t OTA_OP_END = 0x03;
constexpr uint8_t OTA_OP_ABORT = 0x04;
constexpr uint32_t OTA_REBOOT_DELAY_MS = 1500;
// How often, at most, a DATA write refreshes the OTA status characteristic and
// notifies subscribers.
//
// Time-based rather than byte-based (the alternative was every 4-16 KiB) because
// it bounds the notification rate no matter how fast the transfer runs: a
// byte-based threshold ties the rate to throughput, so a 517-byte-MTU link with
// a short connection interval turns a multi-megabyte image into a notify storm
// competing with the DATA writes it is reporting on, while the same threshold on
// a slow link goes nearly silent. 250 ms is at most four notifications per
// second — a smooth progress bar, and negligible next to the data traffic —
// regardless of MTU, connection interval or image size.
constexpr uint32_t OTA_PROGRESS_NOTIFY_INTERVAL_MS = 250;

using namespace beecounter_proto;

NimBLECharacteristic* otaStatus = nullptr;
volatile uint8_t otaState = OTA_STATE_IDLE;
volatile uint8_t otaError = OTA_ERR_NONE;
volatile uint32_t otaReceived = 0;
uint32_t otaSize = 0;
uint32_t otaExpectedCrc = 0;
uint32_t otaRunningCrc = 0xFFFFFFFFUL;
uint32_t otaRebootAt = 0;
uint32_t otaLastNotifyMs = 0;

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

// Build the current six-byte, little-endian status (the same layout HiveInside
// uses). Snapshotting otaReceived once matters: the four length bytes must
// describe one instant, not four.
static void buildOtaStatusValue(uint8_t (&value)[6]) {
    const uint32_t received = otaReceived;
    value[0] = otaState;
    value[1] = static_cast<uint8_t>(received);
    value[2] = static_cast<uint8_t>(received >> 8);
    value[3] = static_cast<uint8_t>(received >> 16);
    value[4] = static_cast<uint8_t>(received >> 24);
    value[5] = otaError;
}

static void publishOtaStatus() {
    if (!otaStatus) return;
    uint8_t value[6];
    buildOtaStatusValue(value);
    otaStatus->setValue(value, sizeof(value));
    otaStatus->notify();
    otaLastNotifyMs = millis();
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
    // The document itself is built by the pure serializer in
    // include/measurement_json.h, which is what test/test_measurement_json/
    // exercises on a host compiler — including the saturated worst case that
    // decides the buffer size below.
    char json[MEASUREMENT_JSON_CAPACITY];
    const int length = buildMeasurementJson(json, sizeof(json), t,
                                            HIVETRAFFIC_FW_VERSION);
    if (length <= 0) {
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

        // Throttled progress. Without this the characteristic held whatever
        // BEGIN left in it — a `received` of zero — for the entire transfer,
        // and subscribers got nothing between BEGIN and END, which made the
        // four progress bytes look live while reporting a constant.
        // Unsigned subtraction, so the millis() rollover needs no special case.
        if (millis() - otaLastNotifyMs >= OTA_PROGRESS_NOTIFY_INTERVAL_MS) {
            publishOtaStatus();
        }
    }
};

// A read must never be answered from a value last written 250 ms (or a whole
// transfer) ago: a client that polls instead of subscribing gets the byte count
// as of this instant.
class OtaStatusCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        uint8_t value[6];
        buildOtaStatusValue(value);
        characteristic->setValue(value, sizeof(value));
    }
};

// Night-mode control. The parsing is intentionally strict: a write that is not
// exactly a known opcode of the right length is ignored rather than guessed at,
// because every misreading of this characteristic costs counted bees.
class ControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const NimBLEAttValue value = characteristic->getValue();
        if (value.size() == 0) return;
        const uint8_t* data = value.data();

        switch (data[0]) {
        case CTRL_OP_SET_IDLE: {
            if (value.size() != CTRL_SET_IDLE_LENGTH) {
                Serial.printf("[BLE-CTRL] SET_IDLE ignored: %u bytes, expected %u\n",
                              (unsigned)value.size(),
                              (unsigned)CTRL_SET_IDLE_LENGTH);
                return;
            }
            // Refusing during an OTA is not caution, it is correctness: the
            // transfer already parks the emitters and pauses polling, and a
            // suspension armed underneath it would still be running when the
            // counter reboots into the new image — except it would not, because
            // the state is not persisted. Rejecting keeps the two mechanisms
            // from having an opinion about each other at all.
            if (isOtaActive()) {
                Serial.println(F("[BLE-CTRL] SET_IDLE refused: OTA in progress"));
                return;
            }
            const uint32_t duration_s =
                (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
            const uint32_t granted = applyIdleRequest(duration_s);
            if (granted != duration_s) {
                Serial.printf("[BLE-CTRL] idle %lus requested, %lus granted (cap %lus)\n",
                              (unsigned long)duration_s, (unsigned long)granted,
                              (unsigned long)MAX_IDLE_SECONDS);
            } else if (granted == 0) {
                Serial.println(F("[BLE-CTRL] idle 0s: sensing resumed"));
            } else {
                Serial.printf("[BLE-CTRL] sensing suspended for %lus\n",
                              (unsigned long)granted);
            }
            break;
        }
        case CTRL_OP_RESUME:
            if (value.size() != 1) {
                Serial.printf("[BLE-CTRL] RESUME ignored: %u bytes, expected 1\n",
                              (unsigned)value.size());
                return;
            }
            applyIdleRequest(0);
            Serial.println(F("[BLE-CTRL] sensing resumed"));
            break;
        case CTRL_OP_SET_BANKS: {
            if (value.size() != CTRL_SET_BANKS_LENGTH) {
                Serial.printf("[BLE-CTRL] SET_BANKS ignored: %u bytes, expected %u\n",
                              (unsigned)value.size(),
                              (unsigned)CTRL_SET_BANKS_LENGTH);
                return;
            }
            // Deliberately NOT refused during an OTA, where SET_IDLE is. A
            // suspension armed under a transfer would outlive the reboot it
            // cannot survive; a bank mask is a configuration HiveHub re-asserts
            // every cycle regardless, and refusing it here would only delay it
            // by one. The emitters are dark for the transfer either way.
            const uint8_t granted = applyBankMask(data[1]);
            if (granted != data[1]) {
                Serial.printf("[BLE-CTRL] banks 0x%02X requested, 0x%02X in force\n",
                              (unsigned)data[1], (unsigned)granted);
            } else {
                Serial.printf("[BLE-CTRL] emitter banks set to 0x%02X\n",
                              (unsigned)granted);
            }
            break;
        }
        default:
            Serial.printf("[BLE-CTRL] unknown opcode 0x%02X ignored\n",
                          (unsigned)data[0]);
            break;
        }
    }

    // Built on read, like the measurement document, so a polling client always
    // sees the live remaining time rather than whatever the last write left.
    void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const uint32_t remaining = idleRemainingSeconds();
        uint8_t value[CTRL_STATUS_LENGTH];
        value[0] = remaining > 0 ? CTRL_STATE_IDLE : CTRL_STATE_SENSING;
        value[1] = static_cast<uint8_t>(remaining);
        value[2] = static_cast<uint8_t>(remaining >> 8);
        value[3] = static_cast<uint8_t>(remaining >> 16);
        value[4] = static_cast<uint8_t>(remaining >> 24);
        // Appended in v5; a client that reads the first five bytes and stops
        // sees exactly the value it saw before this byte existed.
        value[5] = bankMask();
        characteristic->setValue(value, sizeof(value));
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

// NimBLE stores these pointers for the lifetime of the server and never frees
// them: NimBLECharacteristic::setCallbacks() takes no ownership at all, so the
// old `new X(), true` form both fails to compile against NimBLE 2.5.x (the
// deleteCallbacks argument is gone) and would have leaked. File-scope
// singletons outlive the server by construction — they are stateless anyway.
MeasurementCallbacks measurementCallbacks;
ControlCallbacks controlCallbacks;
OtaControlCallbacks otaControlCallbacks;
OtaDataCallbacks otaDataCallbacks;
OtaStatusCallbacks otaStatusCallbacks;
ServerCallbacks serverCallbacks;

}  // namespace

void begin() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEServer* server = NimBLEDevice::createServer();
    // false: never delete a statically allocated callback object.
    server->setCallbacks(&serverCallbacks, false);
    server->advertiseOnDisconnect(true);

    NimBLEService* service = server->createService(SVC_BEECOUNTER);
    NimBLECharacteristic* measurement = service->createCharacteristic(
        CHR_MEASUREMENT, NIMBLE_PROPERTY::READ);
    measurement->setCallbacks(&measurementCallbacks);
    refreshMeasurement(measurement);

    NimBLECharacteristic* control = service->createCharacteristic(
        CHR_CONTROL, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    control->setCallbacks(&controlCallbacks);

    NimBLECharacteristic* otaControl = service->createCharacteristic(
        CHR_OTA_CTRL, NIMBLE_PROPERTY::WRITE);
    otaControl->setCallbacks(&otaControlCallbacks);
    NimBLECharacteristic* payload = service->createCharacteristic(
        CHR_OTA_DATA, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    payload->setCallbacks(&otaDataCallbacks);
    otaStatus = service->createCharacteristic(
        CHR_OTA_STATUS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    otaStatus->setCallbacks(&otaStatusCallbacks);
    publishOtaStatus();
    // No service->start(): in NimBLE 2.5.x it is a deprecated no-op. Services
    // are registered when the GATT server starts, which advertising->start()
    // below does for us.

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
