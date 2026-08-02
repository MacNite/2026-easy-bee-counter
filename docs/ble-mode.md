# Wireless HiveHub mode (BLE/GATT)

The `esp32-c6-devkitc-1-ble` build is compatible with the wireless
**HiveTraffic counter** client in [MacNite/HiveHub](https://github.com/MacNite/HiveHub).
HiveHub's current bee-counter transport is BLE-only; its former wired I2C
client has been removed.

## Build and pair

```sh
cd Firmware
pio run -e esp32-c6-devkitc-1-ble
pio run -e esp32-c6-devkitc-1-ble -t upload
```

Build HiveHub with `ENABLE_WIRELESS_BEECOUNTER=1`, then pair the counter's BLE
MAC as a **HiveTraffic counter** in HiveHub's provisioning portal. The device
advertises as `BeeCounter`, but HiveHub connects by the paired MAC.

## HiveHub GATT contract

| Item | Value |
| --- | --- |
| Service | `8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01` |
| Measurement characteristic | `8e8b0102-7a1c-4b9e-9a2f-1d6e0b9c1a01` |
| Properties | READ |

The value is generated when HiveHub reads it, so it contains current lifetime
totals rather than a periodically cached snapshot:

```json
{"fw":2,"uptime_s":1234,"status":15,"num_gates":24,"gates_healthy":3,"total_in":100,"total_out":95,"glitches":2}
```

The field names, UUIDs, and integer types match HiveHub's
`bee_counter_client.cpp` parser. The firmware deliberately emits only the
fields in HiveHub's documented contract.

## Counting and interval semantics

`total_in` and `total_out` are monotonic lifetime counters until reboot or
32-bit saturation. HiveHub differences consecutive totals on its server to
calculate intervals. The BLE client does not latch or reset the counter after
a read, so an unavailable upload cycle does not discard traffic.

This differs from legacy wired I2C mode, which has interval registers and
`CMD_LATCH`. Use BLE with current HiveHub; the default I2C build remains only
for older or custom HiveScale firmware.

## Power and performance choices

* Measurement JSON uses a fixed 192-byte stack buffer, without ArduinoJson or
  `String` heap churn.
* Telemetry is serialized only on a GATT read, not every two seconds.
* The measurement path remains read-only; OTA uses three separate
  HiveInside-compatible characteristics and does no periodic work when idle.
* Advertising runs once per second. HiveHub connects by configured MAC roughly
  once per upload cycle, so fast advertising only adds radio wakeups. A
  connection can take about one second longer in the worst case.
* The MCP23017 bus runs at 400 kHz, reducing CPU wait and the time all 48 IR
  emitters remain powered during every sample. The 5 ms cadence is unchanged.

The IR emitters still dominate power use. Do not increase the poll period
without validating missed-crossing rates on assembled entrance hardware.

## Compatibility limits

* HiveTraffic implements the OTA **peripheral**, but upstream HiveHub currently
  has no `update_beecounter` GATT relay. Until that relay is added, use a BLE
  central implementing the protocol below or update locally over USB.
* Lifetime totals are held in RAM. HiveHub recognizes a backwards total after
  reboot as a reset, but traffic before the first successful post-reboot read
  cannot be reconstructed.
* Reflective sensors require continuous sampling, so deep sleep is incompatible
  with uninterrupted counting.

## Firmware update over connectable BLE

HiveTraffic advertises connectably at all times for measurements and updates.
Its framing matches HiveInside, with a HiveTraffic UUID base, so a HiveHub relay
can reuse the HiveInside streaming state machine.

| Characteristic | UUID | Properties |
| --- | --- | --- |
| OTA control | `8e8b0110-7a1c-4b9e-9a2f-1d6e0b9c1a01` | WRITE |
| OTA data | `8e8b0111-7a1c-4b9e-9a2f-1d6e0b9c1a01` | WRITE, WRITE_NR |
| OTA status | `8e8b0113-7a1c-4b9e-9a2f-1d6e0b9c1a01` | READ, NOTIFY |

### Frames

* BEGIN: `0x01 + image_size(4 LE) + crc32(4 LE)`.
* DATA: raw application-image bytes in order. Writes with response provide
  flash-level flow control.
* END: `0x03`; accepted only after exact size and CRC verification.
* ABORT: `0x04`; discards the partial inactive-slot write.
* STATUS: `state(1) + received(4 LE) + error(1)`. States are idle `0x00`,
  receiving `0x01`, done `0x02`, or errors `0x10` through `0x15`.

CRC is standard IEEE/zlib CRC-32 (`0xEDB88320`, initial/final XOR
`0xFFFFFFFF`). Upload PlatformIO's application-only `firmware.bin` from the BLE
environment, not a merged factory image. `Update.begin()` rejects an image that
cannot fit the inactive partition.

Gate polling pauses and the emitters remain off during transfer. A disconnect
aborts the partial write and immediately allows counting to resume. DONE remains
readable for 1.5 seconds after verification, then the ESP32-C6 reboots. A bad
size, CRC, interrupted link, or ABORT leaves the running image bootable.

### HiveHub relay work still required

Upstream HiveHub's current HiveInside relay already performs HTTPS-to-GATT
streaming. Adding HiveTraffic support there requires:

1. Accept a `beecounter` release/command, routed to BLE rather than old I2C.
2. Resolve the selected hive's paired `beecounter` MAC.
3. Reuse the relay with the HiveTraffic service and `8e8b01xx` UUIDs above.
4. Stream the ESP32-C6 application image and backend CRC without buffering it.
5. Require STATUS=DONE, then reconnect and confirm a healthy measurement after
   the counter reboots.

The service has no authentication. Deployment therefore relies on BLE radio
proximity and ESP image validation; signed/authenticated firmware is recommended
before treating OTA as secure against a nearby active attacker.
