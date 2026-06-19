# Wireless (BLE/GATT) mode

The Easy Bee Counter can talk to the HiveScale either over the **wired I2C
slave** (the default) or over a **connectable BLE GATT server**, chosen at
compile time. The BLE mode mirrors
[HiveInside's wireless transfer](../../HiveInside/firmware/src/ble_link.cpp):
the same NimBLE 2.x stack, a compact-JSON measurement characteristic, and the
same firmware-over-BLE OTA scheme.

## Why BLE is "free" here

The counter's power budget is dominated by **pulsing the 48 IR emitters**, so
keeping the BLE radio advertising-connectable adds negligible average current.
That means we skip *all* of HiveInside's deep-sleep / wake-sync machinery — the
device just keeps counting and stays connectable continuously, exactly like the
I2C slave is always available on the wire.

## Building

| Build | Env | Link |
| ----- | --- | ---- |
| Wired (default) | `esp32-c6-devkitc-1` | I2C slave @ 0x30 on GPIO2/3 |
| Wireless | `esp32-c6-devkitc-1-ble` | BLE GATT server, name `BeeCounter` |

```sh
pio run -e esp32-c6-devkitc-1-ble -t upload
```

The BLE env adds the `h2zero/NimBLE-Arduino` dependency and a dual-OTA partition
table (`partitions_4mb_ota_no_fs.csv`) so a central can push firmware over BLE.
The two builds are mutually exclusive — `-DBEECOUNTER_BLE` compiles the Wire1
slave and the OTA-over-I2C path out and the GATT server in.

## GATT layout

Custom service `8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01` (distinct from
HiveInside's `8e8b00xx-…` base so a central can tell the devices apart).

| Characteristic | UUID suffix | Props | Payload |
| -------------- | ----------- | ----- | ------- |
| Measurement | `…0102…` | READ + NOTIFY | Compact JSON (below) |
| Control | `…0103…` | WRITE | 1 byte `CMD_*` (same opcodes as I2C `REG_CTRL`) |
| OTA control | `…0110…` | WRITE | Framed BEGIN/END/ABORT |
| OTA data | `…0111…` | WRITE / WRITE_NR | Image byte stream |
| OTA status | `…0113…` | READ + NOTIFY | `state(1) + received(4 BE) + err(1)` |

### Measurement JSON

Only the aggregate figures the backend uploads, plus diagnostics — **no per-gate
arrays** (those are debug-only and stay on the I2C `REG_PER_GATE_*` registers).
This keeps the value far under the 512-byte ATT attribute cap.

```json
{
  "fw": 2,              // beecounter_proto::PROTOCOL_VERSION
  "uptime_s": 1234,
  "status": 15,         // STATUS_* bitfield (same as REG_STATUS)
  "num_gates": 24,
  "gates_healthy": 3,
  "total_in": 100,      // lifetime
  "total_out": 95,
  "interval_in": 5,     // latched (shadow) interval — see below
  "interval_out": 4,
  "glitches": 2
}
```

### Interval semantics (identical to I2C)

`interval_in` / `interval_out` report the **latched (shadow)** interval, exactly
like the I2C `REG_INTERVAL_*` registers. A central performs the same cycle it
does over I2C:

1. Read the Measurement characteristic.
2. Write `CMD_LATCH` (`0x01`) to the Control characteristic.

`CMD_LATCH` atomically copies the live counters into the shadow and zeroes the
live ones, so the next read returns a fresh interval. The firmware re-publishes
immediately after any control write, and otherwise refreshes the characteristic
every ~2 s as a heartbeat for notify subscribers.

Other control commands match the I2C protocol: `CMD_CLEAR_TOTALS` (0x02),
`CMD_CLEAR_FAULTS` (0x04), `CMD_LEDS_OFF/ON/AUTO` (0x10/0x11/0x12).

## Firmware-over-BLE (OTA)

Byte-for-byte the HiveInside scheme, so a HiveScale BLE relay can reuse its
HiveInside OTA code:

* **OTA control** opcodes (first byte): `0x01` BEGIN `+ size(4 LE) + crc32(4 LE)`,
  `0x03` END (verify + reboot), `0x04` ABORT.
* **OTA data**: the image streamed in order; the device writes each chunk to the
  inactive OTA slot and keeps a running CRC-32 (poly `0xEDB88320`, the same
  zlib-compatible CRC the I2C path and the backend use).
* Nothing is committed until END verifies the end-to-end size **and** CRC, so a
  dropped or corrupted transfer always leaves the device on its current image.
* `OTA status` reports `OTA_STATE_*` (`i2c_slave_protocol.h`), so BLE and I2C OTA
  states are interpreted identically.

> **Note on endianness:** the OTA *control* payload (size/crc) is **little-endian**
> to match HiveInside's BLE format. The OTA *status* `received` field and the
> measurement counters follow this ecosystem's I2C convention. Both choices are
> deliberate; keep them in sync with any HiveScale relay.

## HiveScale side (not yet implemented)

This change adds the **peripheral** (the bee counter). For HiveScale to consume
it wirelessly, its `bee_counter_client` needs a GATT-central backend alongside
the existing I2C one — connect to `BeeCounter`, read/parse the measurement JSON
into the existing `Snapshot`, and write `CMD_LATCH`. The scan/connect plumbing
already exists for HiveInside (`firmware/src/ble_sensor.cpp`).
