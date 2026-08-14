# Wireless HiveHub mode (BLE/GATT)

BLE/GATT is the counter's only transport, and this is the only build. It pairs
with the wireless **HiveTraffic counter** client in
[MacNite/HiveHub](https://github.com/MacNite/HiveHub); the wired I2C link, and
HiveHub's client for it, have both been removed.

## Build and pair

```sh
cd Firmware
pio run
pio run -t upload
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
{"fw":2,"ver":"0.1.0","uptime_s":1234,"status":15,"num_gates":24,"gates_healthy":3,"total_in":100,"total_out":95,"glitches":2}
```

The field names, UUIDs, and integer types match HiveHub's
`bee_counter_client.cpp` parser. The firmware deliberately emits only the
fields in HiveHub's documented contract.

`fw` and `ver` are **not** the same thing and neither replaces the other:

* `fw` is `beecounter_proto::PROTOCOL_VERSION` — the wire format's revision.
* `ver` is the image version from `include/version.h`, in `MAJOR.MINOR.PATCH`
  form. HiveHub compares it with `parse_version` to decide whether an OTA relay
  is worth running, and re-reads it after the counter reboots to confirm the
  update actually took. A counter that reports no `ver` (firmware older than
  this field) is never blocked from an update — there is simply nothing to
  compare against.

Bump `HIVETRAFFIC_FW_VERSION` in `Firmware/include/version.h` for every released
image, or HiveHub will refuse the relay as "not newer".

## Advertising layout

The name is carried in the **scan response**, not the advertisement:

| PDU | Contents | Bytes |
| --- | --- | --- |
| Advertisement | flags + the 128-bit service UUID | 21 / 31 |
| Scan response | complete local name `BeeCounter` | 12 / 31 |

All three elements together are 33 bytes and do not fit one legacy 31-byte
advertising PDU. NimBLE 2.x defaults scan response off and does not relocate an
overflowing name on its own, so they are split explicitly in `begin()`. Keep
them split when adding anything else to the advertisement — a counter that fails
to advertise is invisible to the measurement read *and* to the OTA relay, which
locates it by scan before connecting.

## Counting and interval semantics

`total_in` and `total_out` are monotonic lifetime counters until reboot or
32-bit saturation. HiveHub differences consecutive totals on its server to
calculate intervals. The BLE client does not latch or reset the counter after
a read, so an unavailable upload cycle does not discard traffic.

The saturation is genuine: both totals stop at `4294967295` rather than wrapping
to zero, and `status` bit `0x40` is raised when a counter reaches that value. A
wrap would be indistinguishable from a reboot to a client that only sees the
totals, and would make one interval's difference enormously negative. The same
applies to `glitches`, which pins at `65535` — a wrapping diagnostic counter can
make a badly unhealthy device look like it has *fewer* glitches than it did on
the previous read.

`gates_healthy` counts the MCP23017 expanders currently answering on the I2C
bus, so it ranges 0..3 while `num_gates` is 24; each healthy chip covers eight
gates. It reflects live health, not what was found at boot: a chip that stops
answering drops out of the count within a few polls (and clears its
`STATUS_MCP_U*_OK` status bit), and one that recovers — or that was absent at
boot and later appears — is counted again after the firmware's periodic
re-probe. Gates on an unhealthy chip are skipped entirely rather than sampled,
so they contribute neither counts nor glitches while it is down.

The 0..3 range is a known misnomer kept for compatibility with HiveHub's
existing parser; see the note on a future protocol revision below.

This is why the device keeps no interval state at all. The retired wired
protocol had interval registers a `CMD_LATCH` command consumed, which meant a
HiveScale that read but failed to latch — or latched but failed to read —
silently lost an interval. Differencing totals cannot lose anything: a read is
not a consumption.

## Power and performance choices

* Measurement JSON uses a fixed 224-byte stack buffer, without ArduinoJson or
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
`0xFFFFFFFF`). Upload PlatformIO's application-only `firmware.bin`, not a
merged factory image. `Update.begin()` rejects an image that
cannot fit the inactive partition.

Gate polling pauses and the emitters remain off during transfer. A disconnect
aborts the partial write and immediately allows counting to resume. DONE remains
readable for 1.5 seconds after verification, then the ESP32-C6 reboots. A bad
size, CRC, interrupted link, or ABORT leaves the running image bootable.

### The HiveHub relay

HiveHub implements this as the `update_beecounter` command. Its HiveInside
HTTPS-to-GATT relay is parameterised over a `gattota::Target` descriptor
(service + control/data/status UUIDs), so HiveTraffic reuses the same streaming
state machine with the `8e8b01xx` UUIDs above:

1. Upload a `beecounter` firmware release (target `beecounter`, board
   `esp32-c6`) to the backend.
2. Press **Relay to counter** in the dashboard, or
   `POST /api/v1/devices/{id}/commands/update-beecounter?slot=N`.
3. HiveHub resolves hive `N`'s paired `beecounter` MAC from its hive registry,
   streams the image straight from the HTTPS download into the DATA
   characteristic, and requires `STATUS = DONE` before reporting success.
4. The counter reboots and re-advertises; the next measurement read picks up the
   new `ver`, which is what confirms the update took.

The build already names the image so the backend can stamp the board —
`hivetraffic_esp32-c6_<version>.bin`. Upload it as produced; the leading
`hivetraffic` token is what selects the counter target (`beecounter`
server-side). See `Firmware/rename_firmware.py` for the naming contract.

The service has no authentication. Deployment therefore relies on BLE radio
proximity and ESP image validation; signed/authenticated firmware is recommended
before treating OTA as secure against a nearby active attacker.

---

## Known limitations of the current wire format

These are contract-level issues that cannot be fixed on the counter alone —
each needs HiveHub's parser to change in step, so they are deliberately left
alone until both sides can be revised together.

* **`uptime_s` is a `uint16_t` and clamps at 65535** — about 18 h 12 min. An
  always-on hive device therefore reports the same uptime for nearly its whole
  deployment, which makes the field useless for spotting unexpected resets. It
  should become a `uint32_t` (~136 years); the JSON representation has no 16-bit
  limit, so this is purely a parser change.
* **`gates_healthy` counts MCP23017s (0..3), not gates (0..24)** — see above.
  A future protocol revision should either rename it to `mcps_healthy` or report
  actual healthy gates as 0/8/16/24.
* **`glitches` is a `uint16_t` and pins at 65535.** Saturating is strictly
  better than the wrap it replaced, but a device that reaches the ceiling is
  reporting a bounded value; widening it to 32 bits needs the same coordinated
  change.
* **Filename convention.** The build produces
  `hivetraffic_esp32-c6_<version>.bin` (see `Firmware/rename_firmware.py`,
  which is authoritative). `beecounter` is the *server-side target name*, not
  the filename prefix — do not rename the artifact to match it.
