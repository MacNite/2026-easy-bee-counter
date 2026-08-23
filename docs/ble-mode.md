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
| Control characteristic (night mode + emitter banks) | `8e8b0103-7a1c-4b9e-9a2f-1d6e0b9c1a01` |
| Properties | READ, WRITE |

The value is generated when HiveHub reads it, so it contains current lifetime
totals rather than a periodically cached snapshot:

```json
{"fw":5,"ver":"0.3.0","uptime_s":1234,"status":15,"num_gates":24,"mcps_healthy":3,"total_in":100,"total_out":95,"glitches":2,"idle_s":0,"banks":7}
```

| Field | Type | Meaning |
| --- | --- | --- |
| `fw` | uint8 | Revision of *this document's* format (`PROTOCOL_VERSION`), currently 5 |
| `ver` | string | Image version from `include/version.h`, `MAJOR.MINOR.PATCH` |
| `uptime_s` | uint32 | Seconds since boot |
| `status` | uint8 | Status bitfield, see `include/counter_protocol.h` |
| `num_gates` | uint8 | Gates wired to this counter (24) |
| `mcps_healthy` | uint8 | MCP23017 expanders currently answering, 0..3 — **not** gates |
| `total_in` / `total_out` | uint32 | Monotonic lifetime totals, saturating |
| `glitches` | uint32 | Diagnostic tally of ambiguous/aborted pairings, saturating |
| `idle_s` | uint32 | Seconds of night-mode suspension still to run; `0` while counting |
| `banks` | uint8 | Bitmask of enabled emitter banks (MOSFETs): bit 0 = gates 00..07, bit 1 = 10..17, bit 2 = 20..27. `7` unless banks have been switched off |

The field names, UUIDs, and integer types match HiveHub's
`firmware/include/bee_counter_wire.h` parser, which reads `fw` first and
accepts both this revision and the previous one. The firmware deliberately
emits only the fields in HiveHub's documented contract.

The exact bytes are produced by `include/measurement_json.h` and pinned by
`test/test_measurement_json/`, which runs on a host compiler — see
[Revision history](#revision-history) before changing any of them.

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
applies to `glitches`, which is also 32-bit and also saturates — a wrapping
diagnostic counter can make a badly unhealthy device look like it has *fewer*
glitches than it did on the previous read, and the 16-bit ceiling it used to pin
at is reachable in a day on a genuinely noisy device.

`uptime_s` is seconds since boot, and its only real job is to show that the
device restarted when you did not expect it to. It is 32-bit; the practical
limit is not the field but `millis()`, which rolls over after about 49.7 days,
so the value restarts then too. (Before protocol v3 the field was 16-bit and
clamped at 65535 — 18 h 12 min — which meant an always-on counter reported one
constant number for its entire deployment and could not do that job at all.)

`mcps_healthy` counts the MCP23017 expanders currently answering on the I2C bus,
so it ranges 0..3 while `num_gates` is 24; each healthy chip covers eight gates.
**It is not a count of working gates** — the name it carried before protocol v3,
`gates_healthy`, invited exactly that reading, and a healthy device reporting
`3` alongside `num_gates: 24` looked almost entirely broken.

It reflects live health, not what was found at boot: a chip that stops answering
drops out of the count within a few polls (and clears its `STATUS_MCP_U*_OK`
status bit), and one that recovers — or that was absent at boot and later
appears — is counted again after the firmware's periodic re-probe. Gates on an
unhealthy chip are skipped entirely rather than sampled, so they contribute
neither counts nor glitches while it is down. Any consumer treating the value as
fixed after boot is wrong regardless of which name it reads.

This is why the device keeps no interval state at all. The retired wired
protocol had interval registers a `CMD_LATCH` command consumed, which meant a
HiveScale that read but failed to latch — or latched but failed to read —
silently lost an interval. Differencing totals cannot lose anything: a read is
not a consumption.

## Power and performance choices

* Measurement JSON uses a fixed 240-byte stack buffer, without ArduinoJson or
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
* Reflective sensors require continuous sampling, so **uninterrupted** counting
  and sleep are incompatible. Night mode (below) is the deliberate exception:
  it stops counting on purpose, during hours when there is nothing to count, and
  says so in the document.

## Night mode (the control characteristic)

Sampling 24 gates costs almost nothing in CPU and almost everything in emitter
current: 48 IR LEDs in 24 series pairs behind 22 R ballast, all lit together for
the settle+read window of every 5 ms poll. That is roughly 0.5-1.0 A peak and,
at the ~35 % duty the pulsed sampler runs, an average draw an order of magnitude
above the ~18 mA the C6 and the three expanders draw between them. On an
off-grid hive it is the entire power budget.

European honey bees are diurnal. Flight requires light — they will not fly in
darkness at any temperature — and stops below roughly 10 C regardless, so the
emitters spend every night burning the largest item in the budget to count
nothing. Night mode parks them.

### What the counter knows, and what it does not

**The counter never learns what time it is.** It has no RTC, no NVS and no
network. HiveHub has all three (NTP plus a DS3231 at +/-2 ppm) and owns the
schedule; the counter is told only *how long* to stay quiet:

| Frame | Bytes |
| --- | --- |
| SET_IDLE | `0x01 + duration_s(4 LE)` |
| RESUME | `0x02` |
| SET_BANKS | `0x03 + bank_mask(1)` — see [Emitter banks](#emitter-banks-the-other-power-control) |
| Read-back | `state(1) + remaining_s(4 LE) + bank_mask(1)`, state `0x00` sensing / `0x01` idle |

A `SET_IDLE` of `0` means the same thing as `RESUME`, so HiveHub can cancel on a
connection it already has without a second opcode.

This is the whole design, and every property that matters falls out of it:

* **It expires.** A HiveHub that crashes, loses power or is carried off cannot
  leave a counter blind — the deadline runs out and sensing resumes. A stored
  20:00-06:00 schedule would stay wrong until someone walked to the hive.
* **It is bounded.** `MAX_IDLE_SECONDS` is one hour, several times HiveHub's
  default 10-minute cycle. HiveHub re-arms every cycle for as long as its night
  window lasts, so no single request ever has to cover a whole night, and the
  counter's own clock only has to be right for one cycle at a time.
* **A longer request is clamped, not refused.** Refusing would leave the
  emitters running all night because one field was too large.
* **It is never persisted.** Any reset — brownout, watchdog, OTA reboot — comes
  back counting.
* **It is refused during an OTA.** A transfer already parks the emitters and
  pauses polling; the two mechanisms have no opinion about each other.

The deadline arithmetic lives in `include/idle_state.h`, free of Arduino, and is
pinned by `test/test_idle_state/` — including the `millis()` rollover and the
"HiveHub stopped re-arming" case, neither of which is practical to reproduce on
a hive.

### Why not deep sleep

Deep sleep was the obvious implementation and is the wrong one. It buys the
residual ~18 mA on top of what parking the emitters already saves — under 10 %
of the total — and costs:

* the counter is **invisible** for 8-12 h: no measurement read, so every night
  row carries `bee_counter.ok=false`;
* **no OTA**, which contradicts the standing advice to relay firmware at night
  (see HiveHub's `docs/hivetraffic-bee-counter.md`) because a transfer costs
  counted bees;
* **no cancelling** it once entered, however wrong the schedule turns out to be;
* the lifetime totals live in RAM and would need moving into RTC memory;
* `uptime_s` resets every morning, so every day looks like an unexplained reboot
  — the one thing that field exists to reveal;
* and the ESP32-C6's deep-sleep timer runs off the internal ~150 kHz RC
  oscillator, which is strongly temperature-dependent. Over an 8-12 h sleep in a
  hive that swings 10-25 C overnight, expect several minutes of drift and up to
  ~30 minutes worst case. Re-arming a short window from HiveHub's DS3231 instead
  means nothing accumulates and the worst-case error is one cycle.

Staying awake keeps the counter readable, updatable and cancellable all night,
for a few percent of the saving. If the residual current ever does matter, deep
sleep belongs behind its own opcode, with the totals moved to `RTC_DATA_ATTR`
first.

### What a night looks like in the data

`total_in` / `total_out` are frozen for the duration, so the differenced
interval across the suspension is genuinely zero rather than missing. The
`STATUS_NIGHT_IDLE` bit (`0x80`) and the `idle_s` countdown are what separate
that from a counter whose emitter FETs have died — which produces an identical
row of zeros and is otherwise indistinguishable until someone reads a week of
totals.

## Emitter banks (the other power control)

Night mode answers *when should the counter stop?* This answers *how much of
the counter should exist at all?*

Since the 2026-08 hardware revision the 48 IR emitters are split across three
IRLB8721 MOSFETs, one per MCP23017, so each third of the entrance is
independently switchable:

| Bank | Bit | Expander | Gates |
| --- | --- | --- | --- |
| 1 | `0x01` | U2 @ 0x20 | 00..07 |
| 2 | `0x02` | U3 @ 0x21 | 10..17 |
| 3 | `0x04` | U4 @ 0x22 | 20..27 |

Measured on the 3.3 V rail with the pulsed sampler at its defaults:

| Banks enabled | Gates counted | Draw @ 3.3 V |
| --- | --- | --- |
| 1 | 8 | ~0.14 A |
| 2 | 16 | ~0.22 A |
| 3 (default) | 24 | ~0.30 A |

That is roughly 80 mA per bank on top of a ~60 mA floor — dropping one bank
saves about as much current as a quarter of a night of night mode, except it
applies around the clock. The two features compose: a counter can be running on
one bank *and* be suspended, and the numbers multiply rather than compete.

Use it when the entrance is physically narrower than 24 gates, when a hive is
being run with part of its entrance closed, or when an off-grid supply will not
carry the full board. It is a **configuration**, not a schedule — HiveHub's
dashboard exposes it as three checkboxes per device, all ticked by default.

### The rules, and why each one exists

* **All three enabled is the default and the post-reset state.** Nothing about
  this is persisted on the counter, exactly as with night mode: a brownout, a
  watchdog or an OTA reboot comes back counting all 24 gates, and HiveHub
  re-asserts the mask on its next upload cycle. The worst case is one cycle of
  drawing more current than was asked for — never a counter that boots blind on
  two thirds of its entrance because of a write it received a month ago.
* **A mask of `0` is refused, not applied.** The counter keeps whatever it had
  and says so on the serial log. Blinding a counter entirely is not a
  configuration anyone needs — a counter that should count nothing is unpaired
  — and accepting it would let one corrupted byte stop counting until someone
  walks to the hive. Same reasoning as `start == end` disabling the night
  window rather than covering the whole day.
* **Bits above bank 3 are ignored.** A four-FET board's mask arriving here must
  not conjure a bank whose GPIO does not exist. A mask of *only* phantom bits is
  therefore a zero mask, and is refused as one.
* **Gates on a dark bank are skipped, not read as "clear".** An unpowered
  QRE1113 is a bare phototransistor under a 100 k pull-up, and direct sun into a
  hive entrance can pull one low. Feeding those readings to the state machine
  would invent crossings on gates the operator deliberately switched off.
* **The expander is still read and still health-checked.** `mcps_healthy` keeps
  meaning "MCP23017s answering on the I²C bus", so a chip that dies while its
  bank is off is still visible. The read costs about half a millisecond and no
  measurable current; the emitters are what the feature is about.
* **`num_gates` keeps reporting 24.** It describes what is *wired*, which has
  not changed. Active gates are `popcount(banks) * 8`, derived by the consumer,
  so the meaning of every stored reading stays fixed.
* **It is accepted during an OTA**, where `SET_IDLE` is refused. A suspension
  armed under a transfer would outlive a reboot it cannot survive; a bank mask
  is re-asserted every cycle anyway, and the emitters are dark for the transfer
  either way.

### What a switched-off bank looks like in the data

A third of the entrance stops contributing to `total_in` / `total_out`
permanently — which is character for character what a dead emitter FET produces.
`banks` is the only thing that separates them, which is why it is emitted on
every document including the `7` of a counter nobody has reconfigured: a field
that appeared only when it was interesting would make "all banks on" and
"counter too old to say" the same absence.

On the bench the `-DIR_DEBUG` console toggles banks live with the `4`, `5` and
`6` keys (named for the schematic's `/GPIO4`, `/GPIO5`, `/GPIO6` FET rails), and
prints the mask on every readout.

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

`received` is live progress, in both of the ways a client can ask for it:

* a **read** is answered from an `onRead` callback that builds the value at that
  instant, so a polling client always sees the current byte count;
* **subscribers** are notified on every state change (BEGIN, END, ABORT, errors)
  and, during the transfer, at most once every 250 ms.

The throttle is deliberately time-based rather than every-N-kilobytes: it bounds
the notification rate to four per second whatever the MTU, connection interval
or image size, where a byte threshold ties the rate to throughput and turns a
fast link into a notify storm competing with the DATA writes it reports on.

(Until protocol v3's firmware, `publishOtaStatus()` ran only on BEGIN, END,
ABORT and errors. DATA writes advanced the counter without refreshing the
characteristic, so a read mid-transfer returned the value BEGIN left there —
normally zero — and subscribers heard nothing in between. Nothing depended on
it: the HiveHub relay gates on `STATUS = DONE` and counts the bytes it sends
itself.)

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
`hivetraffic_esp32-c6_<version>.bin`. Upload it as produced. See
`Firmware/rename_firmware.py` for the naming contract.

What each token actually does on the HiveHub side, verified against its code
rather than assumed:

* `esp32-c6` is the load-bearing one. `server/firmware.py::board_from_filename`
  reads the board out of the filename, and `resolve_release_board` **refuses**
  the upload if the declared board disagrees with it, or if the filename names a
  board that is not valid for the target. That is what stops a
  cross-architecture image from ever reaching a counter.
* `hivetraffic` is a **convenience for the dashboard's upload form**, not a
  server-side routing rule. `targetFromFilename` in
  `server/dashboard/assets/views.js` matches `/hivetraffic|beecounter/` and
  pre-selects target *HiveTraffic counter*. The backend never infers the target
  from the filename — `target` is an explicit form field — so an image uploaded
  under any name lands wherever the operator (or the API caller) set it.
* The trailing dotted token pre-fills the Version field
  (`versionFromFilename`).

Because that dashboard regex matches `beecounter` too, an image named
`beecounter_esp32-c6_<version>.bin` is *also* accepted and targeted correctly —
it is not silently mis-targeted. Both prefixes work; `hivetraffic` is the one
the build produces and the one to use.

The service has no authentication. Deployment therefore relies on BLE radio
proximity and ESP image validation; signed/authenticated firmware is recommended
before treating OTA as secure against a nearby active attacker.

---

## Revision history

`fw` versions this document's format, and nothing else. It moves only when the
set, name, meaning or range of the reported fields changes; a firmware fix that
reports the same fields bumps `ver` and leaves `fw` alone.

### v5 — current

Emitter bank enables. The three MOSFETs of the 2026-08 board can be switched
individually (see [Emitter banks](#emitter-banks-the-other-power-control)), and
the document now says which are live.

| Change | v4 | v5 |
| --- | --- | --- |
| Enabled emitter banks | — | `banks` (uint8 bitmask, `7` = all three) |
| Control opcode `0x03` | unused | `SET_BANKS`, `0x03 + bank_mask(1)` |
| Control read-back | `state(1) + remaining_s(4 LE)` | + `bank_mask(1)`, appended |

Additive in both directions, and deliberately so. A parser that skips unknown
keys reads a v5 document as a v4 one; a client that reads five bytes of the
control read-back and stops gets exactly the value it got before. The usual
deployment-order rule still applies for the same reason it did at v4: a HiveHub
that does not understand `banks` cannot tell a switched-off bank from a dead
FET, and it is HiveHub that switches them off.

A counter running v4 or earlier has no `SET_BANKS` opcode. The write is ignored
and logged as an unknown opcode, so the feature degrades to "this counter runs
all three banks" — never to an error. HiveHub gates the write on `fw >= 5`
rather than relying on that.

### v4

Night mode. The counter can be told to stop sensing for a bounded period (see
[Night mode](#night-mode-the-control-characteristic)), and the document now says
when it has.

| Change | v3 | v4 |
| --- | --- | --- |
| Suspension countdown | — | `idle_s` (uint32, seconds remaining, `0` while counting) |
| `status` bit `0x80` | unused | `STATUS_NIGHT_IDLE` |
| Control characteristic | — | `8e8b0103-…`, READ + WRITE |

The addition is purely additive: v4 documents carry every v3 key unchanged, so a
parser that skips unknown keys reads one correctly without knowing about night
mode at all. HiveHub's `bee_counter_wire.h` does exactly that, which is why the
usual deployment-order rule is a formality here rather than a hazard — but it
still applies, because a HiveHub that does not understand `idle_s` cannot tell a
suspended counter from a broken one, and it is HiveHub that decides when to
suspend.

### v3

Three contract-level defects that could not be fixed on the counter alone,
fixed together so the fleet needs one coordinated deployment rather than three.

| Change | v2 | v3 |
| --- | --- | --- |
| `uptime_s` | `uint16_t`, clamped at 65535 (18 h 12 min) | `uint32_t`, no clamp |
| `glitches` | `uint16_t`, saturating at 65535 | `uint32_t`, saturating at 4294967295 |
| MCP health field | `gates_healthy` | `mcps_healthy` (same 0..3 value) |


The rename carries no change in meaning: the field counted MCP23017 expanders
in v2 as well. What changed is that the name no longer claims otherwise —
`gates_healthy: 3` next to `num_gates: 24` read as "21 of 24 gates dead" to
every consumer that met it, including HiveHub's own API documentation and its
mock server, both of which had it wrong.

**Deployment order matters.** HiveHub's tolerant parser must be deployed before
any counter that emits `fw: 3`. A counter in the field keeps reporting `fw: 2`
until it is updated over the air, and the OTA relay reads this very
characteristic before it can update anything — so a HiveHub that understood only
v3 would strand exactly the devices needing the update, and a HiveHub that
understood only v2 goes blind the moment a counter is updated.

### v2 — the last wired revision

Numbered an OTA-over-I2C register block that no longer exists. The number was
carried forward rather than reset to 1 when the transport became BLE/GATT, so a
HiveHub that has seen both generations never sees the format revision go
backwards.

---

## Known limitations of the current wire format

These are contract-level issues that cannot be fixed on the counter alone —
each needs HiveHub's parser to change in step, so they are deliberately left
alone until both sides can be revised together.

* **The GATT service has no authentication.** There is no pairing, no
  authorization and no firmware signature check: CRC-32 is an integrity check,
  not an authenticity one, so anyone in radio range can push a validly-framed
  image — and, since v4, write a `SET_IDLE` and stop the counter for up to an
  hour, or, since v5, a `SET_BANKS` and dark two thirds of its entrance. Both
  are strictly the lesser of the two (neither is persisted, both are re-asserted
  by HiveHub within one upload cycle, and anyone able to do either could already
  replace the firmware), but they are new ways to deny counting and they are
  named here rather than left to be discovered. This is an accepted, documented risk rather than an oversight, but
  closing it needs a design decision (Secure Boot + signed images vs. a
  BLE-layer authentication handshake vs. a per-device provisioning key) *and* a
  migration story for counters already in the field, since an unauthenticated
  device cannot be given a key over an unauthenticated channel. It belongs in
  its own piece of work.
* **`partitions_4mb_ota_no_fs.csv` reserves 64 KiB for SPIFFS** despite its
  name. Reclaiming it means resizing the OTA app slots, and a deployed device
  cannot migrate a partition table over an application-only OTA — so this is
  frozen for anything already shipped.
