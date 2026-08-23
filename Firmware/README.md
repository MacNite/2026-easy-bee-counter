# HiveTraffic bee counter — ESP32-C6 mini firmware

Firmware for the ESP32-C6 mini bee counter PCB. Continuously polls the 24
entrance gates, detects directional bee crossings, and serves lifetime totals
to [MacNite/HiveHub](https://github.com/MacNite/HiveHub) over BLE/GATT.

There is one build, and it is wireless. Flash it with:

```sh
pio run -t upload
```

The firmware also exposes HiveInside-style connectable BLE OTA characteristics,
which HiveHub drives with its `update_beecounter` command. Image size and CRC-32
are verified before the inactive app slot is selected; see
[`docs/ble-mode.md`](../docs/ble-mode.md) for the framing and the relay.

The image version lives in [`include/version.h`](include/version.h) and is
reported over BLE as the measurement JSON's `ver` field. **Bump it for every
released image** — HiveHub gates an OTA relay on it being newer than what the
counter reports, and uses it afterwards to confirm the update took.

> **The wired HiveScale link has been removed.** Earlier firmware ran the C6's
> second I2C controller as a permanent slave at 0x30, serving a register map
> (totals, `CMD_LATCH` interval counters, per-gate arrays, OTA-over-I2C) to a
> HiveScale polling over J1. HiveHub reads counters over BLE only and dropped
> its wired client, so nothing spoke that protocol anymore. The connector and
> traces are still on the PCB; the firmware simply never brings that controller
> up.

---

## How it works

### Sensing

Each of the 24 gates has two QRE1113 reflective IR sensors:

- **Inner** — toward the hive interior
- **Outer** — toward the outside world

The 48 sensor lines are read through 3 MCP23017 I2C port expanders:

| MCP   | I2C addr | Gates       | Inner sensors  | Outer sensors  |
| ----- | -------- | ----------- | -------------- | -------------- |
| U2    | 0x20     | GATE_00..07 | GPA0..GPA7     | GPB0..GPB7     |
| U3    | 0x21     | GATE_10..17 | GPA0..GPA7     | GPB0..GPB7     |
| U4    | 0x22     | GATE_20..27 | GPA0..GPA7     | GPB0..GPB7     |

The IR emitters are split into three banks driven by IRLB8721 N-channel MOSFETs — one FET per MCP23017 since the 2026-08 hardware revision:

| Bank | Gates | FET | Enable GPIO | XIAO silk | Schematic net |
| ---- | ----- | --- | ----------- | --------- | ------------- |
| LED_BANK_1 | GATE_00..07 (U2) | Q1 | GPIO19 | D8  | /GPIO4 |
| LED_BANK_2 | GATE_10..17 (U3) | Q2 | GPIO20 | D9  | /GPIO5 |
| LED_BANK_3 | GATE_20..27 (U4) | Q3 | GPIO18 | D10 | /GPIO6 |

The net names `/GPIO4`, `/GPIO5` and `/GPIO6` are labels only — they are **not** physical GPIO4/5/6. Use the Enable GPIO column.

The previous 2-FET build split U3's gates across banks 1 and 2 (00..13 / 14..27); banks now line up 1:1 with the expanders. `pins::IR_LED_BANK_EN[]` maps bank number − 1 to its GPIO, and `gates::TABLE[i].led_bank` gives each gate's bank.

Driving the GPIO HIGH turns the bank's emitters on. In the default `LedMode::AUTO` the emitters are **pulsed**: all three banks are lit together only for the settle + MCP-read window of each poll (~1.75 ms at 100 kHz), then switched off until the next poll. This drops the emitter duty cycle from 100% to roughly 35% at  the default 5 ms poll interval, cutting average emitter current proportionally, with no change to detection behaviour. The IR_DEBUG console's `1` / `0` / `a` keys force steady-on, blackout and pulsed mode respectively for bench work.

Each bank can also be **switched off entirely**, which is a separate control from the LED mode: HiveHub writes an enable bitmask to the control characteristic (`SET_BANKS`, protocol v5) and `include/bank_state.h` decides what is applied. One bank draws ~0.14 A at 3.3 V, two ~0.22 A, three ~0.30 A, so this is the coarsest and most effective power knob on the board — for an entrance narrower than 24 gates, or a supply that will not carry the full one.

All three banks are enabled by default and after any reset; the mask is never persisted, and HiveHub re-asserts it every upload cycle. A mask of `0` is refused rather than applied. Gates on a dark bank are **skipped**, not read as "clear": an unpowered QRE1113 is a bare phototransistor under a 100 kΩ pull-up, and direct sun into the entrance can pull one low. The expander itself is still read and health-checked, so `mcps_healthy` keeps its meaning. The IR_DEBUG console's `4` / `5` / `6` keys toggle banks 1/2/3 for bench work.

### Counting

Each gate is a small state machine:

```
   IDLE ──Inner blocked──▶ INNER_FIRST ──Outer blocked──▶  count OUT
        ──Outer blocked──▶ OUTER_FIRST ──Inner blocked──▶  count IN
                          (within 2 s, else timeout)
```

After a count, the gate waits in `PAIRED` until both sensors are clear,
then returns to `IDLE`. Glitches (both blocked simultaneously, or only one
ever blocks before timeout) are counted and reported as the JSON `glitches` field for diagnostics.

### One I2C bus

The MCP23017s sit on `Wire` (GPIO22 / silk D4 = SDA, GPIO23 / silk D5 = SCL), with the on-board 4.7 kΩ
pull-ups R4/R5. The loop polls them on a ~5 ms cadence and nothing else shares
the bus, so there is no arbitration to think about.

The PCB also routes a second bus to J1 on silk D2/D3 (GPIO2/GPIO21) for the retired HiveScale
link. The firmware never initialises that controller, so those pins stay inputs.

### Reporting

Counting produces **lifetime totals only**. HiveHub connects once per upload
cycle, reads one JSON characteristic and disconnects; it derives each interval by
differencing consecutive reads on its server. That is why there is no latch, no
reset and no per-interval state on the device — a missed connection cannot lose
traffic, because nothing is ever consumed by being read.

See [`docs/ble-mode.md`](../docs/ble-mode.md) for the GATT contract, the JSON
fields and the OTA framing, and `include/counter_protocol.h` for the status
bitfield and OTA state codes shared between `main.cpp` and `ble_link.cpp`.

---

## Building & flashing

```bash
pio run               # build
pio run -t upload     # flash via USB
pio device monitor    # 115200 baud serial output
```

PlatformIO target is `seeed_xiao_esp32c6`. U5 on this PCB is a **Seeed XIAO
ESP32C6** (Seeed part 113991054), not a bare ESP32-C6-MINI-1, so this is the
board profile that matches the hardware: it declares the module's 4 MB flash
(the earlier `esp32-c6-devkitc-1` profile declared 8 MB) and pulls in the
`XIAO_ESP32C6` variant. That variant defines the `D0..D10` aliases but does
*not* enable Arduino pin remapping, so the raw GPIO numbers in `pins.h` are
still correct as written. Do not read the board's silk numbers as GPIO numbers —
they differ (silk D4 is GPIO22). USB CDC is enabled in `platformio.ini`, so
`Serial` output comes out over USB without an external UART bridge.

The build uses `partitions_4mb_ota_no_fs.csv` — two 2 MB app slots so BLE OTA
can write the inactive one. The table sums to exactly 4 MB, which is the flash
the XIAO board profile declares. A counter still running a pre-BLE image was
flashed with a single-app layout and **must be updated once over USB**; OTA
cannot migrate a partition table.

### Build artifacts

[`rename_firmware.py`](rename_firmware.py) (wired in as a `pre:` extra script)
names the products after the board and the version from `include/version.h`
instead of the generic `firmware.bin`:

| Environment          | Artifact in `.pio/build/<env>/`      |
| -------------------- | ------------------------------------ |
| `seeed_xiao_esp32c6` | `hivetraffic_esp32-c6_<version>.bin` |

That name is what makes an image self-describing to HiveHub. Drop the `.bin`
into the dashboard's **Upload firmware** form and it reads the filename to
pre-select target *HiveTraffic counter* and pre-fill the Version field — the
board is fixed at `esp32-c6`, and the backend refuses to publish a release whose
board disagrees with its filename, so a cross-architecture image can never be
relayed to a counter. Upload this file **as-is**: it is the application-only
image, which is what the counter's OTA writes into its inactive app slot, not a
merged factory image.

Because the naming is a contract with HiveHub, the board token and layout have
to stay in sync with HiveHub's `rename_firmware.py`, `server/firmware.py`
(`board_from_filename`, `BEECOUNTER_BOARDS`) and the dashboard's filename
parsing. The header comment in `rename_firmware.py` lists them.

Should a variant build ever be needed again, stamp it after the version rather
than into the board token: `0.1.0-something` is what shows up in the pre-filled
Version field, so it is hard to publish by mistake, and HiveHub compares
versions component-wise with non-digits stripped — such a build ties with the
production image of the same version and can never be relayed over it.
`rename_firmware.py` already falls back to `-<env name>` for any environment it
does not know.

---

## USB IR-sensor debug console (bench bring-up)

For initial testing of the IR sensors over USB — with no HiveScale / I2C master
attached — there is an interactive serial console in `main.cpp`. It has no
environment of its own; compile it into the production build on demand:

```bash
PLATFORMIO_BUILD_FLAGS=-DIR_DEBUG pio run -t upload
pio device monitor    # 115200 baud
```

That build is identical to production but compiles in the console (gated behind
`-DIR_DEBUG`, so it is **never** in the normal firmware). Note that the artifact
keeps the production filename, so do not upload one of these to HiveHub.
Press a single key in the serial monitor:

| Key | Action                                                            |
| --- | ---------------------------------------------------------------- |
| `r` | Read & print all 24 gates' inner/outer beam state once          |
| `s` | Toggle continuous streaming (~200 ms)                          |
| `1` | Force IR LEDs ON (steady)                                       |
| `0` | Force IR LEDs OFF                                               |
| `a` | IR LEDs AUTO (normal pulsed mode)                              |
| `n` | Arm / clear a 60 s night-mode suspension (press again to resume) |
| `4` | Toggle emitter bank 1 (GATE_00..07)                            |
| `5` | Toggle emitter bank 2 (GATE_10..17)                            |
| `6` | Toggle emitter bank 3 (GATE_20..27)                            |
| `h` | Show the command list                                          |

The bank keys are `4`/`5`/`6` because the schematic labels those rails `/GPIO4`,
`/GPIO5` and `/GPIO6` — misleading net names (they are physically GPIO19/20/18)
but the ones silkscreened next to the FETs.

Each reading lists the raw MCP23017 port words, the current bank mask, plus a
per-gate `BLOCK`/`clear` line for the inner and outer sensor. A gate whose bank
is switched off prints `<bank disabled>` rather than a beam state. The emitters are pulsed on for every read
regardless of the LED mode, so the readout is always valid. Wave a finger or a
bee through a gate and you should see that gate's `inner`/`outer` flip to
`BLOCK`.

---

## Tuning

All the knobs are at the top of `src/main.cpp`:

| Constant                  | Default | Effect                                                        |
| ------------------------- | ------- | ------------------------------------------------------------- |
| `POLL_INTERVAL_MS`        | 5       | How often the MCP23017s are polled for crossings              |
| `SENSOR_DEBOUNCE_MS`      | 5       | Minimum time a new sensor value must persist before it is accepted |
| `SENSOR_DEBOUNCE_SAMPLES` | 2       | Consecutive polls that must agree before a change is accepted |
| `GATE_PAIRING_WINDOW_MS`  | 2000    | Max time between inner/outer trip for a directional count     |
| `SENSOR_STUCK_MS`         | 30000   | After this many ms continuously blocked, fault flag is raised |
| `MCP_FAIL_THRESHOLD`      | 3       | Consecutive failed reads before a chip is declared unhealthy  |
| `MCP_RETRY_INTERVAL_MS`   | 5000    | How often an unhealthy chip is re-probed                      |
| `I2C_MASTER_HZ`           | 400000  | MCP bus clock; reduces every pulsed sample's IR-on and CPU-wait time |
| `LED_SETTLE_US`           | 250     | IR emitter settle time before each pulsed read. Lower = less power but risks reading stale "clear" levels if shorter than the real phototransistor settle time. |

If your hive has unusually long entrance tunnels or slow-moving bees, raise
`GATE_PAIRING_WINDOW_MS`.

### The debounce rule

A sensor change is accepted only once the new value has been seen on
`SENSOR_DEBOUNCE_SAMPLES` **consecutive** polls *and* has persisted for at least
`SENSOR_DEBOUNCE_MS`. Both halves matter, and they interact with
`POLL_INTERVAL_MS`:

- A `SENSOR_DEBOUNCE_MS` shorter than one poll period cannot reject anything —
  the first differing sample already satisfies it. At the 5 ms poll rate the
  sample count is the half that actually does the work.
- `SENSOR_DEBOUNCE_SAMPLES = 2` is the practical ceiling. A bee crossing at
  ~25 cm/s dwells ~12 ms in a 3 mm beam, i.e. 2-3 samples; requiring three
  agreeing samples (~10 ms) would start dropping real crossings. Two rejects a
  single corrupted or noisy word while accepting a genuine block ~5 ms after it
  appears.

So if you see noise on `glitches`, do **not** simply raise
`SENSOR_DEBOUNCE_SAMPLES` — poll faster first (lower `POLL_INTERVAL_MS`, at the
cost of emitter duty cycle) so a longer debounce still fits inside the dwell.

The rules live in `include/gate_logic.h` and are covered by the host-side tests
described under [Tests](#tests).

### Expander health

Each MCP23017 is tracked at runtime rather than only at boot:

- A chip that fails `MCP_FAIL_THRESHOLD` consecutive reads is marked unhealthy,
  its `STATUS_MCP_U*_OK` status bit is cleared, and its eight gates are reset
  and skipped. Gates are skipped from the *first* failed read — a failed I2C
  transaction reads as `0x0000`, which is indistinguishable from "all 16 beams
  blocked", so it must never reach the state machine.
- An unhealthy chip (including one missing at boot) is re-probed every
  `MCP_RETRY_INTERVAL_MS`, with the emitters dark so a probe never lengthens the
  LED pulse. On recovery its gates are reset and its status bit is set again.
- `mcps_healthy` in the BLE telemetry reflects this live state — it counts
  expanders (0..3), not gates. It was called `gates_healthy` before protocol v3.

---

## Tests

Two pure headers carry logic that is worth testing without hardware, so both are
deliberately free of Arduino, I2C and NimBLE dependencies and both run on a host
compiler:

```
./test/run_tests.sh
```

No ESP32, no MCP23017 and no bee required.

* **`include/gate_logic.h`** — sensor debounce, the per-gate direction state
  machine and the saturating counters. The suite covers single-sample spikes,
  alternating noise, simultaneous transitions, minimum-dwell and repeated
  crossings, pairing-window expiry, stuck sensors, `millis()` rollover and
  counter saturation. Run it before pushing a change to the timing rules: the
  failure it was written to catch (a debounce that accepted the first differing
  sample) is invisible in a code read and effectively untestable on a live hive.
* **`include/measurement_json.h`** — the exact bytes of the BLE measurement
  characteristic, which is HiveHub's half of a two-repo contract. The suite
  pins the field names, checks that `uptime_s` and `glitches` really do carry
  values past their old 16-bit ceilings, and measures the saturated worst-case
  document against the buffer it has to fit in. Run it before changing any
  reported field — and bump `PROTOCOL_VERSION` and update HiveHub's parser in
  the same revision when you do (see `docs/ble-mode.md`).
* **`include/idle_state.h`** — the night-mode suspension deadline. The suite
  covers clamping an over-long request, the `millis()` rollover, and the case
  that matters most in the field: HiveHub stopping re-arming, after which the
  counter has to free itself. A counter stuck suspended emits perfectly
  well-formed documents full of zeros, which is indistinguishable from a spell
  of bad weather until someone reads a week of totals — so none of this is
  something a bench session would catch.
* **`include/bank_state.h`** — the emitter-bank enable mask. Every mistake it
  can make is silent: a bank that should be on but is off produces a
  permanently flat third of the totals, which reads exactly like a dead FET,
  and a bank that should be off but is on quietly costs ~80 mA on a supply
  sized without it. The suite pins the refusal of an all-off mask, the
  one-based bank numbering that `gates::TABLE[].led_bank` depends on, and the
  masking of bits above the last physical bank.

Everything hardware-facing stays in `src/main.cpp` and is still verified on the
bench with the IR-sensor console above.

---

## Wiring sanity check

When you power the board for the first time you should see, on the serial
monitor:

```
==============================================
Easy Bee Counter 2026 — firmware booting (BLE/GATT link)
==============================================
[MCP] U2 (gates 00..07) @ 0x20: OK
[MCP] U3 (gates 10..17) @ 0x21: OK
[MCP] U4 (gates 20..27) @ 0x22: OK
[BLE] HiveTraffic 0.1.0 advertising for HiveHub
[SETUP] Entering normal counting loop (pulsed IR)
```

A chip that shows `NOT FOUND` is not written off: the firmware re-probes it
every `MCP_RETRY_INTERVAL_MS` and logs `recovered` if it starts answering, so a
marginal connection can come back without a power cycle. Its eight gates are
skipped (not counted as blocked) while it is down. If it never recovers, check:

- That the A0/A1/A2 strap pins of that chip are connected as the schematic
  says (U2=0x20: all GND; U3=0x21: A0→3V3; U4=0x22: A1→3V3).
- That the 4.7 k pull-ups R4/R5 are populated.
- That the chip's VDD pin 9 is at 3.3 V and VSS pin 10 is at GND.

After boot, breaking a gate's beam (e.g. with a piece of paper) should
trigger a serial counter increase on the next 30 s status dump.

The final boot line now reads:

```
[SETUP] Entering normal counting loop (pulsed IR)
```
