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

PlatformIO target is `esp32-c6-devkitc-1`. U5 on this PCB is a **Seeed XIAO
ESP32C6** (Seeed part 113991054), not a bare ESP32-C6-MINI-1; the devkit profile
is used because the firmware names every pin by raw GPIO number in `pins.h`
rather than by the XIAO's `D0..D10` aliases. Do not read the board's silk
numbers as GPIO numbers — they differ (silk D4 is GPIO22). USB CDC is enabled in
`platformio.ini`, so `Serial` output comes out over USB without an external UART
bridge.

The build uses `partitions_4mb_ota_no_fs.csv` — two 2 MB app slots so BLE OTA
can write the inactive one. A counter still running a pre-BLE image was flashed
with a single-app layout and **must be updated once over USB**; OTA cannot
migrate a partition table.

---

## USB IR-sensor debug console (bench bring-up)

For initial testing of the IR sensors over USB — with no HiveScale / I2C master
attached — flash the dedicated debug environment instead of the production one:

```bash
pio run -e esp32-c6-devkitc-1-irdebug -t upload
pio device monitor    # 115200 baud
```

This build is identical to production but compiles in an interactive serial
console (gated behind `-DIR_DEBUG`, so it is **never** in the normal firmware).
Press a single key in the serial monitor:

| Key | Action                                                            |
| --- | ---------------------------------------------------------------- |
| `r` | Read & print all 24 gates' inner/outer beam state once          |
| `s` | Toggle continuous streaming (~200 ms)                          |
| `1` | Force IR LEDs ON (steady)                                       |
| `0` | Force IR LEDs OFF                                               |
| `a` | IR LEDs AUTO (normal pulsed mode)                              |
| `h` | Show the command list                                          |

Each reading lists the raw MCP23017 port words plus a per-gate `BLOCK`/`clear`
line for the inner and outer sensor. The emitters are pulsed on for every read
regardless of the LED mode, so the readout is always valid. Wave a finger or a
bee through a gate and you should see that gate's `inner`/`outer` flip to
`BLOCK`.

---

## Tuning

All the knobs are at the top of `src/main.cpp`:

| Constant                  | Default | Effect                                                        |
| ------------------------- | ------- | ------------------------------------------------------------- |
| `POLL_INTERVAL_MS`        | 5       | How often the MCP23017s are polled for crossings              |
| `SENSOR_DEBOUNCE_MS`      | 3       | Minimum stable time before a sensor state change counts       |
| `GATE_PAIRING_WINDOW_MS`  | 2000    | Max time between inner/outer trip for a directional count     |
| `SENSOR_STUCK_MS`         | 30000   | After this many ms continuously blocked, fault flag is raised |
| `I2C_MASTER_HZ`           | 400000  | MCP bus clock; reduces every pulsed sample's IR-on and CPU-wait time |
| `LED_SETTLE_US`           | 250     | IR emitter settle time before each pulsed read. Lower = less power but risks reading stale "clear" levels if shorter than the real phototransistor settle time. |


If your hive has unusually long entrance tunnels or slow-moving bees, raise
`GATE_PAIRING_WINDOW_MS`. If you see noise events on `GLITCH_COUNT`, raise
`SENSOR_DEBOUNCE_MS`.

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

If any MCP shows `NOT FOUND`, check:

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
