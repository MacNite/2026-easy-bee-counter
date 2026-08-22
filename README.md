# Bee Counter Redesign — Full Specification
### Single PCB + 3D Printed Housing, reported over BLE to HiveHub

A redesign of the [2019-easy-bee-counter](https://github.com/hydronics2/2019-easy-bee-counter)
by hydronics2: 24 entrance gates, 48 reflective IR sensors read through I²C
port expanders, and an always-on ESP32-C6 doing the counting and reporting
totals over BLE.

> **Status:** the PCB (`PCB_files/easy-bee-counter-2026.kicad_*`) and the
> ESP32-C6 firmware (`Firmware/`) are implemented. The 3D-printed housing
> (Section 5) is the remaining mechanical work.

---

## 1. System Overview

The **ESP32-C6 mini** runs always-on as a dedicated bee counter and advertises
as a connectable BLE peripheral. A HiveHub wakes roughly every 10 minutes,
connects, reads one JSON characteristic of lifetime totals and disconnects,
combining the counts with weight/timestamp data before logging and transmitting.

```
[HiveHub ESP32] ←——— BLE/GATT ———→ [ESP32-C6 mini]
  wakes every ~10 min                always-on, ~15 mA
  RTC, SD, WiFi, scale               bee counting, 3× MCP23017 (I²C)
```

The counter's own I²C bus (`Wire`, silk D4 = GPIO22 SDA / silk D5 = GPIO23 SCL) is used only to reach
the 3× MCP23017 port expanders.

> **The wired HiveScale link is gone.** Earlier revisions ran the C6's second
> I²C controller as a permanent slave at 0x30, serving a register map to a
> HiveScale polling over J1 — and the firmware for it has been removed, because
> HiveHub reads counters over BLE only and dropped its wired client. Sections
> below still describe the J1 connector and its cable, since both are physically
> on the PCB; no firmware brings that bus up, so populating them is optional.

---

## 2. Complete Bill of Materials

### 2.1 Microcontroller

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 1 | Seeed XIAO ESP32C6 (Seeed 113991054) | 1 | Always-on bee counter MCU. ~15 mA active, 3.3 V logic, USB-C. Two hardware I²C controllers. **Silk D0..D10 ≠ GPIO numbers** — see `Firmware/include/pins.h` |

### 2.2 I/O Expansion (replaces all 6× 74HC165)

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 2 | MCP23017-E/SP (DIP-28) | 3 | 16 inputs each = 48 total. I²C addresses 0x20, 0x21, 0x22 (on bus 0) |

### 2.3 IR Sensors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 3 | QRE1113 or ITR8307 reflectance sensor | 48 | 2 per gate × 24 gates. ITR8307 from LCSC is cheaper (~€0.13 each) |

### 2.4 MOSFETs (IR LED switching)

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 4 | IRLB8721PbF N-channel MOSFET (TO-220) | 3 | 3.3 V gate-safe. One per MCP23017: Q1 = LED_BANK_1 (gates 00..07), Q2 = LED_BANK_2 (gates 10..17), Q3 = LED_BANK_3 (gates 20..27). Individually switchable at runtime — see [Power Budget](#6-power-budget) |

### 2.5 Resistors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 5 | 22 Ω bussed SIP resistor (9-pin, 8R) | 3 | IR LED current limiting (2 LEDs in series per gate) |
| 6 | 100 kΩ bussed SIP resistor (9-pin, 8R) | 6 | Sensor pull-up resistors (1 per MCP23017 input, to +3.3 V) |
| 7 | 10 kΩ resistor (through-hole) | 6 | MOSFET gate pull-downs (3×), MCP23017 RESET pull-ups (3×) |
| 8 | 4.7 kΩ resistor (through-hole) | 2 | Bus 0 I²C pull-ups (SDA + SCL to 3.3 V), R4/R5 |

> J1 (the retired HiveScale link) does **not** need on-board pull-ups — those were
> provided by the HiveScale-side I²C network.

### 2.6 Capacitors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 9 | 100 nF ceramic capacitor (through-hole) | 6 | Decoupling, 1 per MCP23017 VDD pin + 3 spare |
| 10 | 10 µF electrolytic capacitor | 1 | Bulk decoupling on 3.3 V rail |

### 2.7 Connectors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 11 | 4-pin JST-PH or screw terminal (I²C bus 1 + GND + 3.3 V) | 1 | Connection cable to HiveScale (J1) |
| 12 | 2-pin screw terminal | 1 | 3.3 V power input from HiveScale |
| 13 | USB-C connector (on C6 mini board) | — | For firmware flashing / serial only |

### 2.8 PCB & Housing

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 14 | Custom PCB (single board, black substrate) | 1 | Designed in KiCad (`PCB_files/`). Order black. See Section 4 |
| 15 | 3D printed top baffle / housing | 1 | Carbon-filled PETG or ASA. See Section 5 |
| 16 | M2×6 screws | 8 | PCB to housing standoffs |
| 17 | M2 brass heat-set inserts | 8 | Into 3D printed housing |

---

## 3. Wiring & Connection Map

### 3.1 I²C Buses

This board uses **two separate I²C buses**, one per controller:

**Bus 0 (`Wire`) — MCP23017 master bus.** Silk D4 = GPIO22 = SDA, silk D5 = GPIO23 = SCL. On-board
4.7 kΩ pull-ups (R4/R5) to 3.3 V.

```
XIAO D4 = GPIO22 (SDA) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
                        |      |      |
                      MCP1   MCP2   MCP3
                      0x20   0x21   0x22

XIAO D5 = GPIO23 (SCL) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
```

**J1 (silk D2 / D3 = GPIO2 / GPIO21) — the retired HiveScale slave bus.** The pads and traces
are on the board, and pull-ups were supplied by the HiveScale side. **No current
firmware brings this controller up**, so the pins stay inputs and populating J1
is optional — see the note in Section 1.

> **Note on the schematic net names:** the schematic labels the master bus nets
> "/SDA" and "/SDC" (a typo for SCL), and the LED-bank nets "/GPIO4" / "/GPIO5"
> / "/GPIO6". Those are just net *names* — physically the LED banks are driven
> from silk D8 = GPIO19, D9 = GPIO20 and D10 = GPIO18. U5 is a Seeed XIAO ESP32C6, whose silk
> numbers are **not** its GPIO numbers; see `Firmware/include/pins.h` for the
> authoritative map.

### 3.2 MCP23017 Wiring (repeat for each of the 3 chips, all on bus 0)

| MCP23017 Pin | Connection | Notes |
|---|---|---|
| 9 (VDD) | 3.3 V | Add 100 nF ceramic cap to GND nearby |
| 10 (VSS) | GND | |
| 12 (SCL) | Bus 0 SCL (silk D5 = GPIO23) | |
| 13 (SDA) | Bus 0 SDA (silk D4 = GPIO22) | |
| 15 (A0) | GND / 3.3 V | Address bit 0 (see table below) |
| 16 (A1) | GND / 3.3 V | Address bit 1 |
| 17 (A2) | GND / 3.3 V | Address bit 2 |
| 18 (RESET) | 3.3 V via 10 kΩ | Tie high; no active reset needed |
| 21–28 (GPA0–7) | IR sensor outputs (inner sensors) | 8 sensors per chip |
| 1–8 (GPB0–7) | IR sensor outputs (outer sensors) | 8 sensors per chip |

**Address configuration:**

| Chip | I²C Address | A2 | A1 | A0 | Gates |
|------|-------------|----|----|----|----|
| MCP1 (U2) | 0x20 | GND | GND | GND | 00–07 |
| MCP2 (U3) | 0x21 | GND | GND | 3.3 V | 10–17 |
| MCP3 (U4) | 0x22 | GND | 3.3 V | GND | 20–27 |

> Gate tags have gaps (08, 09, 18, 19, 28, 29 are skipped) to keep the
> per-chip pin→gate map clean; there are 24 physical gates indexed 0..23
> internally.

### 3.3 IR Sensor Wiring (per sensor pair, 24× repeated)

Each gate has 2 sensors — one "inner" (towards hive), one "outer" (towards field).

```
QRE1113 pin 1 (Anode / LED+)   ——→ LED string common (+ / 3.3 V via 22 Ω)
QRE1113 pin 2 (Cathode / LED-) ——→ MOSFET Drain (bank rail)
QRE1113 pin 3 (Collector)      ——→ MCP23017 GPIO input pin + 100 kΩ pull-up to 3.3 V
QRE1113 pin 4 (Emitter)        ——→ GND
```

Logical convention in firmware: **sensor reads LOW = beam reflected/blocked
(bee in beam); HIGH = clear or LEDs off.**

### 3.4 MOSFET Wiring (IRLB8721PbF, 3× identical — one per MCP23017)

| IRLB8721 Pin | Connection |
|---|---|
| Gate (pin 1) | Bank enable GPIO from the table below, + 10 kΩ pull-down to GND |
| Drain (pin 2) | IR LED cathode strings for that bank |
| Source (pin 3) | GND |

| FET | Bank | Gates | XIAO silk | GPIO | Schematic net |
|---|---|---|---|---|---|
| Q1 | LED_BANK_1 | 00..07 (U2 @ 0x20) | D8  | GPIO19 | /GPIO4 |
| Q2 | LED_BANK_2 | 10..17 (U3 @ 0x21) | D9  | GPIO20 | /GPIO5 |
| Q3 | LED_BANK_3 | 20..27 (U4 @ 0x22) | D10 | GPIO18 | /GPIO6 |

Driving the gate HIGH turns that bank's IR emitters ON.

### 3.5 HiveScale I²C Connection Cable (bus 1, connector J1)

4-wire cable between HiveScale and bee counter PCB:

| Wire | Signal |
|---|---|
| Red | 3.3 V (from HiveScale to power the C6 + MCP23017s) |
| Black | GND |
| Yellow | SDA (silk D2 = GPIO2) |
| Blue | SCL (silk D3 = GPIO21) |

This cable served the retired wired link and is not needed for a BLE counter;
the board only needs 3.3 V and GND. The MCP23017s live on their own bus and were
never visible over J1 in any case.

---

## 4. PCB Dimensions & Layout

### 4.1 Target Dimensions

Based on standard Langstroth 10-frame hive (476 mm internal entrance width):

| Dimension | Value | Notes |
|---|---|---|
| PCB length | 375 mm | Covers full entrance, leaving margin each side for housing walls |
| PCB width | 40 mm | Enough for components + connector |
| Gate pitch | 15.6 mm | 375 mm / 24 gates. Each gate ~12 mm clear opening + 3.6 mm divider wall |
| Sensor pair spacing | 8 mm | Inner-to-outer sensor distance within one gate |

**Note for European Zander/Deutsch Normal hives:** entrance width is typically 370 mm — adjust PCB length accordingly.

### 4.2 Component Placement

```
[USB-C]  [C6 mini]  [MCP1]  [MCP2]  [MCP3]  [J1 I²C connector]
                    ← electronics zone, left end →

← 24 sensor pairs spread across the remaining length →

Top edge: inner sensors (facing hive interior)
Bottom edge: outer sensors (facing landing board)
```

- ESP32-C6 and MCP23017s cluster at the left end of the PCB
- Sensor pairs run along the full length
- Q1/Q2 MOSFETs near the sensor zone centre with thermal via to bottom copper pour
- Keep the MCP I²C traces short; keep the J1 routing away from the MCP bus
- 100 nF decoupling caps placed directly adjacent to each MCP23017 VDD pin

---

## 5. 3D Printed Housing Specification

### 5.1 Material

| Property | Requirement |
|---|---|
| Material | Carbon-filled PETG (e.g. Prusament PETG CF) or ASA |
| Color | Black (carbon content ensures genuine IR opacity at 950 nm) |
| Layer height | 0.2 mm or finer for gate walls |
| Infill | 40%+ for structural rigidity |

Do NOT use standard black PLA or standard black PETG — most black dyes are IR-transparent at 950 nm. Carbon-filled variants are the exception.

### 5.2 Housing Parts

**Part 1 — Bottom tray** (holds PCB):
- Internal width: PCB width + 2 mm clearance
- Internal length: PCB length + 2 mm clearance
- Depth: 12 mm (enough for tallest through-hole component)
- 4 M2 standoff bosses at corners, 3 mm proud of floor
- Cable exit slot at left end for I²C cable

**Part 2 — Top baffle** (replaces the original second PCB):
- Same footprint as bottom tray
- 24 channel slots cut through, matching sensor pair positions
- Channel slot width: 12 mm (bee passage width)
- Channel slot height (depth of baffle): 10 mm
- Divider wall thickness between channels: 3.6 mm
- Snap or M2 screw attachment to bottom tray (4 points)
- Landing ramp angle on outer face: 15° downward slope toward hive entrance

### 5.3 Gate Geometry

```
Top view of one gate channel:
┌────────────────────────┐
│  ←── 12 mm ──→         │  ← bee passage
│  [outer sensor]         │  ← facing landing board
│          ↕ 8 mm         │
│  [inner sensor]         │  ← facing hive
└────────────────────────┘
     ← 3.6 mm wall →
```

Sensors face upward from the bottom PCB into the channel. The black baffle absorbs any IR not reflected by a bee.

---

## 6. Power Budget

| Component | Current | Count | Total |
|---|---|---|---|
| ESP32-C6 mini (active, BLE advertising) | 15 mA | 1 | 15 mA |
| MCP23017 (active) | 1 mA | 3 | 3 mA |
| Quiescent leakage | — | — | ~1 mA |
| **Electronics subtotal** | | | **~19 mA** |
| IR emitters, all banks lit (peak) | ~20–40 mA/gate | 24 gates | **~0.5–1.0 A** |
| IR emitters, pulsed at 35 % duty (average) | | | **~170–350 mA** |
| HiveScale ESP32 (sleep) | ~0.05 mA | 1 | 0.05 mA |
| HiveScale ESP32 (awake, ~10 min cycle) | 80 mA × 30 s / 600 s | 1 | ~4 mA avg |

**The emitters are the power budget.** Each gate drives two IR LEDs in series
through a 22 Ω ballast off the 3.3 V rail — roughly (3.3 − 2·V_f)/22, i.e. tens
of milliamps per gate — and the pulsed sampler lights all three banks together
for the settle+read window of every 5 ms poll. Everything else on the board
together is under 20 mA, an order of magnitude below. The per-gate figure
depends on the actual LED forward voltage; **measure it on an assembled board**
before sizing a panel or a pack, rather than trusting the range above.

Three things follow, and all three are implemented:

* **Pulsed emitters** (`LedMode::AUTO`, the default since the 2026-06 revision)
  cut the duty cycle from 100 % to ~35 %. Raising `POLL_INTERVAL_MS` lowers it
  proportionally. There is no `CMD_LEDS_OFF` — that was an I²C command on the
  wired link, which no longer exists; the bench overrides are the `-DIR_DEBUG`
  console's `1`/`0`/`a` keys.
* **Night mode** parks the emitters entirely through the hours honey bees do not
  fly, on request from HiveHub. Over an 8–12 h night that is the difference
  between the counter fitting an off-grid budget and not. See
  [`docs/ble-mode.md`](docs/ble-mode.md#night-mode-the-control-characteristic) —
  including why it is *not* implemented as deep sleep.
* **Per-bank enables** switch the three MOSFETs individually, so an entrance
  narrower than 24 gates costs only the banks it uses. Measured on an assembled
  board, at 3.3 V:

  | Banks enabled | Gates counted | Draw |
  |---|---|---|
  | 1 | 8 | ~0.14 A |
  | 2 | 16 | ~0.22 A |
  | 3 (default) | 24 | ~0.30 A |

  Roughly 80 mA per bank on top of a ~60 mA floor. All three are enabled unless
  HiveHub says otherwise (three checkboxes per device in its dashboard), and a
  counter that resets comes back with all three on. It composes with night mode
  rather than competing with it. See
  [`docs/ble-mode.md`](docs/ble-mode.md#emitter-banks-the-other-power-control).

Add solar for indefinite runtime.

---

## 7. BLE Data Handoff

The counter is a connectable BLE peripheral. The canonical definition of the
GATT contract lives in [`docs/ble-mode.md`](docs/ble-mode.md); this is a summary.

- Advertises as `BeeCounter`; HiveHub connects by the MAC paired in its portal.
- One service, `8e8b0101-7a1c-4b9e-9a2f-1d6e0b9c1a01`, holding a READ
  measurement characteristic, a READ/WRITE control characteristic (night mode
  and emitter-bank enables) and three OTA characteristics.
- The measurement value is built on read, so it is never a stale snapshot:

```json
{"fw":4,"ver":"0.2.0","uptime_s":1234,"status":15,"num_gates":24,
 "mcps_healthy":3,"total_in":100,"total_out":95,"glitches":2,"idle_s":0}
```

`mcps_healthy` counts MCP23017 port expanders (0..3), not gates — each covers
eight of the 24. `idle_s` is the night-mode countdown, `0` while counting. `fw`
is the wire format's revision, not the image version (`ver`); see
[docs/ble-mode.md](docs/ble-mode.md) for the revision history and the deployment
order it requires.

### Totals only

`total_in` / `total_out` are monotonic lifetime counters; HiveHub differences
consecutive reads into per-interval counts on its server. Nothing on the device
is consumed by being read, so a missed connection cannot lose traffic.

This replaced a register map with interval counters that a `CMD_LATCH` command
zeroed — where a HiveScale that read but failed to latch, or latched but failed
to read, silently lost an interval.

### OTA over BLE

HiveHub relays a new C6 image (downloaded over WiFi) into the counter's OTA
characteristics: BEGIN (size + CRC-32) → DATA frames → END (verify, swap slots,
reboot). Gate counting pauses for the duration. See
[`docs/ble-mode.md`](docs/ble-mode.md).

---

## 8. Firmware Summary

Implemented in `Firmware/` (PlatformIO, `seeed_xiao_esp32c6` env). See
`Firmware/README.md` for build/flash and tuning details.

### ESP32-C6 (bee counter)
- **`Wire` (master):** continuously polls the 3× MCP23017 (~5 ms loop), reading
  all 16 inputs per chip. Read failures are detected, so an expander that goes
  missing has its gates skipped rather than read as "all beams blocked", and is
  re-probed until it comes back.
- **BLE/GATT peripheral:** serves lifetime totals as JSON on read, accepts a
  firmware image on the OTA characteristics, and takes a bounded "stop sensing"
  request plus an emitter-bank enable mask on the control characteristic
  (`src/ble_link.cpp`).
- Per-gate debounce + direction state machine (IDLE → INNER/OUTER_FIRST →
  PAIRED) emits IN/OUT counts; glitches tallied for diagnostics. The logic lives
  in `Firmware/include/gate_logic.h` and is covered by host-side tests
  (`Firmware/test/run_tests.sh`).
- IR banks driven on GPIO19 / silk D8 (bank 1), GPIO20 / silk D9 (bank 2) and
  GPIO18 / silk D10 (bank 3) — one MOSFET per MCP23017. LED mode and per-bank
  enables are both settable from the IR_DEBUG serial console (`1`/`0`/`a` and
  `4`/`5`/`6`), and the bank mask is settable over BLE by HiveHub.
- BLE OTA image receiver (`Update` library) with size + CRC-32 verification
  before the inactive app slot is selected.

### HiveHub ESP32 (data aggregator — lives in the HiveHub firmware, not here)
- On wake: read RTC timestamp, read weight, connect to the counter over BLE and
  read its lifetime totals.
- Difference consecutive totals server-side to get the interval counts.
- Write combined record to SD card; transmit via WiFi if available.
- Optionally relay a firmware update to the C6 over BLE (`update_beecounter`).
- Optionally re-arm the counter's night-mode suspension for the next cycle, and
  re-assert its emitter-bank enable mask.
- Sleep ~10 minutes.

---

## 9. Key Datasheet References

| Component | Key parameter |
|---|---|
| IRLB8721PbF | Vgs(th) max 2.4 V — fully on at 3.3 V ✓ |
| MCP23017 | 3× address pins → up to 8 devices on one bus; 1.7 MHz I²C |
| QRE1113 | 950 nm IR, 20 mA forward current, reflective mode |
| ESP32-C6 | **Two** independent I²C controllers; GPIO map is board-specific (see `pins.h`) |
| DS3231 | ±2 ppm accuracy, integrated TCXO, I²C, backup battery (on HiveScale) |

---

*Document version 2.0 — based on the 2019-easy-bee-counter project by
hydronics2, redesigned for MCP23017 I²C, ESP32-C6 mini, dual independent I²C
buses, OTA-over-I²C, dual-hive addressing, single PCB + 3D printed housing, and
HiveScale integration.*
