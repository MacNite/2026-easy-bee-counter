# Bee Counter Redesign — Full Specification
### Single PCB + 3D Printed Housing, integrated with HiveScale

A redesign of the [2019-easy-bee-counter](https://github.com/hydronics2/2019-easy-bee-counter)
by hydronics2: 24 entrance gates, 48 reflective IR sensors read through I²C
port expanders, an always-on ESP32-C6 doing the counting, and a HiveScale
ESP32 that aggregates counts with weight/timestamp data.

> **Status:** the PCB (`PCB_files/easy-bee-counter-2026.kicad_*`) and the
> ESP32-C6 firmware (`Firmware/`) are implemented. The 3D-printed housing
> (Section 5) is the remaining mechanical work.

---

## 1. System Overview

Two-MCU architecture. The **ESP32-C6 mini** runs always-on as a dedicated bee
counter. The HiveScale ESP32 sleeps, wakes roughly every 10 minutes, reads the
buffered counts from the C6 over a dedicated I²C link, combines them with
weight/timestamp data, then logs and transmits.

```
[HiveScale ESP32] ←—I²C (bus 1)——→ [ESP32-C6 mini]
  wakes every ~10 min                always-on, ~15 mA
  RTC, SD, WiFi, scale               bee counting, 3× MCP23017 (bus 0)
```

The ESP32-C6 has **two independent I²C controllers**, so each role gets its
own dedicated bus (this is the key change from the original single-bus draft):

- **Bus 0 (`Wire`)** — MASTER only, GPIO4 (SDA) / GPIO5 (SCL) → the 3× MCP23017s.
- **Bus 1 (`Wire1`)** — SLAVE only, GPIO2 (SDA) / GPIO3 (SCL) → the HiveScale link.

Because each role owns its own controller, there is **no master/slave
time-multiplexing** and the HiveScale can poll at any instant with no risk of a
bus collision.

---

## 2. Complete Bill of Materials

### 2.1 Microcontroller

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 1 | ESP32-C6-MINI-1 module | 1 | Always-on bee counter MCU. ~15 mA active, 3.3 V logic, USB. Two hardware I²C controllers |

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
| 4 | IRLB8721PbF N-channel MOSFET (TO-220) | 2 | 3.3 V gate-safe. Q1 controls LED_BANK_1 (gates 00..13), Q2 controls LED_BANK_2 (gates 14..27) |

### 2.5 Resistors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 5 | 22 Ω bussed SIP resistor (9-pin, 8R) | 3 | IR LED current limiting (2 LEDs in series per gate) |
| 6 | 100 kΩ bussed SIP resistor (9-pin, 8R) | 6 | Sensor pull-up resistors (1 per MCP23017 input, to +3.3 V) |
| 7 | 10 kΩ resistor (through-hole) | 5 | MOSFET gate pull-downs (2×), MCP23017 RESET pull-ups (3×) |
| 8 | 4.7 kΩ resistor (through-hole) | 2 | Bus 0 I²C pull-ups (SDA + SCL to 3.3 V), R4/R5 |

> Bus 1 (the HiveScale link) does **not** need on-board pull-ups — those are
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

**Bus 0 (`Wire`) — MCP23017 master bus.** GPIO4 = SDA, GPIO5 = SCL. On-board
4.7 kΩ pull-ups (R4/R5) to 3.3 V.

```
ESP32-C6 GPIO4 (SDA) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
                        |      |      |
                      MCP1   MCP2   MCP3
                      0x20   0x21   0x22

ESP32-C6 GPIO5 (SCL) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
```

**Bus 1 (`Wire1`) — HiveScale slave bus.** GPIO2 = SDA, GPIO3 = SCL. Pull-ups
supplied by the HiveScale side. The C6 listens here as a slave (address 0x30,
or 0x31 for a second hive — see Section 7).

> **Note on the schematic net names:** the schematic labels the master bus nets
> "/SDA" and "/SDC" (a typo for SCL), and the LED-bank nets "/GPIO4" / "/GPIO5".
> Those are just net *names* — physically the LED banks are driven from GPIO8
> and GPIO9. See `Firmware/include/pins.h` for the authoritative map.

### 3.2 MCP23017 Wiring (repeat for each of the 3 chips, all on bus 0)

| MCP23017 Pin | Connection | Notes |
|---|---|---|
| 9 (VDD) | 3.3 V | Add 100 nF ceramic cap to GND nearby |
| 10 (VSS) | GND | |
| 12 (SCL) | Bus 0 SCL (GPIO5) | |
| 13 (SDA) | Bus 0 SDA (GPIO4) | |
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

### 3.4 MOSFET Wiring (IRLB8721PbF, 2× identical)

| IRLB8721 Pin | Connection |
|---|---|
| Gate (pin 1) | ESP32-C6 **GPIO8** (Q1 = bank 1, gates 00..13) or **GPIO9** (Q2 = bank 2, gates 14..27) + 10 kΩ pull-down to GND |
| Drain (pin 2) | IR LED cathode strings for that bank |
| Source (pin 3) | GND |

Driving the gate HIGH turns that bank's IR emitters ON.

### 3.5 HiveScale I²C Connection Cable (bus 1, connector J1)

4-wire cable between HiveScale and bee counter PCB:

| Wire | Signal |
|---|---|
| Red | 3.3 V (from HiveScale to power the C6 + MCP23017s) |
| Black | GND |
| Yellow | SDA (GPIO2) |
| Blue | SCL (GPIO3) |

The C6 acts as an I²C slave (address 0x30) on bus 1. The MCP23017s live on a
**separate** bus (bus 0) and are never visible to the HiveScale.

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
- Keep bus 0 I²C traces short; keep bus 1 (J1) routing away from the MCP bus
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
| ESP32-C6 mini (active, no WiFi) | 15 mA | 1 | 15 mA |
| MCP23017 (active) | 1 mA | 3 | 3 mA |
| IR LEDs (driven continuously while counting) | ~bank current | 2 banks | budget per LED string |
| Quiescent leakage | — | — | ~1 mA |
| **Bee counter total** | | | **~20 mA + LED draw** |
| HiveScale ESP32 (sleep) | ~0.05 mA | 1 | 0.05 mA |
| HiveScale ESP32 (awake, ~10 min cycle) | 80 mA × 30 s / 600 s | 1 | ~4 mA avg |

The C6 firmware currently keeps the IR LEDs on continuously while counting
(`LedMode::AUTO`); they can be forced off over I²C (`CMD_LEDS_OFF`) for
diagnostics. If average power matters, a future revision can pulse the banks.
Add solar for indefinite runtime.

---

## 7. I²C Data Handoff Protocol

The bee counter is a **register-based I²C slave** on bus 1. The canonical
definition lives in `Firmware/include/i2c_slave_protocol.h`; this is a summary.

- Slave address: **0x30** (hive 1). For a **dual-hive** setup, flash the second
  unit with `-DBEECOUNTER_I2C_ADDRESS=0x31`.
- Transaction style: master writes 1 register-pointer byte, then reads N bytes.
- All multi-byte values are **big-endian on the wire**.
- `PROTOCOL_VERSION` is currently **2** (v2 added the OTA-over-I²C block).

### Register cheat-sheet

| Addr | Width | Name | Meaning |
|---|---|---|---|
| 0x00 | 1 | PROTOCOL_VERSION | Currently `2` |
| 0x01 | 1 | STATUS | Bitfield (`STATUS_*`) |
| 0x02 | 2 | UPTIME_S | Seconds since boot (clipped at 65535) |
| 0x04 | 1 | NUM_GATES | Always 24 |
| 0x05 | 1 | GATES_HEALTHY | 0..3, number of MCPs that ACK'd at boot |
| 0x10 | 4 | TOTAL_IN | Lifetime inbound bees |
| 0x14 | 4 | TOTAL_OUT | Lifetime outbound bees |
| 0x18 | 4 | INTERVAL_IN | Inbound since last LATCH |
| 0x1C | 4 | INTERVAL_OUT | Outbound since last LATCH |
| 0x20 | 2 | GLITCH_COUNT | Sensor noise events since boot |
| 0x22 | 2 | BUSY_RETRIES | Retained for compatibility, always 0 on the dual-bus board |
| 0x30 | 24 | PER_GATE_IN | Per-gate inbound count since last LATCH |
| 0x48 | 24 | PER_GATE_OUT | Per-gate outbound count since last LATCH |
| 0x80 | 1 w-o | CTRL | Write a `CMD_*` (LATCH, CLEAR_TOTALS, LED control…) |
| 0x90 | w-o | OTA_BEGIN | size(4) + crc32(4) — start OTA-over-I²C |
| 0x91 | w-o | OTA_DATA | offset(4) + data — stream firmware image |
| 0x92 | w-o | OTA_END | finalize, verify CRC, reboot |
| 0x93 | w-o | OTA_ABORT | cancel transfer |
| 0x94 | 6 r | OTA_STATUS | state(1) + received(4) + err(1) |

### Typical poll cycle

When the HiveScale wakes it: (1) reads STATUS, (2) reads INTERVAL_IN /
INTERVAL_OUT (and optionally PER_GATE_*), then (3) writes `CTRL = CMD_LATCH`
to atomically zero the interval counters so the next interval starts clean.
**Only send `CMD_LATCH` after every read succeeded**, otherwise the interval's
data is lost. The HiveScale then combines the counts with its weight + RTC
timestamp and logs a record.

### OTA-over-I²C

The HiveScale can relay a new C6 firmware image (downloaded over WiFi) to the
bee counter via the `REG_OTA_*` registers: BEGIN (size + CRC32) → repeated
DATA frames → END (verify + reboot). Gate counting pauses for the duration of
the transfer. See `i2c_slave_protocol.h` and `Firmware/README.md`.

---

## 8. Firmware Summary

Implemented in `Firmware/` (PlatformIO, `esp32-c6-devkitc-1` env). See
`Firmware/README.md` for build/flash and tuning details.

### ESP32-C6 (bee counter)
- **Bus 0 (`Wire`, master):** continuously polls the 3× MCP23017 (~5 ms loop),
  reading all 16 inputs per chip via `readGPIOAB()`.
- **Bus 1 (`Wire1`, slave, 0x30):** answers HiveScale register reads/writes via
  `onReceive`/`onRequest` callbacks — no time-multiplexing, no slave window.
- Per-gate debounce + direction state machine (IDLE → INNER/OUTER_FIRST →
  PAIRED) emits IN/OUT counts; glitches tallied for diagnostics.
- IR banks driven on GPIO8 (bank 1) / GPIO9 (bank 2); LED mode controllable
  over I²C.
- OTA-over-I²C image receiver (`Update` library) with CRC32 verification.

### HiveScale ESP32 (data aggregator — lives in the HiveScale firmware, not here)
- On wake: read RTC timestamp, read weight (HX711), poll bee counts from the C6.
- Send `CMD_LATCH` after a successful read.
- Write combined record to SD card; transmit via WiFi if available.
- Optionally relay a firmware update to the C6 over OTA-over-I²C.
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
