# Bee Counter Redesign — Full Specification
## This is all WIP - Nothing is done yet
### Single PCB + 3D Printed Housing, integrated with HiveScale

---

## 1. System Overview

Two-MCU architecture. The ESP32-C3 SuperMini runs always-on as a dedicated bee counter. The HiveScale ESP32 sleeps every 10 minutes, wakes up, reads buffered counts from the C3 via I²C, combines with weight/timestamp data, then logs and transmits.

```
[HiveScale ESP32] ←—I²C slave——→ [ESP32-C3 SuperMini]
  wakes every 10 min                always-on, ~15 mA
  RTC, SD, WiFi, scale              bee counting, MCP23017s
```

---

## 2. Complete Bill of Materials

### 2.1 Microcontroller

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 1 | ESP32-C3 SuperMini | 1 | Always-on bee counter MCU. ~15 mA active, 3.3 V logic, USB-C |

### 2.2 I/O Expansion (replaces all 6× 74HC165)

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 2 | MCP23017-E/SP (DIP-28) | 3 | 16 inputs each = 48 total. I²C addresses 0x20, 0x21, 0x22 |

### 2.3 IR Sensors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 3 | QRE1113 or ITR8307 reflectance sensor | 48 | 2 per gate × 24 gates. ITR8307 from LCSC is cheaper (~€0.13 each) |

### 2.4 MOSFETs (IR LED switching)

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 4 | IRLB8721PbF N-channel MOSFET (TO-220) | 2 | 3.3 V gate-safe. Controls 2 banks of 24 IR LEDs each |

### 2.5 Resistors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 5 | 22 Ω bussed SIP resistor (9-pin, 8R) | 3 | IR LED current limiting (2 LEDs in series per gate) |
| 6 | 100 kΩ bussed SIP resistor (9-pin, 8R) | 6 | Sensor pull-down resistors (1 per MCP23017 input) |
| 7 | 10 kΩ resistor (through-hole) | 7 | I²C pull-ups (2×), MOSFET gate pull-downs (2×), MCP23017 RESET pull-ups (3×) |
| 8 | 4.7 kΩ resistor (through-hole) | 2 | I²C bus pull-ups (SDA + SCL to 3.3 V) |

### 2.6 Capacitors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 9 | 100 nF ceramic capacitor (through-hole) | 6 | Decoupling, 1 per MCP23017 VDD pin + 3 spare |
| 10 | 10 µF electrolytic capacitor | 1 | Bulk decoupling on 3.3 V rail |

### 2.7 Connectors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 11 | 4-pin JST-PH or screw terminal (I²C + GND + 3.3 V) | 1 | Connection cable to HiveScale |
| 12 | 2-pin screw terminal | 1 | 3.3 V power input from HiveScale |
| 13 | USB-C connector (on C3 SuperMini board) | — | For firmware flashing only |

### 2.8 PCB & Housing

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 14 | Custom PCB (single board, black substrate) | 1 | Designed in KiCad. Order black. See Section 4 for dimensions |
| 15 | 3D printed top baffle / housing | 1 | Carbon-filled PETG or ASA. See Section 5 |
| 16 | M2×6 screws | 8 | PCB to housing standoffs |
| 17 | M2 brass heat-set inserts | 8 | Into 3D printed housing |

---

## 3. Wiring & Connection Map

### 3.1 I²C Bus (shared by all chips)

All devices share a single I²C bus. Pull-ups go to 3.3 V.

```
ESP32-C3 GPIO8 (SDA) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
                        |      |      |
                      MCP1   MCP2   MCP3
                      0x20   0x21   0x22

ESP32-C3 GPIO9 (SCL) ——+——————+——————+——— 4.7 kΩ ——→ 3.3 V
```

### 3.2 MCP23017 Wiring (repeat for each of the 3 chips)

| MCP23017 Pin | Connection | Notes |
|---|---|---|
| 9 (VDD) | 3.3 V | Add 100 nF ceramic cap to GND nearby |
| 10 (VSS) | GND | |
| 12 (SCL) | I²C SCL bus | |
| 13 (SDA) | I²C SDA bus | |
| 15 (A0) | GND / 3.3 V | Address bit 0 (see table below) |
| 16 (A1) | GND / 3.3 V | Address bit 1 |
| 17 (A2) | GND / 3.3 V | Address bit 2 |
| 18 (RESET) | 3.3 V via 10 kΩ | Tie high; no active reset needed |
| 21–28 (GPA0–7) | IR sensor outputs (inner sensors) | 8 sensors per chip |
| 1–8 (GPB0–7) | IR sensor outputs (outer sensors) | 8 sensors per chip |

**Address configuration:**

| Chip | I²C Address | A2 | A1 | A0 | Gates |
|------|-------------|----|----|----|----|
| MCP1 | 0x20 | GND | GND | GND | 0–7 |
| MCP2 | 0x21 | GND | GND | 3.3 V | 8–15 |
| MCP3 | 0x22 | GND | 3.3 V | GND | 16–23 |

### 3.3 IR Sensor Wiring (per sensor pair, 24× repeated)

Each gate has 2 sensors — one "inner" (towards hive), one "outer" (towards field).

```
QRE1113 pin 1 (Anode / LED+)  ——→ LED string common (+)
QRE1113 pin 2 (Cathode / LED-) ——→ 22 Ω resistor → MOSFET Drain
QRE1113 pin 3 (Collector)      ——→ MCP23017 GPIO input pin
QRE1113 pin 4 (Emitter)        ——→ GND
                                     + 100 kΩ pull-down to GND on pin 3
```

### 3.4 MOSFET Wiring (IRLB8721PbF, 2× identical)

| IRLB8721 Pin | Connection |
|---|---|
| Gate (pin 1) | ESP32-C3 GPIO (GPIO4 for bank 1, GPIO5 for bank 2) + 10 kΩ pull-down to GND |
| Drain (pin 2) | IR LED cathode strings (24 LEDs per MOSFET) |
| Source (pin 3) | GND |

### 3.5 HiveScale I²C Connection Cable

4-wire cable between HiveScale and bee counter PCB:

| Wire | Signal |
|---|---|
| Red | 3.3 V (from HiveScale to power C3 + MCP23017s) |
| Black | GND |
| Yellow | SDA |
| Blue | SCL |

The C3 acts as an I²C slave (address 0x30) towards the HiveScale. The MCP23017s and the C3 all share the same physical bus — the HiveScale sees C3 at 0x30, MCP1 at 0x20, MCP2 at 0x21, MCP3 at 0x22.

---

## 4. PCB Dimensions & Layout

### 4.1 Target Dimensions

Based on standard Langstroth 10-frame hive (476 mm internal entrance width):

| Dimension | Value | Notes |
|---|---|---|
| PCB length | 375 mm | Covers full entrance, leaving 50 mm margin each side for housing walls |
| PCB width | 40 mm | Enough for components + connector |
| Gate pitch | 15.6 mm | 375 mm / 24 gates. Each gate ~12 mm clear opening + 3.6 mm divider wall |
| Sensor pair spacing | 8 mm | Inner-to-outer sensor distance within one gate |

**Note for European Zander/Deutsch Normal hives:** entrance width is typically 370 mm — adjust PCB length accordingly.

### 4.2 Component Placement

```
[USB-C]  [C3 SuperMini]  [MCP1]  [MCP2]  [MCP3]  [I²C connector]
                         ← electronics zone, left 120 mm →

← 24 sensor pairs spread across remaining 255 mm →

Top edge: inner sensors (facing hive interior)
Bottom edge: outer sensors (facing landing board)
```

- ESP32-C3 and MCP23017s cluster at the left end of the PCB
- Sensor pairs run along the full length
- MOSFETs near the sensor zone centre with thermal via to bottom copper pour
- All I²C traces kept short and together; run as a differential pair if possible
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
| ESP32-C3 SuperMini (active, no WiFi) | 15 mA | 1 | 15 mA |
| MCP23017 (active) | 1 mA | 3 | 3 mA |
| IR LED pulse (75 µs / 15 ms duty) | ~170 mA peak × 0.5% | 48 | ~1 mA avg |
| Quiescent leakage | — | — | ~1 mA |
| **Bee counter total** | | | **~20 mA** |
| HiveScale ESP32 (sleep) | ~0.05 mA | 1 | 0.05 mA |
| HiveScale ESP32 (awake, 10 min cycle) | 80 mA × 30 s / 600 s | 1 | ~4 mA avg |
| **Combined system total** | | | **~24 mA avg** |

At 24 mA average draw, a 10,000 mAh LiPo battery provides ~17 days of operation. Add solar for indefinite runtime.

---

## 7. I²C Data Handoff Protocol

When the HiveScale ESP32 wakes, it requests 4 bytes from C3 (address 0x30):

| Byte | Content |
|---|---|
| 0–1 | `inTotal` (uint16, little-endian) |
| 2–3 | `outTotal` (uint16, little-endian) |

After a successful read, the HiveScale sends a single reset byte (0xFF) to the C3, which clears both counters. The HiveScale then combines with its own weight + RTC timestamp and logs:

```
timestamp, weight_kg, bees_in, bees_out, net_flow, battery_v, temp_c
```

---

## 8. Firmware Summary (to be developed)

### ESP32-C3 (bee counter)
- I²C master to 3× MCP23017 (polling loop, ~1 ms per full read)
- MOSFET pulse control: GPIO high for 75 µs, latch MCP23017, GPIO low
- Debounce + direction logic per gate (same algorithm as original `bee_counting.ino`)
- I²C slave (addr 0x30) responding to HiveScale requests
- Enable MCP23017 interrupt-on-change (INTA/INTB pins) for power-efficient sensing

### HiveScale ESP32 (data aggregator)
- On wake: read RTC timestamp, read weight (HX711), request bee counts from C3
- Send reset command to C3 after successful read
- Write combined record to SD card
- Transmit via WiFi if available
- Sleep for 10 minutes

---

## 9. Key Datasheet References

| Component | Key parameter |
|---|---|
| IRLB8721PbF | Vgs(th) max 2.4 V — fully on at 3.3 V ✓ |
| MCP23017 | 3× address pins → up to 8 devices on one bus; 1.7 MHz I²C |
| QRE1113 | 950 nm IR, 20 mA forward current, reflective mode |
| ESP32-C3 | GPIO8=SDA, GPIO9=SCL (default I²C); 13 usable GPIOs total |
| DS3231 | ±2 ppm accuracy, integrated TCXO, I²C, backup battery |

---

*Document version 1.0 — based on 2019-easy-bee-counter project by hydronics2, redesigned for MCP23017 I²C, ESP32-C3, single PCB + 3D printed housing, and HiveScale integration.*