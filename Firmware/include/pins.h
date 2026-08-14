// ============================================================================
// pins.h — Easy Bee Counter 2026 hardware map
// ============================================================================
// All physical pin numbers and I2C addresses were derived from the KiCad
// schematic (easy-bee-counter-2026.kicad_sch) and the netlist export
// (easy-bee-counter-2026.net). Update this file if the PCB is revised.
// ============================================================================
//
// ESP32-C6 mini silk-label -> GPIO mapping
// ----------------------------------------
// U5 is a Seeed XIAO ESP32C6 (schematic symbol "ESP32 C6 mini", Seeed part
// 113991054), NOT a bare ESP32-C6-MINI-1 module. That distinction is the whole
// reason this table exists: on the XIAO form factor the silk labels D0..D10 do
// *not* equal the GPIO numbers. The authority is the Arduino core variant
// header variants/XIAO_ESP32C6/pins_arduino.h:
//
//   silk   GPIO   net on this PCB
//   ----   ----   ---------------------------------------------------------
//   D0  -> GPIO0
//   D1  -> GPIO1
//   D2  -> GPIO2   SDA_HiveScale (unused: the wired link was removed)
//   D3  -> GPIO21  SCL_HiveScale (unused: the wired link was removed)
//   D4  -> GPIO22  /SDA   (I2C data, MCP23017 master bus)
//   D5  -> GPIO23  /SDC   (I2C clock, schematic spells it "SDC")
//   D6  -> GPIO16  TX (UART0)                    [unused on this board]
//   D7  -> GPIO17  RX (UART0)                    [unused on this board]
//   D8  -> GPIO19  /GPIO4 net -> Q1 gate (LED_BANK_1 enable, gates 00..07)
//   D9  -> GPIO20  /GPIO5 net -> Q2 gate (LED_BANK_2 enable, gates 10..17)
//   D10 -> GPIO18  /GPIO6 net -> Q3 gate (LED_BANK_3 enable, gates 20..27)
//
// Two traps live in that table, and this firmware fell into both:
//
//   1. The schematic net labels "/GPIO4", "/GPIO5" and "/GPIO6" do NOT mean
//      physical GPIO4/5/6 — they are just net names. Physically they land on U5
//      pins 9, 10 and 11, silk-labelled D8, D9 and D10, i.e. GPIO19, GPIO20 and
//      GPIO18.
//   2. GPIO3..GPIO15 are simply not on the XIAO header. Assigning a bus to one
//      of them compiles and boots happily, drives nothing, and every device on
//      the real bus reports NOT FOUND. GPIO3 in particular is tied to the
//      module's RF antenna switch, so using it also disturbs the radio.
//
// Always write the GPIO number here, never the silk number. The PlatformIO
// board is esp32-c6-devkitc-1, whose variant defines no D-aliases at all.
//
// I2C bus layout
// --------------
//   Bus 0 (Wire) — MASTER, GPIO22/D4 (SDA) / GPIO23/D5 (SCL). Talks to the 3x
//                  MCP23017 port expanders. On-board pull-ups R4/R5 (4.7k each).
//
// The PCB also routes a second bus to J1 on D2/D3 (GPIO2/GPIO21), which earlier
// firmware ran as a permanent I2C slave for a HiveScale. That link is gone: the
// counter reports over BLE only. The pads and traces are harmless — the firmware
// simply never brings up the second controller, so the pins stay inputs.
//
// MCP23017 addresses (set by A0/A1/A2 strap pins, base 0x20):
//   U2 -> A0=0 A1=0 A2=0 -> 0x20  (gates 00..07)
//   U3 -> A0=1 A1=0 A2=0 -> 0x21  (gates 10..17)
//   U4 -> A0=0 A1=1 A2=0 -> 0x22  (gates 20..27)
//
// MCP23017 GPIO assignment per chip:
//   GPA0..GPA7 -> Inner sensor of gate N0..N7 (data line of QRE1113 pin 3)
//   GPB0..GPB7 -> Outer sensor of gate N0..N7 (data line of QRE1113 pin 3)
//
// In the Adafruit_MCP23X17 library, digitalRead(0..7) -> port A,
// digitalRead(8..15) -> port B, so:
//   inner sensor of gate K -> mcp.digitalRead(K)        // 0..7
//   outer sensor of gate K -> mcp.digitalRead(K + 8)    // 8..15
//
// QRE1113 wiring on this board:
//   - Each phototransistor collector goes to its MCP pin, with a 100k
//     pull-up to +3.3V (RN1/RN3/RN5/RN6/RN8/RN9).
//   - IR LEDs on a gate are tied to the LED_BANK_1, LED_BANK_2 or LED_BANK_3
//     rail, with ballast resistors in series and Q1/Q2/Q3 (IRLB8721 N-FETs) on
//     the low side. Driving the FET gate HIGH turns the IR LEDs ON, which
//     causes reflected light to drop the phototransistor collector voltage.
//   - Since the 2026-08 hardware revision there is one FET per MCP23017, so a
//     bank maps 1:1 onto a chip: U2 -> bank 1, U3 -> bank 2, U4 -> bank 3.
//     (The previous 2-FET build split U3's gates across banks 1 and 2.)
//   - Therefore the *logical* convention used in this firmware is:
//        sensor reads LOW  -> bee body in the beam (beam reflected/blocked)
//        sensor reads HIGH -> beam clear OR IR LEDs off
//
// Gate numbering
// --------------
// The PCB has 24 active gates physically. The naming has gaps (08, 09, 18, 19,
// 28, 29 are skipped) to keep the per-chip pin -> gate map clean. We expose
// them as a logical 0..23 dense index internally but keep the original
// "GATE_NM" tags in debug logs and protocol responses for traceability.
//
// ============================================================================

#pragma once

#include <Arduino.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// ESP32-C6 GPIO pin assignments (physical GPIO numbers)
// ---------------------------------------------------------------------------
namespace pins {

// Bus 0 (Wire) — master to the MCP23017s.
constexpr int I2C_SDA           = 22;   // U5 pin 5  / silk "D4" -> /SDA net
constexpr int I2C_SCL           = 23;   // U5 pin 6  / silk "D5" -> /SDC net

// One IRLB8721 per MCP23017 since the 2026-08 revision (was two FETs total).
constexpr int IR_LED_BANK_1_EN  = 19;   // U5 pin 9  / silk "D8"  -> Q1 gate -> gates 00..07
constexpr int IR_LED_BANK_2_EN  = 20;   // U5 pin 10 / silk "D9"  -> Q2 gate -> gates 10..17
constexpr int IR_LED_BANK_3_EN  = 18;   // U5 pin 11 / silk "D10" -> Q3 gate -> gates 20..27

// How many MOSFET-switched emitter rails exist.
constexpr uint8_t NUM_LED_BANKS = 3;

// Bank number (1..NUM_LED_BANKS) -> enable GPIO. Indexed by bank-1 so callers
// that iterate all banks don't have to repeat the pin names; see
// gates::GateLocation::led_bank for the per-gate mapping.
constexpr int IR_LED_BANK_EN[NUM_LED_BANKS] = {
    IR_LED_BANK_1_EN,
    IR_LED_BANK_2_EN,
    IR_LED_BANK_3_EN,
};

}  // namespace pins

// ---------------------------------------------------------------------------
// I2C device addresses
// ---------------------------------------------------------------------------
// Only the MCP23017 expanders remain. The counter no longer has a slave address
// of its own: BEECOUNTER_I2C_ADDRESS (0x30/0x31) existed so two counters could
// share one HiveScale bus, and hives are told apart by BLE MAC now.
namespace i2c_addr {
constexpr uint8_t MCP_GATES_00_07 = 0x20;   // U2
constexpr uint8_t MCP_GATES_10_17 = 0x21;   // U3
constexpr uint8_t MCP_GATES_20_27 = 0x22;   // U4
}  // namespace i2c_addr

// ---------------------------------------------------------------------------
// Gate topology
// ---------------------------------------------------------------------------
namespace gates {

// Number of physical gates that are wired up on the PCB.
constexpr uint8_t NUM_GATES = 24;

// Each gate has an Inner sensor (toward the hive interior) and an
// Outer sensor (toward the outside world). Both are read from the same
// MCP23017 chip; the inner is on port A, the outer is on port B.
struct GateLocation {
    uint8_t mcp_address;   // I2C address of the MCP23017
    uint8_t inner_pin;     // 0..7  (GPA0..GPA7)
    uint8_t outer_pin;     // 8..15 (GPB0..GPB7)
    uint8_t led_bank;      // 1..3 (which MOSFET-controlled LED rail); index
                           // pins::IR_LED_BANK_EN[led_bank - 1] for its GPIO
    const char* tag;       // original schematic name, e.g. "GATE_03"
};

// The 24 physical gates, indexed 0..23. The "tag" field carries the
// original schematic name so debug logs match the PCB silk and the netlist.
constexpr GateLocation TABLE[NUM_GATES] = {
    // U2 (0x20): gates 00..07, all on LED_BANK_1
    { i2c_addr::MCP_GATES_00_07, 0,  8, 1, "GATE_00" },
    { i2c_addr::MCP_GATES_00_07, 1,  9, 1, "GATE_01" },
    { i2c_addr::MCP_GATES_00_07, 2, 10, 1, "GATE_02" },
    { i2c_addr::MCP_GATES_00_07, 3, 11, 1, "GATE_03" },
    { i2c_addr::MCP_GATES_00_07, 4, 12, 1, "GATE_04" },
    { i2c_addr::MCP_GATES_00_07, 5, 13, 1, "GATE_05" },
    { i2c_addr::MCP_GATES_00_07, 6, 14, 1, "GATE_06" },
    { i2c_addr::MCP_GATES_00_07, 7, 15, 1, "GATE_07" },
    // U3 (0x21): gates 10..17, all on LED_BANK_2
    { i2c_addr::MCP_GATES_10_17, 0,  8, 2, "GATE_10" },
    { i2c_addr::MCP_GATES_10_17, 1,  9, 2, "GATE_11" },
    { i2c_addr::MCP_GATES_10_17, 2, 10, 2, "GATE_12" },
    { i2c_addr::MCP_GATES_10_17, 3, 11, 2, "GATE_13" },
    { i2c_addr::MCP_GATES_10_17, 4, 12, 2, "GATE_14" },
    { i2c_addr::MCP_GATES_10_17, 5, 13, 2, "GATE_15" },
    { i2c_addr::MCP_GATES_10_17, 6, 14, 2, "GATE_16" },
    { i2c_addr::MCP_GATES_10_17, 7, 15, 2, "GATE_17" },
    // U4 (0x22): gates 20..27, all on LED_BANK_3
    { i2c_addr::MCP_GATES_20_27, 0,  8, 3, "GATE_20" },
    { i2c_addr::MCP_GATES_20_27, 1,  9, 3, "GATE_21" },
    { i2c_addr::MCP_GATES_20_27, 2, 10, 3, "GATE_22" },
    { i2c_addr::MCP_GATES_20_27, 3, 11, 3, "GATE_23" },
    { i2c_addr::MCP_GATES_20_27, 4, 12, 3, "GATE_24" },
    { i2c_addr::MCP_GATES_20_27, 5, 13, 3, "GATE_25" },
    { i2c_addr::MCP_GATES_20_27, 6, 14, 3, "GATE_26" },
    { i2c_addr::MCP_GATES_20_27, 7, 15, 3, "GATE_27" },
};

}  // namespace gates