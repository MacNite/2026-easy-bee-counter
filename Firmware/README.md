# Easy Bee Counter 2026 — ESP32-C6 mini firmware

Firmware for the ESP32-C6 mini bee counter PCB. Continuously polls the 24
entrance gates, detects directional bee crossings, and serves the results to
the HiveScale main board over a dedicated I2C link.

> **2026 dual-I2C revision:** this firmware uses **both** of the ESP32-C6's
> independent I2C controllers — one permanently MASTER for the on-board
> MCP23017s, one permanently SLAVE for the HiveScale. The old single-bus
> time-multiplexing scheme is gone.

---

## How it works

### Sensing

Each of the 24 gates has two QRE1113 reflective IR sensors:

- **Inner** — toward the hive interior
- **Outer** — toward the outside world

The 48 sensor lines are read through 3 MCP23017 I2C port expanders on **bus 0**:

| MCP   | I2C addr | Gates       | Inner sensors  | Outer sensors  |
| ----- | -------- | ----------- | -------------- | -------------- |
| U2    | 0x20     | GATE_00..07 | GPA0..GPA7     | GPB0..GPB7     |
| U3    | 0x21     | GATE_10..17 | GPA0..GPA7     | GPB0..GPB7     |
| U4    | 0x22     | GATE_20..27 | GPA0..GPA7     | GPB0..GPB7     |

The IR emitters are split into two banks driven by IRLB8721 N-channel MOSFETs:

- **LED_BANK_1** — gates 00..13, FET driven by ESP32-C6 GPIO8 (silk "D8")
- **LED_BANK_2** — gates 14..27, FET driven by ESP32-C6 GPIO9 (silk "D9")

Driving the GPIO HIGH turns the bank's emitters on. In the default `LedMode::AUTO` the emitters are **pulsed**: both banks are lit only for the settle + MCP-read window of each poll (~1.75 ms at 100 kHz), then switched off until the next poll. This drops the emitter duty cycle from 100% to roughly 35% at  the default 5 ms poll interval, cutting average emitter current proportionally, with no change to detection behaviour. `CMD_LEDS_ON` forces the old always-on behaviour for bench work; `CMD_LEDS_OFF` blacks them out; `CMD_LEDS_AUTO` returns to pulsed mode.

### Counting

Each gate is a small state machine:

```
   IDLE ──Inner blocked──▶ INNER_FIRST ──Outer blocked──▶  count OUT
        ──Outer blocked──▶ OUTER_FIRST ──Inner blocked──▶  count IN
                          (within 2 s, else timeout)
```

After a count, the gate waits in `PAIRED` until both sensors are clear,
then returns to `IDLE`. Glitches (both blocked simultaneously, or only one
ever blocks before timeout) are counted in `REG_GLITCH_COUNT` for diagnostics.

### Two dedicated I2C buses — no role switching

This is the most important thing to understand about the board.

The ESP32-C6 has **two independent I2C hardware controllers**, and this
revision dedicates one to each role:

```
  Wire   (bus 0): MASTER only, GPIO4 (SDA) / GPIO5 (SCL)  -> 3× MCP23017
  Wire1  (bus 1): SLAVE  only, GPIO2 (SDA) / GPIO3 (SCL)  -> HiveScale link (J1)
```

Bus 0 has on-board 4.7 kΩ pull-ups (R4/R5). Bus 1's pull-ups are provided by
the HiveScale-side I2C network. The two buses are electrically separate, so:

- The loop polls the MCP23017s on `Wire` whenever it wants (~5 ms cadence).
- `Wire1` answers the HiveScale **asynchronously** in its `onReceive` /
  `onRequest` callbacks. The HiveScale can start a transaction at any instant
  and can never collide with an MCP read.
- There is **no** master/slave time-multiplexing and **no** retry requirement.
  `REG_BUSY_RETRIES` is retained in the protocol for compatibility but is now
  always `0`.

> This is the "v2 PCB" arrangement the original single-bus firmware looked
> forward to: the J1↔master-bus traces are cut and the HiveScale link is on its
> own controller (GPIO2/GPIO3).

### Dual-hive addressing

The slave address defaults to **0x30**. To run two counters on one HiveScale
bus, flash the second unit with a different address:

```bash
pio run -t upload                                   # hive 1 -> 0x30 (default)
pio run -t upload -e esp32-c6-devkitc-1 \
    --build-flag "-DBEECOUNTER_I2C_ADDRESS=0x31"    # hive 2 -> 0x31
```

(The macro is defined in `include/pins.h`; override it via a build flag.)

---

## I2C slave protocol — what the HiveScale should do

See `include/i2c_slave_protocol.h` for the canonical definitions. Slave
address is `0x30` (or `0x31` for a second hive).

### Typical poll cycle from the HiveScale

```c
// (pseudo-C; this lives in the HiveScale firmware, not here)
#define BEE_ADDR 0x30

uint32_t in, out;
uint8_t  status;
uint8_t  per_gate_in[24], per_gate_out[24];

// 1. quick health check
Wire.beginTransmission(BEE_ADDR); Wire.write(0x01 /*REG_STATUS*/); Wire.endTransmission();
Wire.requestFrom(BEE_ADDR, 1); status = Wire.read();

// 2. read interval counters (the new traffic since last poll)
Wire.beginTransmission(BEE_ADDR); Wire.write(0x18 /*REG_INTERVAL_IN*/); Wire.endTransmission();
Wire.requestFrom(BEE_ADDR, 4);
in  = (uint32_t)Wire.read() << 24;
in |= (uint32_t)Wire.read() << 16;
in |= (uint32_t)Wire.read() << 8;
in |= (uint32_t)Wire.read();

Wire.beginTransmission(BEE_ADDR); Wire.write(0x1C /*REG_INTERVAL_OUT*/); Wire.endTransmission();
Wire.requestFrom(BEE_ADDR, 4);
out  = (uint32_t)Wire.read() << 24;
// ... etc

// 3. (optional) per-gate detail
Wire.beginTransmission(BEE_ADDR); Wire.write(0x30 /*REG_PER_GATE_IN*/); Wire.endTransmission();
Wire.requestFrom(BEE_ADDR, 24);
for (int i = 0; i < 24; i++) per_gate_in[i] = Wire.read();
// ...

// 4. LATCH — atomically zero the interval counters so next poll measures
//    only the next interval. Send this AFTER successful read.
Wire.beginTransmission(BEE_ADDR);
Wire.write(0x80 /*REG_CTRL*/);
Wire.write(0x01 /*CMD_LATCH*/);
Wire.endTransmission();
```

> Because the slave is on its own controller, a transaction won't collide with
> MCP polling. Still, only send `CMD_LATCH` once every read of the interval /
> per-gate registers has succeeded, or you'll discard that interval's data.

### Register cheat-sheet

| Addr   | Width  | Name                | Meaning                                       |
| ------ | ------ | ------------------- | --------------------------------------------- |
| `0x00` | 1      | PROTOCOL_VERSION    | Currently `2`                                 |
| `0x01` | 1      | STATUS              | Bitfield (see header for `STATUS_*`)          |
| `0x02` | 2      | UPTIME_S            | Seconds since boot (clipped at 65535)         |
| `0x04` | 1      | NUM_GATES           | Always 24                                     |
| `0x05` | 1      | GATES_HEALTHY       | 0..3, number of MCPs that ACK'd at boot       |
| `0x10` | 4      | TOTAL_IN            | Lifetime inbound bees                         |
| `0x14` | 4      | TOTAL_OUT           | Lifetime outbound bees                        |
| `0x18` | 4      | INTERVAL_IN         | Inbound since last LATCH                      |
| `0x1C` | 4      | INTERVAL_OUT        | Outbound since last LATCH                     |
| `0x20` | 2      | GLITCH_COUNT        | Sensor noise events since boot                |
| `0x22` | 2      | BUSY_RETRIES        | Retained for compatibility; always 0 (dual-bus) |
| `0x30` | 24     | PER_GATE_IN         | Per-gate inbound count since last LATCH       |
| `0x48` | 24     | PER_GATE_OUT        | Per-gate outbound count since last LATCH      |
| `0x80` | 1 w-o  | CTRL                | Write a CMD_* (see header)                    |
| `0x90` | w-o    | OTA_BEGIN           | size(4) + crc32(4): start OTA-over-I2C        |
| `0x91` | w-o    | OTA_DATA            | offset(4) + data(1..64): stream image         |
| `0x92` | w-o    | OTA_END             | finalize, verify CRC, reboot                  |
| `0x93` | w-o    | OTA_ABORT           | cancel transfer, return to IDLE               |
| `0x94` | 6 r-o  | OTA_STATUS          | state(1) + received(4) + err(1)               |

All multi-byte fields are **big-endian on the wire**.

### OTA-over-I2C firmware updates (PROTOCOL_VERSION ≥ 2)

The HiveScale (which has WiFi) can download a new C6 image and relay it over
I2C:

1. `REG_OTA_BEGIN` — write `size(4) + crc32(4)`. The slave calls
   `Update.begin()`, pauses gate polling, and enters `OTA_STATE_RECEIVING`.
2. `REG_OTA_DATA` — repeatedly write `offset(4) + data` (≤ `OTA_CHUNK_MAX` = 64
   data bytes/frame). The offset must equal bytes-received-so-far or the
   transfer aborts with `OTA_STATE_ERR_SEQ`.
3. `REG_OTA_END` — the slave checks received size and CRC32 (IEEE 802.3, the
   same as `zlib.crc32`), commits with `Update.end(true)`, reports
   `OTA_STATE_DONE`, and reboots shortly after.
4. Poll `REG_OTA_STATUS` at any time for progress / error state. `REG_OTA_ABORT`
   cancels.

Bee counts during the ~20 s transfer are intentionally sacrificed.

---

## Building & flashing

```bash
pio run               # build
pio run -t upload     # flash via USB
pio device monitor    # 115200 baud serial output
```

PlatformIO target is `esp32-c6-devkitc-1`, the right board profile for the
ESP32-C6-MINI-1 module on this PCB. USB CDC is enabled in `platformio.ini`, so
`Serial` output comes out over USB without an external UART bridge.

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
| `I2C_MASTER_HZ`           | 100000  | Bus 0 (MCP) clock. 400 kHz is the MCP spec limit; 100 kHz is the safe default |
| `I2C_SLAVE_HZ`            | 100000  | Bus 1 controller rate (as a slave, the C6 follows the master's clock) |
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
Easy Bee Counter 2026 — firmware booting (dual-I2C)
==============================================
[MCP] U2 (gates 00..07) @ 0x20: OK
[MCP] U3 (gates 10..17) @ 0x21: OK
[MCP] U4 (gates 20..27) @ 0x22: OK
[I2C] HiveScale slave bus up on GPIO2/3 @ 0x30
[SETUP] Entering normal counting loop
```

If any MCP shows `NOT FOUND`, check:

- That the A0/A1/A2 strap pins of that chip are connected as the schematic
  says (U2=0x20: all GND; U3=0x21: A0→3V3; U4=0x22: A1→3V3).
- That the bus 0 4.7 k pull-ups R4/R5 are populated.
- That the chip's VDD pin 9 is at 3.3 V and VSS pin 10 is at GND.

After boot, breaking a gate's beam (e.g. with a piece of paper) should
trigger a serial counter increase on the next 30 s status dump.

The final boot line now reads:

```
[SETUP] Entering normal counting loop (pulsed IR)
```
