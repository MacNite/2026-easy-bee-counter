# Easy Bee Counter 2026 — ESP32-C6 mini firmware

Firmware for the new ESP32-C6 bee counter PCB. Continuously polls the 24
entrance gates, detects directional bee crossings, and serves the results
to the HiveScale main board over I2C.

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

The IR emitters are split into two banks driven by IRLB8721 N-channel MOSFETs:

- **LED_BANK_1** — gates 00..13, FET driven by ESP32-C6 GPIO8 (silk "D8")
- **LED_BANK_2** — gates 14..27, FET driven by ESP32-C6 GPIO9 (silk "D9")

Driving the GPIO HIGH turns the bank's emitters on.

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

### I2C role: master most of the time, slave on demand

This is the most important thing to understand about the board.

The ESP32-C6 has **only one I2C hardware peripheral**. The external
connector J1 (which goes to the HiveScale) is wired to the **same** SDA/SCL
net that the on-board MCP23017s live on. So we share one bus between four
slaves (3× MCP, 1× ESP32-C6 in slave mode) and two potential masters
(the HiveScale and the ESP32-C6 itself).

To make this work without a hardware change, the firmware time-multiplexes:

```
 │■■■■■ MASTER 50 ms ■■■■■│SLAVE 5 ms│■■■■■ MASTER 50 ms ■■■■■│SLAVE 5 ms│ ...
```

- During the master window, the C6 polls the MCP23017s (~5 ms per loop iteration).
- During the slave window, the C6 listens for HiveScale transactions at
  address `0x30`.

The HiveScale only polls every ~10 minutes, so it almost always hits the
slave window on first try. If it hits the master window it will get a NACK
or a bus stretch — the HiveScale code (which lives in the main HiveScale
firmware, not here) must retry once. The `REG_BUSY_RETRIES` field surfaces
how often this happens.

> **Cleaner alternative for v2 of the PCB:** cut the J1↔SDA/SCL traces and
> reroute the HiveScale link to two unused ESP32-C6 pins (D0..D3, D10),
> then bit-bang software I2C for the MCP23017s on D4/D5 and use the
> hardware peripheral exclusively in slave mode on the rerouted pins. That
> removes the role-switching entirely. The current firmware works on the
> as-built PCB.

---

## I2C slave protocol — what the HiveScale should do

See `include/i2c_slave_protocol.h` for the canonical definitions. Slave
address is `0x30`.

### Typical 10-minute poll cycle from the HiveScale

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

> **Recommended retry pattern**: if any of the reads above fails (NACK,
> short transfer), retry the whole cycle once after a 50 ms delay — that
> guarantees you have left the C6's current master window. Don't send
> CMD_LATCH unless every read succeeded, or you'll lose the interval's data.

### Register cheat-sheet

| Addr   | Width  | Name                | Meaning                                       |
| ------ | ------ | ------------------- | --------------------------------------------- |
| `0x00` | 1      | PROTOCOL_VERSION    | Currently `1`                                 |
| `0x01` | 1      | STATUS              | Bitfield (see header for `STATUS_*`)          |
| `0x02` | 2      | UPTIME_S            | Seconds since boot (clipped at 65535)         |
| `0x04` | 1      | NUM_GATES           | Always 24                                     |
| `0x05` | 1      | GATES_HEALTHY       | 0..3, number of MCPs that ACK'd at boot       |
| `0x10` | 4      | TOTAL_IN            | Lifetime inbound bees                         |
| `0x14` | 4      | TOTAL_OUT           | Lifetime outbound bees                        |
| `0x18` | 4      | INTERVAL_IN         | Inbound since last LATCH                      |
| `0x1C` | 4      | INTERVAL_OUT        | Outbound since last LATCH                     |
| `0x20` | 2      | GLITCH_COUNT        | Sensor noise events since boot                |
| `0x22` | 2      | BUSY_RETRIES        | Times slave NACK'd because mid-master cycle   |
| `0x30` | 24     | PER_GATE_IN         | Per-gate inbound count since last LATCH       |
| `0x48` | 24     | PER_GATE_OUT        | Per-gate outbound count since last LATCH      |
| `0x80` | 1 w-o  | CTRL                | Write a CMD_* (see header)                    |

All multi-byte fields are **big-endian on the wire**.

---

## Building & flashing

```bash
pio run               # build
pio run -t upload     # flash via USB
pio device monitor    # 115200 baud serial output
```

PlatformIO target is `esp32-c6-devkitc-1`, which is the right board profile
for the ESP32-C6-MINI-1 module that's on this PCB.

---

## Tuning

All the knobs are at the top of `src/main.cpp`:

| Constant                  | Default | Effect                                                        |
| ------------------------- | ------- | ------------------------------------------------------------- |
| `MASTER_WINDOW_MS`        | 50      | Time in master mode between slave windows                     |
| `SLAVE_WINDOW_MS`         | 5       | Time in slave mode each cycle                                 |
| `SENSOR_DEBOUNCE_MS`      | 3       | Minimum stable time before a sensor state change counts       |
| `GATE_PAIRING_WINDOW_MS`  | 2000    | Max time between inner/outer trip for a directional count     |
| `SENSOR_STUCK_MS`         | 30000   | After this many ms continuously blocked, fault flag is raised |
| `I2C_MASTER_HZ`           | 100000  | I2C bus clock. Lower if you see errors over long cables       |

If your hive has unusually long entrance tunnels or slow-moving bees, raise
`GATE_PAIRING_WINDOW_MS`. If you see noise events on `GLITCH_COUNT`, raise
`SENSOR_DEBOUNCE_MS`.

---

## Wiring sanity check

When you power the board for the first time you should see, on the serial
monitor:

```
==============================================
Easy Bee Counter 2026 — firmware booting
==============================================
[MCP] U2 (gates 00..07) @ 0x20: OK
[MCP] U3 (gates 10..17) @ 0x21: OK
[MCP] U4 (gates 20..27) @ 0x22: OK
[SETUP] Entering normal counting loop
```

If any MCP shows `NOT FOUND`, check:

- That the A0/A1/A2 strap pins of that chip are connected as the schematic
  says (U2=0x20: all GND; U3=0x21: A0→3V3; U4=0x22: A1→3V3).
- That the 4.7 k pull-ups R4/R5 are populated.
- That the chip's VDD pin 9 is at 3.3 V and VSS pin 10 is at GND.

After boot, breaking a gate's beam (e.g. with a piece of paper) should
trigger a serial counter increase on the next 30 s status dump.
