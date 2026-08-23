# Project instructions for Claude Code

## Delivery policy — ask if data should be pushed to github directly

This repository uses a **"no remote writes"** workflow. When you finish making
changes, **do not publish them to GitHub automatically** — always ask if it should be pushed and a pr should be created.

**You MUST NOT, under any circumstances unless I explicitly ask in that very message:**

- run `git push` (to any branch or remote);
- create, update, merge, or comment on pull requests;
- create or move remote branches or tags;
- push files through the GitHub API / MCP tools (`create_or_update_file`,
  `push_files`, `create_pull_request`, etc.);
- otherwise transmit repository contents to GitHub or any external service.

Working **locally** is fine: edit files, run builds/tests, and commit to the
local branch if it helps you organize work. Just never send anything to the
remote.

---

## About this project

HiveTraffic is the **bee counter** half of the hive monitoring setup: an
always-on ESP32-C6 mini that watches 24 hive-entrance gates with 48 reflective
IR sensors, works out which direction each bee crossed, and serves lifetime
in/out totals over BLE/GATT. It is a redesign of hydronics2's
[2019-easy-bee-counter](https://github.com/hydronics2/2019-easy-bee-counter),
replacing the 74HC165 shift registers with 3× MCP23017 I²C port expanders and
the second PCB with a 3D-printed baffle.

**There is no server or database in this repo.** The counter's only client is
[MacNite/HiveHub](https://github.com/MacNite/HiveHub), a separate project that
wakes roughly every 10 minutes, connects over BLE, reads one JSON
characteristic, and does all the logging, differencing, and storage on its own
backend.

### Layout

- `Firmware/` — PlatformIO project for the ESP32-C6 (`board = seeed_xiao_esp32c6`).
  - `src/main.cpp` — MCP polling loop, per-gate debounce/direction state machine,
    pulsed IR emitter control, and the `-DIR_DEBUG` serial console.
  - `src/ble_link.cpp` — NimBLE GATT peripheral: the measurement characteristic,
    the night-mode control characteristic and the OTA characteristics.
  - `include/idle_state.h` — night-mode suspension as pure deadline arithmetic.
  - `include/bank_state.h` — the emitter-bank (MOSFET) enable mask, as pure
    bitmask rules.
  - `include/pins.h` — **authoritative** GPIO map for the PCB.
  - `include/counter_protocol.h` — status bitfield + OTA state codes shared
    between `main.cpp` and `ble_link.cpp`.
  - `include/version.h` — image version reported over BLE.
- `PCB_files/` — KiCad schematic/board for `easy-bee-counter-2026`, plus
  `fabrication/` gerbers and drill files.
- `docs/ble-mode.md` — the canonical GATT contract: UUIDs, JSON fields,
  advertising layout, OTA framing.
- `Data/` — sample plots and notes on what the counts mean (foraging peaks,
  orientation flights).
- `OLD/` — archived Arduino sketches, Eagle/KiCad files, and build instructions
  from the 2019 design. Reference material only; nothing here is built.
- `README.md` — the full hardware spec: BOM, wiring maps, PCB dimensions,
  housing geometry, power budget.

### Things to know before changing firmware

- **One build environment** in `Firmware/platformio.ini`: `seeed_xiao_esp32c6`
  (production). The interactive IR sensor console in `main.cpp` is gated behind
  `-DIR_DEBUG`, has no environment of its own, and must never end up in the
  production build; compile it ad hoc with
  `PLATFORMIO_BUILD_FLAGS=-DIR_DEBUG pio run`.
- **Bump `HIVETRAFFIC_FW_VERSION` in `include/version.h` for every released
  image.** HiveHub refuses an OTA relay that isn't newer, and uses the same
  field afterwards to confirm the update took.
- **Night mode is a bounded duration, never a schedule.** The counter has no
  clock and must never be given one: HiveHub writes "stop sensing for N
  seconds", capped at `MAX_IDLE_SECONDS`, and re-arms it each cycle. Every
  failure — a dead HiveHub, a malformed write, a reset — has to end with the
  counter counting. Do not add persistence, and do not replace it with deep
  sleep: the emitters are >90% of the draw, and sleeping costs the measurement
  read, the OTA path and the RAM totals for the remainder. See
  `docs/ble-mode.md`.
- **The counter reports lifetime totals only** — no latch, no reset, no
  per-interval state on the device. HiveHub differences consecutive reads.
  This is deliberate: a missed connection can't lose traffic if nothing is
  consumed by being read. Don't add interval counters back.
- **The wired I²C/HiveScale slave link has been removed.** The J1 connector and
  its traces are still physically on the PCB, but no firmware brings that
  controller up. Don't reintroduce it.
- **`pins.h` beats the schematic.** The schematic's net names are misleading —
  `/SDC` is a typo for SCL, and the `/GPIO4` / `/GPIO5` / `/GPIO6` LED-bank nets
  are just names; the three banks are physically driven from GPIO19, GPIO20 and
  GPIO18 (XIAO silk D8, D9, D10).
- **Three IR emitter banks, one MOSFET per MCP23017** since the 2026-08 hardware
  revision: bank 1 = gates 00..07 (U2), bank 2 = 10..17 (U3), bank 3 = 20..27
  (U4). Older docs describing a 2-FET split (00..13 / 14..27) are stale.
- **Each bank is individually switchable** (protocol v5), because one bank
  costs ~0.14 A, two ~0.22 A and three ~0.30 A at 3.3 V. Same fail-open rules as
  night mode: not persisted, re-asserted by HiveHub every cycle, all three on
  after any reset, and an all-off mask REFUSED rather than applied. Gates on a
  dark bank must be *skipped*, never read as "clear" — an unpowered QRE1113 is a
  bare phototransistor and sunlight into the entrance will pull it low. See
  `include/bank_state.h` and `docs/ble-mode.md`.
- OTA needs the dual-slot `partitions_4mb_ota_no_fs.csv` layout. A board still
  running a single-app image has to be updated once over USB.

### Secrets

The current firmware is BLE-only and holds no credentials — there is nothing to
configure. `OLD/arduino/secrets.h` is a tracked *placeholder* template from the
old WiFi sketch (`MySSID` / `MyPassword`); never fill in real values there, and
never add credentials to tracked files or to a bundle you hand back.