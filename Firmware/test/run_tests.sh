#!/bin/sh
# Build and run the host-side logic tests.
#
# These cover the two pure headers, neither of which needs an ESP32, an
# MCP23017 or a bee:
#
#   * include/gate_logic.h      — sensor debounce, the per-gate crossing state
#                                 machine, and the saturating counters;
#   * include/measurement_json.h — the exact bytes of the BLE measurement
#                                 characteristic, i.e. HiveHub's half of the
#                                 wire contract;
#   * include/idle_state.h      — the night-mode suspension deadline, including
#                                 the millis() rollover and the "HiveHub stopped
#                                 re-arming" case;
#   * include/bank_state.h      — the emitter-bank enable mask, whose mistakes
#                                 are eight gates that silently stop counting.
#
# Everything hardware-facing stays in src/main.cpp and is still verified on the
# bench (see the IR_DEBUG console in the README).
#
#     ./test/run_tests.sh
#
# Exits non-zero if any check fails.
set -eu

cd "$(dirname "$0")/.."

CXX=${CXX:-c++}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++11 -Wall -Wextra -Werror \
    -I include \
    test/test_gate_logic/test_gate_logic.cpp \
    -o "$OUT/test_gate_logic"

"$OUT/test_gate_logic"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
    -I include \
    test/test_measurement_json/test_measurement_json.cpp \
    -o "$OUT/test_measurement_json"

"$OUT/test_measurement_json"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
    -I include \
    test/test_idle_state/test_idle_state.cpp \
    -o "$OUT/test_idle_state"

"$OUT/test_idle_state"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
    -I include \
    test/test_bank_state/test_bank_state.cpp \
    -o "$OUT/test_bank_state"

"$OUT/test_bank_state"
