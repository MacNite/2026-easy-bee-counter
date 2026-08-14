#!/bin/sh
# Build and run the host-side logic tests.
#
# These cover include/gate_logic.h — the sensor debounce, the per-gate crossing
# state machine and the saturating counters — none of which need an ESP32, an
# MCP23017 or a bee. Everything hardware-facing stays in src/main.cpp and is
# still verified on the bench (see the IR_DEBUG console in the README).
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
