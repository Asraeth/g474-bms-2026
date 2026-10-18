#!/usr/bin/env bash
#
# Builds the firmware against the board simulator with the PC's gcc and runs every test,
# each in its own process (the firmware keeps state in statics).
#
# Usage: test/run_tests.sh [test name ...]     (default: all tests, then every variant)
#   SIM_PESSIMISTIC=1   datasheet-slowest ADC conversion times and slower SPI driver
#   ARM_GCC_BIN         arm-none-eabi-gcc folder, for the register layout check
#   PYTHON              Python with cantools (DBC check), openpyxl (SoC check), GUI/requirements.txt (GUI tests)
#   BENCH_CONFIG        a bms_config_local.h-style file to build with instead of the car's config,
#                       e.g. Core/Inc/bms_config_local.h.example (run tests that fit its slave count)
#   VARIANTS=0          skip the configuration variants
#   SANITIZE=1          undefined-behaviour checks (UBSan, trap mode: a test hitting one dies with "Illegal
#                       instruction"; add -g and run that test in gdb to see where), float-to-int overflow included
#   TICK_WRAP=<s>       HAL_GetTick wraps (as after 49.7 days of uptime) this many seconds into every test;
#                       the tick then starts near 0xFFFFFFFF instead of 0, which code assuming a small tick
#                       right after power-up can't tell apart from a wrap
#   EXTRA_CFLAGS        more compiler flags for the simulation build
#
# Variants (test/variants/*.h): override files like bms_config_local.h. On a full run the firmware is
# rebuilt with each one and runs the tests named on its "// tests:" line ("all except a b" works too).
#
set -uo pipefail

cd "$(dirname "$0")/.."

OUT=test/build
mkdir -p "$OUT"
CC="${CC:-gcc}"
PYTHON="${PYTHON:-python}"

# Everything generated from bms_config.h (DBCs, signal lists, docs) is brought up to date first, and
# the CubeMX settings are checked against the config
if [ "${SKIP_GENERATE:-0}" != "1" ] && ! "$PYTHON" tools/generate.py; then
    echo "FAIL  generate (tools/generate.py)"
    exit 1
fi

INCS="-Itest/include -Itest/sim -ICore/Inc -ICore/custom_lib/inc -IDrivers/STM32G4xx_HAL_Driver/Inc \
      -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G4xx/Include -IDrivers/CMSIS/Include"
if [ "${SANITIZE:-0}" = "1" ]; then
    EXTRA_CFLAGS="${EXTRA_CFLAGS:-} -fsanitize=undefined,float-cast-overflow -fsanitize-undefined-trap-on-error -fno-sanitize-recover=all"
fi
if [ -n "${TICK_WRAP:-}" ]; then
    EXTRA_CFLAGS="${EXTRA_CFLAGS:-} -DSIM_TICK_OFFSET_MS=$(awk -v s="$TICK_WRAP" 'BEGIN { printf "%.0f", 4294967296 - s * 1000 }')u"
fi
CFLAGS="-std=gnu11 -g -O1 -DBMS_SIMULATION_BUILD -DBMS_BOARD_PROFILE=4 -DSTM32G474xx -DUSE_HAL_DRIVER -include test/include/cmsis_sim.h $INCS \
        -Wall -Wextra -Wno-unused-parameter -Wno-old-style-declaration -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
        -fstack-protector-strong ${EXTRA_CFLAGS:-}"

FIRMWARE="Core/Src/bms_health.c Core/custom_lib/src/ivt_s.c Core/custom_lib/src/bms_command_counter.c Core/Src/bms_can.c Core/Src/bms_gpio.c Core/Src/bms_ivt.c Core/Src/bms_safety.c Core/Src/bms_soc.c \
          Core/Src/bms_precharge.c Core/Src/bms_inverter.c Core/Src/bms_settings.c Core/Src/uartDMA.c \
          Core/Src/gpio.c Core/Src/dma.c Core/Src/fdcan.c Core/Src/spi.c Core/Src/tim.c Core/Src/usart.c \
          Core/Src/stm32g4xx_hal_msp.c Core/custom_lib/src/bms_cmdlist.c Core/custom_lib/src/bms_libWrapper.c \
          Core/custom_lib/src/bms_mcuWrapper.c Core/custom_lib/src/bms_utility.c"
SIM="test/sim/sim_core.c test/sim/sim_can.c test/sim/sim_adbms.c test/sim/sim_ivt.c test/sim/sim_flash.c test/tests.c"

# Compiler flags that build with an override file instead of the car's configuration
override_flags() {
    echo "-I$(dirname "$1") -DBMS_LOCAL_CONFIG_FILE=\"$(basename "$1")\""
}

# build_tests <output folder> <extra flags>: compiles and links <folder>/bms_tests
build_tests() {
    local out="$1" extra="$2" objs="" obj
    mkdir -p "$out"
    for src in $FIRMWARE $SIM; do
        obj="$out/$(echo "$src" | tr '/' '_' | sed 's/\.c$/.o/')"
        $CC $CFLAGS $extra -c "$src" -o "$obj" || return 1
        objs="$objs $obj"
    done
    # main() and Error_Handler() are renamed so the simulator can drive and catch them
    obj="$out/Core_Src_main.o"
    $CC $CFLAGS $extra -Dmain=firmware_main -DError_Handler=Error_Handler_firmware -c Core/Src/main.c -o "$obj" || return 1
    $CC $objs "$obj" -o "$out/bms_tests" -lm
}

# expand_tests <binary> <spec...>: "all", "all except a b", or a list of names
expand_tests() {
    local bin="$1"; shift
    if [ "${1:-}" = "all" ]; then
        local skip=" ${*:3} "
        for t in $("$bin" --list | tr -d '\r'); do
            [[ "$skip" == *" $t "* ]] || echo "$t"
        done
    else
        echo "$@"
    fi
}

pass=0
fail=0
failed=""

# run_list <output folder> <label prefix> <tests...>
run_list() {
    local out="$1" label="$2" t rc start secs; shift 2
    for t in "$@"; do
        start=$(date +%s.%N)
        "$out/bms_tests" "$t" "$out" > "$out/$t.result.txt" 2>&1
        rc=$?
        secs=$(echo "$(date +%s.%N) - $start" | bc 2>/dev/null || echo "?")
        if [ $rc -eq 0 ]; then
            pass=$((pass + 1))
            printf "PASS  %-32s (%ss)\n" "$label$t" "${secs:0:4}"
        else
            fail=$((fail + 1))
            failed="$failed $label$t"
            printf "FAIL  %-32s (%ss)\n" "$label$t" "${secs:0:4}"
        fi
        sed 's/^/      /' "$out/$t.result.txt"
    done
}

# ---- The car's configuration (or BENCH_CONFIG) -------------------------------------------------

# The tests describe the car's configuration: a local bench override file is ignored unless asked for
if [ -n "${BENCH_CONFIG:-}" ]; then
    MAIN_FLAGS=$(override_flags "$BENCH_CONFIG")
    echo "Bench configuration: $BENCH_CONFIG"
else
    MAIN_FLAGS="-DBMS_IGNORE_LOCAL_CONFIG"
fi
build_tests "$OUT" "$MAIN_FLAGS" || exit 1

if [ $# -gt 0 ]; then
    TESTS="$*"
else
    TESTS=$("$OUT/bms_tests" --list | tr -d '\r')
fi
run_list "$OUT" "" $TESTS

# Register/command bitfield layout, for the ARM build and this simulation build
if bash test/check_layout.sh > "$OUT/layout.result.txt" 2>&1; then
    pass=$((pass + 1))
    echo "PASS  register_layout"
else
    fail=$((fail + 1))
    failed="$failed register_layout"
    echo "FAIL  register_layout"
fi
sed 's/^/      /' "$OUT/layout.result.txt" | grep -v " ok$"

# Decode the recorded CAN traffic with the generated DBC files (needs Python with cantools)
if [ -n "${BENCH_CONFIG:-}" ]; then
    echo "DBC check skipped (the DBCs describe the car's configuration, not $BENCH_CONFIG)"
elif [ -f "$OUT/can_layout.csv" ] && "$PYTHON" -c "import cantools" 2>/dev/null; then
    if "$PYTHON" test/check_dbc.py "$OUT/can_layout.csv"; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        failed="$failed dbc_check"
    fi
else
    echo "DBC check skipped (needs the can_layout test and a Python with cantools; set PYTHON=...)"
fi

# Firmware SoC curve against the Battery Tracker spreadsheet itself (needs Python with openpyxl)
SOC_BOOK="Supporting Docs/Battery Tracker.xlsx"
if [ -f "$OUT/soc_model.csv" ] && [ -f "$SOC_BOOK" ] && "$PYTHON" -c "import openpyxl" 2>/dev/null; then
    if "$PYTHON" test/check_soc.py "$OUT/soc_model.csv" "$SOC_BOOK"; then
        pass=$((pass + 1))
    else
        fail=$((fail + 1))
        failed="$failed soc_check"
    fi
else
    echo "SoC spreadsheet check skipped (needs the soc_model test, $SOC_BOOK and a Python with openpyxl)"
fi

# GUI (GUI/tests): SoC estimate against the firmware's curve, settings protocol, window with the demo BMS
if [ -z "${BENCH_CONFIG:-}" ] && [ $# -eq 0 ] && "$PYTHON" -c "import can, cantools, PySide6, pyqtgraph" 2>/dev/null; then
    if "$PYTHON" -m unittest discover -s GUI/tests > "$OUT/gui.result.txt" 2>&1; then
        pass=$((pass + 1))
        echo "PASS  gui ($(grep -o 'Ran [0-9]* tests' "$OUT/gui.result.txt"))"
    else
        fail=$((fail + 1))
        failed="$failed gui"
        echo "FAIL  gui"
        sed 's/^/      /' "$OUT/gui.result.txt"
    fi
else
    echo "GUI tests skipped (full runs only; need a Python with GUI/requirements.txt installed)"
fi

# ---- Configuration variants --------------------------------------------------------------------

if [ $# -eq 0 ] && [ -z "${BENCH_CONFIG:-}" ] && [ "${VARIANTS:-1}" != "0" ]; then
    for v in test/variants/*.h; do
        [ -f "$v" ] || continue
        name=$(basename "$v" .h)
        spec=$(sed -n 's#^// tests: ##p' "$v" | tr -d '\r')
        out="$OUT/variant_$name"
        echo
        echo "Variant $name ($v): $spec"
        if ! build_tests "$out" "$(override_flags "$v")"; then
            fail=$((fail + 1))
            failed="$failed $name:build"
            echo "FAIL  $name: build"
            continue
        fi
        run_list "$out" "$name:" $(expand_tests "$out/bms_tests" $spec)
    done
fi

echo
echo "$pass passed, $fail failed"
[ $fail -eq 0 ] || { echo "failed:$failed"; exit 1; }
