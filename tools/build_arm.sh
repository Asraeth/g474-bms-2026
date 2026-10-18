#!/usr/bin/env bash
#
# Command-line ARM build, mirroring the STM32CubeIDE project settings (.cproject).
# Lets the firmware be built and checked without the IDE, e.g. on a CI machine.
#
# Usage: tools/build_arm.sh [debug|release]      (default: debug)
#   ARM_GCC_BIN  directory holding arm-none-eabi-gcc (default: on PATH)
#   EXTRA_CFLAGS extra compiler flags, e.g. "-fanalyzer"
#
set -euo pipefail

cd "$(dirname "$0")/.."

CONFIG="${1:-debug}"
case "$CONFIG" in
    debug)   OPT="-O0 -g3" ;;
    release) OPT="-Os -g0" ;;
    *) echo "unknown config: $CONFIG" >&2; exit 2 ;;
esac

PREFIX="${ARM_GCC_BIN:+$ARM_GCC_BIN/}arm-none-eabi-"
CC="${PREFIX}gcc"
SIZE="${PREFIX}size"

OUT="build/$CONFIG"
mkdir -p "$OUT"

MCU="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
DEFS="-DUSE_HAL_DRIVER -DSTM32G474xx"
[ "$CONFIG" = debug ] && DEFS="$DEFS -DDEBUG"
INCS="-ICore/Inc -ICore/custom_lib/inc -IDrivers/STM32G4xx_HAL_Driver/Inc -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
      -IDrivers/CMSIS/Device/ST/STM32G4xx/Include -IDrivers/CMSIS/Include"
CFLAGS="$MCU $DEFS $INCS $OPT -std=gnu11 -ffunction-sections -fdata-sections -fstack-usage -Wall ${EXTRA_CFLAGS:-}"

# Stricter warnings for the project's own code only
OWN_WARN="-Wextra -Wshadow -Wformat=2 -Wno-unused-parameter -Wno-old-style-declaration"

SOURCES=$(ls Core/Src/*.c Core/custom_lib/src/*.c Drivers/STM32G4xx_HAL_Driver/Src/*.c)
OBJS=""
for src in $SOURCES; do
    obj="$OUT/$(echo "$src" | tr '/' '_' | sed 's/\.c$/.o/')"
    flags="$CFLAGS"
    case "$src" in Core/*) flags="$flags $OWN_WARN" ;; esac
    $CC $flags -c "$src" -o "$obj"
    OBJS="$OBJS $obj"
done

$CC $MCU -x assembler-with-cpp -c Core/Startup/startup_stm32g474retx.s -o "$OUT/startup.o"
OBJS="$OBJS $OUT/startup.o"

$CC $MCU -T STM32G474RETX_FLASH.ld --specs=nosys.specs --specs=nano.specs -u _printf_float -u _scanf_float \
    -Wl,-Map="$OUT/g474-bms.map" -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group \
    $OBJS -o "$OUT/g474-bms.elf"

$SIZE "$OUT/g474-bms.elf"
