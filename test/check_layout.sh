#!/usr/bin/env bash
#
# Checks that the ADBMS6830B register/command bitfield structs produce the datasheet byte layout,
# both for the ARM compiler (the real firmware) and the PC compiler (the simulation tests).
#
# Usage: test/check_layout.sh       (ARM_GCC_BIN = folder with arm-none-eabi-gcc, if not on PATH)
#
set -uo pipefail
cd "$(dirname "$0")/.."

OUT=test/build/layout
mkdir -p "$OUT"
INCS="-ICore/custom_lib/inc"

# Expected bytes from the ADBMS6830B datasheet (Tables 18, 23, 55, 56, 95, 96) for the values in layout_probe.c
declare -A EXPECTED=(
    [probe_cfa]="85a5985a0216"
    [probe_cfb]="bc3a12aa0180"
    [probe_pwma]="21436587a9cb"
    [probe_pwmb]="ed1f00000000"
    [probe_adcv]="03f6"
    [probe_adsv]="01e9"
    [probe_adax]="05da"
)
SYMBOLS="probe_cfa probe_cfb probe_pwma probe_pwmb probe_adcv probe_adsv probe_adax"

fail=0
compare() {
    local who="$1" sym="$2" got="$3"
    local want="${EXPECTED[$sym]}"
    got="${got:0:${#want}}"     # Registers are sent as 6 bytes, commands as 2
    if [ "$got" = "$want" ]; then
        printf "  %-5s %-11s %s  ok\n" "$who" "$sym" "$got"
    else
        printf "  %-5s %-11s %s  EXPECTED %s\n" "$who" "$sym" "$got" "$want"
        fail=1
    fi
}

# ARM: read the initialised bytes out of the object file
ARM_PREFIX="${ARM_GCC_BIN:+$ARM_GCC_BIN/}arm-none-eabi-"
if "${ARM_PREFIX}gcc" --version > /dev/null 2>&1; then
    "${ARM_PREFIX}gcc" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -O2 -fdata-sections \
        $INCS -c test/layout_probe.c -o "$OUT/probe_arm.o" || exit 1
    for sym in $SYMBOLS; do
        "${ARM_PREFIX}objcopy" -O binary --only-section=".rodata.$sym" "$OUT/probe_arm.o" "$OUT/$sym.arm.bin"
        compare ARM "$sym" "$(od -An -v -tx1 "$OUT/$sym.arm.bin" | tr -d ' \n')"
    done
else
    echo "  arm-none-eabi-gcc not found, ARM layout not checked (set ARM_GCC_BIN)"
fi

# PC (simulation build): print the bytes from a small program
cat > "$OUT/dump_host.c" <<'EOF'
#include <stdio.h>
#include "bms_datatypes.h"
#include "bms_cmdlist.h"
extern const ad68_cfa_t probe_cfa; extern const ad68_cfb_t probe_cfb; extern const ad68_pwma_t probe_pwma;
extern const ad68_pwmb_t probe_pwmb; extern const ADCV_t probe_adcv; extern const ADSV_t probe_adsv; extern const ADAX_t probe_adax;
static void dump(const char* name, const void* p, size_t n)
{
    printf("%s ", name);
    for (size_t i = 0; i < n; i++) printf("%02x", ((const unsigned char*)p)[i]);
    printf("\n");
}
int main(void)
{
    dump("probe_cfa", &probe_cfa, sizeof probe_cfa);   dump("probe_cfb", &probe_cfb, sizeof probe_cfb);
    dump("probe_pwma", &probe_pwma, sizeof probe_pwma); dump("probe_pwmb", &probe_pwmb, sizeof probe_pwmb);
    dump("probe_adcv", &probe_adcv, sizeof probe_adcv); dump("probe_adsv", &probe_adsv, sizeof probe_adsv);
    dump("probe_adax", &probe_adax, sizeof probe_adax);
    return 0;
}
EOF
if ${CC:-gcc} -std=gnu11 $INCS test/layout_probe.c "$OUT/dump_host.c" -o "$OUT/dump_host"; then
    while read -r sym bytes; do compare PC "$sym" "$bytes"; done < <("$OUT/dump_host" | tr -d '\r')
else
    fail=1
fi

[ $fail -eq 0 ] && echo "Register layout check: PASS" || { echo "Register layout check: FAIL"; exit 1; }
