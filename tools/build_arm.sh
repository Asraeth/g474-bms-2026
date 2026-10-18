#!/usr/bin/env bash
# Compatibility entrypoint for strict, profile-aware ARM builds.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ -n "${ARM_GCC_BIN:-}" ]]; then export ARM_GCC="$ARM_GCC_BIN/arm-none-eabi-gcc"; fi
exec "${PYTHON:-python3}" tools/build_arm.py "${1:-debug}" --profile "${BMS_BOARD_PROFILE:-3}"
