#!/usr/bin/env bash
# Supported compatibility runner. No safety test exclusion lists.
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-python3}"
flags=()
[[ "${SANITIZE:-0}" == 1 ]] && flags+=(--sanitize)
[[ "${SIM_PESSIMISTIC:-0}" == 1 ]] && flags+=(--pessimistic)
[[ -n "${TICK_WRAP:-}" ]] && flags+=(--wrap)
"$PYTHON" tools/generate.py --check
"$PYTHON" tools/run_digital.py "${flags[@]}" "$@"
if [[ $# == 0 ]]; then
 "$PYTHON" tools/run_digital.py "${flags[@]}" --variant test/variants/power_window.h boot fast_frames power_window_sustained power_window_bursts power_window_random load_swings limiter_boundaries
 "$PYTHON" tools/run_migrated_contracts.py $([[ "${SANITIZE:-0}" == 1 ]] && echo --sanitize || true)
 "$PYTHON" tools/run_digital.py --profile 3 bench_contract
fi
