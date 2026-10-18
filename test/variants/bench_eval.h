/* EVAL single-board verification uses the explicit BENCH profile:
 * python tools/run_digital.py --profile 3 bench_contract
 * The full vehicle suite always runs with the 14-board digital profile.
 * No safety scenario is excluded from that suite.
 */
#error "Select BMS_BOARD_PROFILE=3 with --profile 3; do not override vehicle topology piecemeal"
