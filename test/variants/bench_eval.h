/*
 * Configuration variant for test/run_tests.sh: the bench profile for EVAL-ADBMS6830BMSW boards.
 * Tests that need more than one slave (balancing between slaves, LEDs on slave 3), run for 10 minutes, or
 * drive the car's pack at 80 kW (an 8-cell pack would need thousands of amps) are left out.
 */
// tests: all except balancing leds soak_1 soak_2 soak_3 power_window_sustained power_window_bursts power_window_random load_swings

#include "bms_config_local.h.example"
