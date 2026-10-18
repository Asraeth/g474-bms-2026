/* The former excluded random floor regression is mandatory. */
// tests: boot fast_frames power_window_sustained power_window_bursts power_window_random load_swings
#undef CFG_POWER_WINDOW_LIMITER_ENABLED
#define CFG_POWER_WINDOW_LIMITER_ENABLED 1
