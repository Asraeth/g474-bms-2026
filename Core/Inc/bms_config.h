#ifndef INC_BMS_CONFIG_H_
#define INC_BMS_CONFIG_H_
#include "stm32g4xx_hal.h"
#include "profiles/bms_board_profile.h"
#include "config/bms_topology_config.h"
#include "config/bms_sensor_calibration.h"
#include "config/bms_safety_limits.h"
#include "config/bms_sensor_config.h"
#include "config/bms_vehicle_config.h"
#include "config/bms_bus_config.h"
#include "config/bms_protocol_config.h"
/* ===================================================================== */
/*  Local overrides (bench work)                                         */
/* ===================================================================== */
/*
 * Core/Inc/bms_config_local.h, if it exists, is included here and can #undef / #define anything
 * above, e.g. fewer slaves for eval boards on the bench (start from bms_config_local.h.example).
 * It is git-ignored, so it can't reach the car through the repository. While it exists every build
 * warns, the UART prints a banner at start-up and BMS_Status sets Local_Config_Overrides on CAN.
 * The generated DBCs and docs always describe this file without overrides.
 */
#if defined(BMS_LOCAL_CONFIG_FILE)
#include BMS_LOCAL_CONFIG_FILE                  // Test builds: test/run_tests.sh with BENCH_CONFIG=...
#define CFG_LOCAL_OVERRIDES         1
#elif !defined(BMS_IGNORE_LOCAL_CONFIG) && defined(__has_include)
#if __has_include("bms_config_local.h")
#include "bms_config_local.h"
#define CFG_LOCAL_OVERRIDES         1
#endif
#endif

#ifndef CFG_LOCAL_OVERRIDES
#define CFG_LOCAL_OVERRIDES         0
#else
#pragma message("Local BMS configuration selected; inspect profile before use")
#endif


/* ===================================================================== */
/*  Compile-time checks                                                  */
/* ===================================================================== */

_Static_assert(CFG_FAULT_DEADLINE_MS + CFG_SAFETY_TICK_MS + CFG_FS_MARGIN_MS <= CFG_FS_MAX_REACTION_MS,
               "Fault deadline leaves no margin within 500 ms");
_Static_assert(CFG_MEAS_STALE_TIMEOUT_MS <= CFG_FAULT_DEADLINE_MS, "isoSPI stale timeout beyond the fault deadline");
_Static_assert(CFG_IVT_FAULT_TIMEOUT_MS <= CFG_FAULT_DEADLINE_MS, "IVT fault timeout beyond the fault deadline");
_Static_assert(CFG_IVT_RESTART_WAIT_MS + CFG_MEAS_CYCLE_MAX_MS + 50 + 2 * CFG_IVT_CURRENT_PERIOD_MS < CFG_IVT_FAULT_TIMEOUT_MS,
               "A stopped IVT isn't running again (wait, a measurement cycle, ~40 ms configuration) before its data counts as lost");
_Static_assert(CFG_FAULT_DEBOUNCE_MS < CFG_FAULT_DEADLINE_MS, "Debounce longer than the fault deadline");
_Static_assert(CFG_FAULT_DECAY_DIVISOR >= 1, "Over-limit time can't drain faster than it builds up");
_Static_assert(CFG_BALANCE_HYSTERESIS_V >= 0.0f && CFG_BALANCE_HYSTERESIS_V < CFG_BALANCE_THRESHOLD_V,
               "Balancing hysteresis must be below the threshold");
_Static_assert(CFG_IVT_CURRENT_TIMEOUT_MS <= CFG_IVT_FAULT_TIMEOUT_MS && CFG_IVT_VOLTAGE_TIMEOUT_MS <= CFG_IVT_FAULT_TIMEOUT_MS,
               "IVT control timeouts longer than the fault timeout");
_Static_assert(CFG_COMMS_RETRY_DELAY_MS < CFG_MEAS_PERIOD_MS, "Comms retry delay longer than the measurement period");
_Static_assert(CFG_FAULT_DEBOUNCE_MS > CFG_MEAS_PERIOD_MS + CFG_MEAS_CYCLE_MAX_MS,
               "Debounce too short: a single bad measurement would trip the AMS");
_Static_assert(CFG_MEAS_STALE_TIMEOUT_MS > 2 * CFG_MEAS_PERIOD_MS + CFG_MEAS_CYCLE_MAX_MS,
               "Stale timeout too short: a single failed measurement cycle would trip the AMS");
_Static_assert(CFG_WATCHDOG_TIMEOUT_MS < CFG_FS_MAX_REACTION_MS, "Watchdog timeout too long");
_Static_assert(CFG_MEAS_PERIOD_MS < CFG_MEAS_STALE_TIMEOUT_MS, "Measurement period must be shorter than the stale timeout");
_Static_assert(CFG_MEAS_PERIOD_MS < CFG_WATCHDOG_TIMEOUT_MS, "Main loop period must be shorter than the watchdog");
_Static_assert(CFG_IVT_CURRENT_PERIOD_MS * 3 <= CFG_IVT_CURRENT_TIMEOUT_MS, "IVT current timeout allows < 3 messages");
_Static_assert(CFG_IVT_U1_PERIOD_MS * 3 <= CFG_IVT_VOLTAGE_TIMEOUT_MS, "IVT U1 timeout allows < 3 messages");
_Static_assert(CFG_IVT_U2_PERIOD_MS * 3 <= CFG_IVT_VOLTAGE_TIMEOUT_MS, "IVT U2 timeout allows < 3 messages");
_Static_assert(CFG_IVT_ENERGY_PERIOD_MS * 3 <= CFG_ENERGY_MAX_GAP_MS, "Energy gap allows < 3 messages");
_Static_assert(CFG_IVT_CURRENT_PERIOD_MS >= 1 && CFG_IVT_U1_PERIOD_MS >= 2 && CFG_IVT_U2_PERIOD_MS >= 2,
               "IVT-S minimum output periods: I 1 ms, U1/U2 2 ms");
_Static_assert((1000 + CFG_IVT_CURRENT_PERIOD_MS - 1) / CFG_IVT_CURRENT_PERIOD_MS +
               (1000 + CFG_IVT_U1_PERIOD_MS - 1) / CFG_IVT_U1_PERIOD_MS +
               (1000 + CFG_IVT_U2_PERIOD_MS - 1) / CFG_IVT_U2_PERIOD_MS +
               (1000 + CFG_IVT_ENERGY_PERIOD_MS - 1) / CFG_IVT_ENERGY_PERIOD_MS <= CFG_IVT_MAX_MESSAGES_PER_S,
               "IVT-S: more than 1000 result messages per second");
_Static_assert(CFG_PRECHARGE_TARGET_PERCENT >= 95 && CFG_PRECHARGE_TARGET_PERCENT < 100,
               "FS 2027 EV 5.7.1: precharge to at least 95 % before closing the second AIR");
_Static_assert(CFG_PRECHARGE_OVERLAP_MS < CFG_PRECHARGE_TIMEOUT_MS, "Overlap longer than the precharge timeout");
_Static_assert(CFG_IVT_ID_RESULT_I <= CFG_IVT_ID_RESULT_U1 && CFG_IVT_ID_RESULT_U1 < CFG_IVT_ID_RESULT_U2 &&
               CFG_IVT_ID_RESULT_U2 <= CFG_IVT_ID_RESULT_WH,
               "CAN2 filters: I..U1 to RX FIFO 0, U2..Wh to RX FIFO 1");
_Static_assert(CFG_CELLS_PER_SLAVE >= 1 && CFG_CELLS_PER_SLAVE <= 16, "ADBMS6830B monitors 1..16 cells");
_Static_assert(CFG_TEMPS_PER_SLAVE >= 1 && CFG_TEMPS_PER_SLAVE <= 10, "ADBMS6830B has 10 GPIOs");
_Static_assert(CFG_CAN_ID_CELL_V_BASE + CFG_NUM_SLAVES * CFG_CAN_V_FRAMES_PER_SLAVE <= CFG_CAN_ID_CELL_T_BASE, "Cell voltage CAN IDs overlap temperature IDs");
_Static_assert(CFG_CAN_ID_CELL_T_BASE + CFG_NUM_SLAVES * CFG_CAN_T_FRAMES_PER_SLAVE <= CFG_CAN_ID_SLAVE_BASE, "Temperature CAN IDs overlap slave IDs");
_Static_assert(CFG_CELLS_PER_SLAVE <= 8, "Slave CAN frame holds one flag byte per 8 cells");
_Static_assert(CFG_CAN_ID_SLAVE_BASE + CFG_NUM_SLAVES <= CFG_CAN_ID_PACK, "Slave CAN IDs overlap pack IDs");
_Static_assert(CFG_CAN_TX_PERIOD_MS % CFG_SAFETY_TICK_MS == 0, "CAN period must be a multiple of the tick");
_Static_assert(CFG_SAFETY_TICK_MS % CFG_TIMER_TICK_MS == 0, "Safety tick must be a multiple of the timer tick");
_Static_assert(CFG_CAN_FAST_PERIOD_MS % CFG_TIMER_TICK_MS == 0, "Fast CAN period must be a multiple of the timer tick");
_Static_assert(CFG_CAN_FAST_PERIOD_MS >= CFG_IVT_CURRENT_PERIOD_MS && CFG_CAN_FAST_PERIOD_MS >= CFG_IVT_U1_PERIOD_MS &&
               CFG_CAN_FAST_PERIOD_MS >= CFG_IVT_U2_PERIOD_MS, "Fast CAN frames faster than the IVT sends new values");
_Static_assert(CFG_CAN_ID_ENERGY < CFG_CAN_ID_FAST_1 && CFG_CAN_ID_FAST_1 < CFG_CAN_ID_FAST_2, "Fast frame IDs clash");
_Static_assert(CFG_CAN_ID_CELL_V_BASE > CFG_CAN1_INVERTER_ID_MAX && CFG_CAN_ID_TS_REQUEST > CFG_CAN1_INVERTER_ID_MAX,
               "BMS CAN1 IDs inside the inverter's (DTI packet << 8 | node) range");
_Static_assert((CFG_CAN_BURST_FRAMES * 1000 / CFG_CAN_TX_PERIOD_MS + CFG_CAN_FAST_FRAMES * 1000 / CFG_CAN_FAST_PERIOD_MS +
                (CFG_INVERTER_LIMITS_ENABLED * 2 + 1) * 1000 / CFG_INVERTER_LIMIT_PERIOD_MS +
                1000 / CFG_CAN_FAST_PERIOD_MS /* settings responses, worst case */) *
               CFG_CAN_FRAME_BITS_MAX <= CFG_CAN1_MAX_LOAD_PERCENT * (CFG_CAN1_BITRATE / 100),
               "BMS traffic would take more than CFG_CAN1_MAX_LOAD_PERCENT of CAN1");
_Static_assert(CFG_INVERTER_DERATE_START_C < CFG_INVERTER_DERATE_END_C, "Inverter derating starts above where it ends");
_Static_assert(CFG_INVERTER_DERATE_CURRENT_A >= 0 && CFG_INVERTER_DERATE_CURRENT_A <= CFG_MAX_DISCHARGE_CURRENT_A,
               "Inverter derating current outside 0..CFG_MAX_DISCHARGE_CURRENT_A");
_Static_assert(CFG_INVERTER_REGEN_DERATE_START_C < CFG_INVERTER_REGEN_DERATE_END_C, "Regen derating starts above where it ends");
_Static_assert(CFG_INVERTER_REGEN_DERATE_CURRENT_A >= 0 && CFG_INVERTER_REGEN_DERATE_CURRENT_A <= CFG_INVERTER_REGEN_CURRENT_A,
               "Regen derating current outside 0..CFG_INVERTER_REGEN_CURRENT_A");
_Static_assert(CFG_INVERTER_REGEN_CURRENT_A <= CFG_MAX_CHARGE_CURRENT_A, "Inverter regen limit above the AMS charge current trip");
_Static_assert(CFG_PACK_RESISTANCE_MIN_OHM >= 0 && CFG_PACK_RESISTANCE_MIN_OHM <= CFG_PACK_RESISTANCE_MAX_OHM,
               "Pack resistance range inverted");
_Static_assert(CFG_POWER_WINDOW_FLOOR_W > 0 && CFG_POWER_WINDOW_FLOOR_W < CFG_POWER_WINDOW_LIMIT_W,
               "Power window floor must be below the window limit");
_Static_assert(CFG_POWER_WINDOW_MS % CFG_TIMER_TICK_MS == 0 && CFG_POWER_WINDOW_LAG_MS % CFG_TIMER_TICK_MS == 0 &&
               CFG_POWER_WINDOW_SPEND_MS > 0, "Power window times must be multiples of the timer tick");
_Static_assert(CFG_INVERTER_NODE_ID >= 1 && CFG_INVERTER_NODE_ID <= 0xFF, "DTI extended node IDs are 1..254, 255 = broadcast");
// Limit frames go out half a fast period after the fast frames and BMS_Limits one tick later, so none of
// them compete for TX buffers
_Static_assert(CFG_INVERTER_LIMIT_PERIOD_MS % CFG_CAN_FAST_PERIOD_MS == 0 && CFG_CAN_FAST_PERIOD_MS / CFG_TIMER_TICK_MS >= 3,
               "Inverter limit period must be a multiple of the fast frame period (at least 3 ticks)");
_Static_assert(CFG_CAN_ID_FAST_2 < CFG_CAN_ID_LIMITS && CFG_CAN_ID_LIMITS < CFG_CAN_ID_CONFIG_RESPONSE &&
               CFG_CAN_ID_CONFIG_RESPONSE < CFG_CAN_ID_TS_REQUEST, "Limits or settings frame ID clashes");
_Static_assert(CFG_CAN_ID_CONFIG_REQUEST != CFG_CAN_ID_TS_REQUEST && CFG_CAN_ID_CONFIG_REQUEST > CFG_CAN1_INVERTER_ID_MAX,
               "Settings request ID clashes");
_Static_assert(CFG_SETTING_POWER_MIN_W > 0 && CFG_SETTING_POWER_MIN_W <= CFG_INVERTER_POWER_LIMIT_W,
               "Power limit setting range empty");
_Static_assert(CFG_SETTING_CURRENT_MIN_A > 0 && CFG_SETTING_CURRENT_MIN_A <= CFG_MAX_DISCHARGE_CURRENT_A,
               "Discharge current setting range empty");
_Static_assert(CFG_SETTING_TEMP_LIMIT_MIN_C > 0 && CFG_SETTING_TEMP_LIMIT_MIN_C <= CFG_INVERTER_DERATE_END_C,
               "Temperature limit setting range empty");

_Static_assert(CFG_CHARGER_CURRENT_A >= 0 && CFG_CHARGER_CURRENT_A <= CFG_CHARGER_CURRENT_MAX_A &&
               CFG_CHARGER_CURRENT_MAX_A <= CFG_MAX_CHARGE_CURRENT_A, "Charge current default or maximum out of range");
_Static_assert(CFG_CHARGER_VOLTAGE_V >= CFG_PACK_V_MIN && CFG_CHARGER_VOLTAGE_V <= CFG_CHARGER_VOLTAGE_MAX_V &&
               CFG_CHARGER_VOLTAGE_MAX_V < CFG_PACK_V_MAX, "Charge voltage default or maximum out of range");
_Static_assert(CFG_SETTINGS_FLASH_ADDR % 2048 == 0 && CFG_SETTINGS_FLASH_ADDR >= 0x0807F000 && CFG_SETTINGS_FLASH_ADDR < 0x08080000,
               "Settings page must be one of the last two 2 KB pages (kept free by the linker script)");


#endif
