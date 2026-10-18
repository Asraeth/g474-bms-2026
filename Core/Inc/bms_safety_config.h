/* BRING-UP CONFIGURATION: none of these safety values is hardware-approved.
 * Inherited values are retained explicitly; new timeouts are software budgets,
 * not claims about hardware timing. Resolve every listed category before use. */
#ifndef BMS_SAFETY_CONFIG_H
#define BMS_SAFETY_CONFIG_H
#include "bms_topology.h"

/* UNRESOLVED: chemistry/operating window. Preserve inherited cell limits. */
#define BMS_CELL_UV_MV 3300
#define BMS_CELL_OV_MV 4200
/* UNRESOLVED: retained ADBMS reset-register comparator codes. These are not
 * programmed from the software OV/UV limits and are not approved protection.
 * VUV/VOV are the chip's signed 12-bit encodings, not millivolts. */
#define BMS_ADBMS_HW_VUV_CODE 0x800U
#define BMS_ADBMS_HW_VOV_CODE 0x7FFU
#define BMS_ADBMS_CS_COMPARISON_CODE 1U
#define BMS_ADBMS_FILTER_CODE 1U /* Existing filter selection; no new policy. */
/* UNRESOLVED: aggregate UV differs from cell UV; do not silently unify them. */
#define BMS_AGGREGATE_UV_PER_POSITION_MV 3000
#define BMS_BOARD_UV_MV (BMS_AGGREGATE_UV_PER_POSITION_MV * TOTAL_CELL)
#define BMS_BOARD_OV_MV (BMS_CELL_OV_MV * TOTAL_CELL)
#define BMS_PACK_UV_MV (BMS_AGGREGATE_UV_PER_POSITION_MV * TOTAL_PACK_CELLS)
#define BMS_PACK_OV_MV (BMS_CELL_OV_MV * TOTAL_PACK_CELLS)
/* UNRESOLVED: vehicle/shunt rating and polarity convention. Signed limits. */
#define BMS_PACK_CURRENT_MIN_MA (-10000)
#define BMS_PACK_CURRENT_MAX_MA 10000
/* UNRESOLVED: sensor model, placement and thermal operating limits. */
#define BMS_TEMP_MIN_C 0.0f
#define BMS_TEMP_MAX_C 60.0f
#define BMS_DIE_TEMP_MIN_C 0.0f
#define BMS_DIE_TEMP_MAX_C 70.0f
/* UNRESOLVED: inherited software watchdogs; U1 output configuration unknown. */
#define BMS_IVT_CURRENT_TIMEOUT_MS 100U
#define BMS_IVT_U1_TIMEOUT_MS 300U
/* UNRESOLVED: new finite software budgets, to verify on the physical chain. */
#define BMS_ADBMS_SPI_TIMEOUT_MS 10U
#define BMS_ADBMS_POLL_TIMEOUT_MS 50U
#define BMS_ADBMS_CELL_MAX_AGE_MS 1000U
#define BMS_ADBMS_AUX_MAX_AGE_MS 1000U
/* Preserve existing cadence/delays; all remain subject to timing review. */
#define BMS_ACQUISITION_DELAY_MS 500U
#define BMS_SENSOR_STARTUP_DELAY_MS 500U
#define BMS_CHAIN_WAKE_PULSE_MS 1U
#define BMS_CELL_INITIAL_FILL_MS 12U
#define BMS_COMMS_RETRY_DELAY_MS 250U
#define BMS_COMMS_RETRY_ATTEMPTS 2U
#define BMS_VOLTAGE_FAULT_DELAY_MS 490U
#define BMS_TEMPERATURE_FAULT_DELAY_MS 990U
/* UNRESOLVED: resistance, thermal budget and allowable duty. Zero is the
 * bring-up interlock, not a newly approved balancing operating value. */
#define BMS_BALANCING_ENABLED_DEFAULT 0
#ifndef BMS_BALANCING_CONFIG_CONFIRMED
#define BMS_BALANCING_CONFIG_CONFIRMED 0
#endif
#define BMS_BALANCING_THRESHOLD_MV 30
#ifndef BMS_BALANCING_MAX_PWM
#define BMS_BALANCING_MAX_PWM 0U
#endif
#ifndef BMS_BALANCING_PWM_BUDGET
#define BMS_BALANCING_PWM_BUDGET 0U
#endif
#define BMS_BALANCING_TIMEOUT_MINUTES 1U /* Inherited, inactive during bring-up. */
/* UNRESOLVED: inherited charger settings/enable prerequisite. */
#define BMS_CHARGER_TARGET_MV 450000
#define BMS_CHARGER_MAX_CURRENT_MA 1000
#define BMS_CHARGER_MIN_OUTPUT_MV 300000

#define BMS_MV_TO_V(mv) ((float)(mv) / 1000.0f)
#define BMS_MA_TO_A(ma) ((float)(ma) / 1000.0f)

#if BMS_CELL_UV_MV >= BMS_CELL_OV_MV || BMS_PACK_CURRENT_MIN_MA >= BMS_PACK_CURRENT_MAX_MA
#error "Invalid safety limit ordering"
#endif
#if BMS_BALANCING_MAX_PWM > 15 || BMS_ADBMS_SPI_TIMEOUT_MS == 0 || BMS_ADBMS_POLL_TIMEOUT_MS == 0
#error "Invalid PWM cap or unbounded/zero communication timeout"
#endif
#if BMS_BALANCING_PWM_BUDGET > (BMS_BALANCING_MAX_PWM * TOTAL_CELL) || BMS_BALANCING_TIMEOUT_MINUTES > 63
#error "Invalid balancing budget or hardware discharge timer"
#endif
#endif
