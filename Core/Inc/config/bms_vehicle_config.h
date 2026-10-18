#pragma once

/* ===================================================================== */
/*  Precharge                                                            */
/* ===================================================================== */
/*
 * A TS activate request on CAN1 closes the precharge relay. When the vehicle side of the AIRs
 * (IVT U2) reaches CFG_PRECHARGE_TARGET_PERCENT of the battery voltage (IVT U1), the bypass (AIR)
 * closes and the precharge relay opens. Everything opens on an AMS fault, SDC open, lost IVT
 * voltages, a deactivate request or a precharge timeout.
 */
#define CFG_PRECHARGE_TARGET_PERCENT 95     // FS 2027 EV 5.7.1: at least 95 %
#define CFG_PRECHARGE_TIMEOUT_MS    5000    // VERIFY: > 3 x R_precharge x C_DC-link, with margin
#define CFG_PRECHARGE_OVERLAP_MS    50      // Bypass closes this long before precharge opens; 0 = both at once

// TS request from the VCU. Address TBD. Byte 0: 1 = activate, 0 = deactivate.
// Precharge starts when the request changes from 0 to 1 (the first activate after start-up counts),
// so a VCU repeating "activate" can't restart a failed precharge over and over.
#define CFG_CAN_ID_TS_REQUEST       0xB300  // TBD
#define CFG_CAN_TS_REQUEST_EXTENDED 1
// Runtime settings request from the GUI (extended ID). The BMS answers with CFG_CAN_ID_CONFIG_RESPONSE.
#define CFG_CAN_ID_CONFIG_REQUEST   0xB301
// Deactivate when no request frame has arrived for this long. 0 = off (one activate frame is enough).
#define CFG_TS_REQUEST_TIMEOUT_MS   0



/* ===================================================================== */
/*  Inverter current limits (DTI HV500 on CAN1)                          */
/* ===================================================================== */
/*
 * The BMS owns the inverter's DC current limits (DTI Inverter CAN Manual v2.4: 0x0A "Set max. DC
 * current", 0x0B "Set max. DC brake current"), sent every CFG_INVERTER_LIMIT_PERIOD_MS:
 *   discharge = min(CFG_MAX_DISCHARGE_CURRENT_A, CFG_INVERTER_POWER_LIMIT_W / pack voltage): with 200 A
 *               and 80 kW the current starts falling above 400 V (FS 2027 EV 2.2.1). Falls linearly
 *               to CFG_INVERTER_DERATE_CURRENT_A as the hottest cell goes from
 *               CFG_INVERTER_DERATE_START_C to CFG_INVERTER_DERATE_END_C.
 *   regen     = CFG_INVERTER_REGEN_CURRENT_A, no power limit (EV 2.2.3). Falls linearly to
 *               CFG_INVERTER_REGEN_DERATE_CURRENT_A between the REGEN_DERATE temperatures.
 *   both 0 A while the AMS is faulted.
 * BMS_Limits (CFG_CAN_ID_LIMITS) reports the live limits, including the discharge power limit
 * (current limit x pack voltage).
 * The inverter limits its own DC current measurement; leave margin in the power limit if needed.
 * DTI CAN Tool: "Drive enable via CAN2" must be on, or the inverter ignores CAN commands.
 */
#define CFG_INVERTER_LIMITS_ENABLED     1
#define CFG_INVERTER_NODE_ID            0xFF    // TBD. 0xFF = DTI broadcast: every DTI inverter on the bus applies it
#define CFG_INVERTER_LIMIT_PERIOD_MS    20
#define CFG_INVERTER_POWER_LIMIT_W      80000.0f
#define CFG_INVERTER_DERATE_START_C     55.0f
#define CFG_INVERTER_DERATE_END_C       60.0f
#define CFG_INVERTER_DERATE_CURRENT_A   25.0f
// Regen: the cells' fast-charge rating (Enepaq/VTC6: 5 A per cell, 35 A for 7p), reduced gently from the
// top of their charge window (45 C). Below the AMS charge trip (CFG_MAX_CHARGE_CURRENT_A), so an
// overshoot past the inverter limit doesn't trip the AMS.
#define CFG_INVERTER_REGEN_CURRENT_A          35.0f
#define CFG_INVERTER_REGEN_DERATE_START_C     45.0f
#define CFG_INVERTER_REGEN_DERATE_END_C       60.0f
#define CFG_INVERTER_REGEN_DERATE_CURRENT_A   25.0f

/* Conservative power ceiling with a measured 500 ms diagnostic window.
 * The former credit-spending boost caused a repeated floor undershoot. Both
 * settings now obey the instantaneous ceiling; the random floor regression
 * remains mandatory. No unqualified above-ceiling boost is shipped.
 */
#define CFG_POWER_WINDOW_LIMITER_ENABLED  0
#define CFG_POWER_WINDOW_MS               500     // FS 2027 D 10.4.1
#define CFG_POWER_WINDOW_LIMIT_W          CFG_INVERTER_POWER_LIMIT_W  // Lower it for margin to the logger
#define CFG_POWER_WINDOW_FLOOR_W          72000.0f    // Higher = shallower dip after a boost, shorter boost
#define CFG_POWER_WINDOW_LAG_MS           10      // VERIFY: time the inverter takes to apply a new limit
#define CFG_POWER_WINDOW_SPEND_MS         200     // Unused energy is spent over about this long

// Pack resistance range (cells, busbars, fuse, AIRs). A power limit becomes a current limit at the present
// pack voltage, which moves with the current: below the present current it recovers (the highest
// resistance is assumed, so the power can't overshoot), above it it sags (the lowest is assumed, so the
// power still can't overshoot but gets close). 7p VTC6 modules alone are ~0.27 R (datasheet impedance).
// VERIFY against the sag in logged runs (BMS_Fast_1: IVT current and U1).
#define CFG_PACK_RESISTANCE_MIN_OHM       0.15f
#define CFG_PACK_RESISTANCE_MAX_OHM       0.6f



/* ===================================================================== */
/*  Runtime settings (GUI over CAN1)                                     */
/* ===================================================================== */
/*
 * The GUI (GUI/bms_gui.py) reads and changes these with BMS_Config_Request on CAN1; the BMS answers
 * with BMS_Config_Response (README.md, "Runtime settings"). A change applies at once; "save" stores the
 * settings in flash (last page, kept out of the program by the linker script) and they're loaded at
 * every start-up. Saving is refused while the TS is active or charging.
 *
 * The inverter limits can only be lowered below the values built in above, never raised past them:
 *   Power limit             CFG_SETTING_POWER_MIN_W .. CFG_INVERTER_POWER_LIMIT_W
 *   Discharge current limit CFG_SETTING_CURRENT_MIN_A .. CFG_MAX_DISCHARGE_CURRENT_A
 *   Temperature limit       hottest cell at which both inverter limits are fully derated. Lowering it moves the
 *                           discharge and regen derating ramps down by the same amount.
 *                           CFG_SETTING_TEMP_LIMIT_MIN_C .. CFG_INVERTER_DERATE_END_C
 *   Regen                   on / off (off: 0 A brake current)
 *   Regen current limit     0 .. CFG_INVERTER_REGEN_CURRENT_A
 * Charger setpoints, inside the cell limits:
 *   Charge current          0 .. CFG_CHARGER_CURRENT_MAX_A, default CFG_CHARGER_CURRENT_A
 *   Charge voltage          CFG_PACK_V_MIN .. CFG_CHARGER_VOLTAGE_MAX_V, default CFG_CHARGER_VOLTAGE_V
 * The AMS trip limits (cell voltages and temperatures, pack current) are not settings: they stay as built.
 * Out-of-range writes are refused. Stored values outside a new build's limits fall back to the default.
 */
#define CFG_SETTING_POWER_MIN_W       1000.0f
#define CFG_SETTING_CURRENT_MIN_A     1.0f
#define CFG_SETTING_TEMP_LIMIT_MIN_C  30.0f

#define CFG_CHARGER_CURRENT_A         1.0f    // VERIFY: default charge current (the old hard-coded value)
#define CFG_CHARGER_CURRENT_MAX_A     9.9f   // VERIFY: cells' max charge current (VTC6 5 A x 7p) and the charger's rating
#define CFG_CHARGER_VOLTAGE_V         (CFG_TOTAL_CELLS * 4.02f)     // Default: 4.02 V per cell (450 V for 112 cells)
#define CFG_CHARGER_VOLTAGE_MAX_V     (CFG_TOTAL_CELLS * (CFG_CELL_OV_V - 0.05f))   // 50 mV per cell below the OV trip

// Last 2 KB of the 512 KB flash. The linker script keeps the last 4 KB out of the program, so the page can
// be erased in either bank mode (2 KB pages dual-bank, 4 KB single-bank).
#define CFG_SETTINGS_FLASH_ADDR       0x0807F800
#define CFG_SETTINGS_SAVE_INTERVAL_MS 2000    // At most one settings page erase this often (flash wear)



/* ===================================================================== */
/*  Debug output (UART)                                                  */
/* ===================================================================== */
#define CFG_DEBUG_PRINT_LOOP        1       // Every loop: tick, loop time, active fault flags
#define CFG_DEBUG_PRINT_CELLS       0       // Every cycle: cell voltages, slave sums, open wire results (bring-up)
#define CFG_DEBUG_PRINT_TEMPS       0       // Every cycle: sensor temperatures, die temperature, slave voltage (bring-up)



/* Lead TIM15 clock assumption; verify prescaler/clock on the selected MCU profile. */
#define CFG_IMD_TIMER_CLOCK_HZ 128000000U
