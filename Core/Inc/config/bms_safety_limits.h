#pragma once

/* ===================================================================== */
/*  Cell limits (Sony/Murata VTC6 via Enepaq datasheet)                  */
/* ===================================================================== */

#define CFG_CELL_UV_V               2.50f
#define CFG_CELL_OV_V               4.20f

// Discharge window. FS 2027 EV 5.8.4: never above 60 C.
#define CFG_CELL_TEMP_MIN_C         (-20.0f)
#define CFG_CELL_TEMP_MAX_C         60.0f

// Charge window, used instead of the discharge window while charging is enabled.
#define CFG_CELL_CHARGE_TEMP_MIN_C  0.0f
#define CFG_CELL_CHARGE_TEMP_MAX_C  45.0f

// Slave board limits
#define CFG_SLAVE_DIE_TEMP_MAX_C    70.0f
#define CFG_SLAVE_DIE_TEMP_MIN_C    (-40.0f)
#define CFG_SLAVE_V_MAX             (CFG_CELL_OV_V * CFG_CELLS_PER_SLAVE)
#define CFG_SLAVE_V_MIN             (CFG_CELL_UV_V * CFG_CELLS_PER_SLAVE)



/* ===================================================================== */
/*  Pack current and voltage (IVT-S)                                     */
/* ===================================================================== */

#define CFG_MAX_DISCHARGE_CURRENT_A 200.0f
// VERIFY: charge/regen limit should be the cell datasheet charge limit x parallel count.
#define CFG_MAX_CHARGE_CURRENT_A    35.0f

#define CFG_PACK_V_MAX              (CFG_CELL_OV_V * CFG_TOTAL_CELLS)
#define CFG_PACK_V_MIN              (CFG_CELL_UV_V * CFG_TOTAL_CELLS)

// IVT U1 vs. sum of cell voltages. Larger difference = implausible measurement (T 11.9).
// Allows for busbar drops under load. 0 disables the check.
// U1 is averaged over the same cell conversion; conversions during which the IVT current moved more than
// CFG_PACK_V_PLAUSIBILITY_MAX_DI_A aren't used (the pack voltage moves with the current: 0.5 R x 20 A = 10 V).
#define CFG_PACK_V_PLAUSIBILITY_V   20.0f
#define CFG_PACK_V_PLAUSIBILITY_MAX_DI_A 20.0f



/* ===================================================================== */
/*  Fault timing                                                         */
/* ===================================================================== */
/*
 * FS 2027 EV 5.8.10: the SDC must open if a critical voltage or current value persists for
 * more than 500 ms (1 s for temperature). T 11.9: message timeouts must not exceed 500 ms.
 * Everything here is held to 500 ms, less CFG_FS_MARGIN_MS for the latch and relays.
 *
 * The deadline: a value last seen good CFG_FAULT_DEADLINE_MS ago, and out of limits or missing since,
 * is faulted on that safety tick, so the fault output follows within the deadline + one safety tick:
 *   - missing data (stale isoSPI measurements, IVT timeouts) faults after its timeout (<= deadline)
 *   - a violation faults once it has been over the limit for CFG_FAULT_DEBOUNCE_MS, timed from the
 *     sample that showed it, or at the deadline from the last sample that was good, whichever is first
 *     (so a late sample can't add the debounce on top of its gap). Time back under the limit only
 *     counts the over-limit time back at 1/CFG_FAULT_DECAY_DIVISOR, so noise around a limit or pulses
 *     over it fault too (over the limit half the time: within ~2 x the debounce)
 *   - a reading that doesn't arrive doesn't clear a pending violation
 * That leaves the time up to the deadline for riding through brief losses of data: an IVT restart
 * (~240 ms on the bench) or an isoSPI glitch (failed cycles are retried every CFG_COMMS_RETRY_DELAY_MS
 * meanwhile), without the AMS tripping.
 *
 * Nuisance trips are avoided as well:
 *   - A single bad sample counts as over the limit until the next sample reports (period + cycle time
 *     later). Debounce > period + cycle time means it never trips, and its time drains away again.
 *   - Stale timeout > 2 x period + cycle time means one failed cycle (e.g. a PEC error) is
 *     tolerated.
 *
 * isoSPI measurement cycle with 14 slaves at 2 MHz SPI (ADBMS6830B timings): ~41 ms in simulation,
 * ~47 ms with the datasheet's slowest conversion times:
 *   wake-up 14 x 0.62 ms              ~  9 ms
 *   3 x S-ADC single shot (8 ms each) ~ 24 ms  (normal + even + odd open wire)
 *   AUX conversion (all channels)       18 ms, runs at the same time as the S-ADC conversions
 *   ~16 register reads/writes, 116 bytes each at 2 MHz ~ 8 ms
 * Measure it on the hardware: the firmware warns when a cycle takes longer than CFG_MEAS_CYCLE_MAX_MS.
 */

#define CFG_FS_MAX_REACTION_MS      500
#define CFG_FS_MARGIN_MS            20      // Kept for the fault output, latch and relays to act

#define CFG_TIMER_TICK_MS           1       // TIM16 period, set in g474-bms.ioc (checked by tools/generate.py)
#define CFG_SAFETY_TICK_MS          10      // Every 10th timer tick: fault evaluation, relays, CAN burst
#define CFG_FAULT_DEADLINE_MS       (CFG_FS_MAX_REACTION_MS - CFG_FS_MARGIN_MS - CFG_SAFETY_TICK_MS)
#define CFG_MEAS_PERIOD_MS          100     // Target period of the isoSPI measurement cycle
#define CFG_MEAS_CYCLE_MAX_MS       70      // Longest a measurement cycle may take
#define CFG_MEAS_STALE_TIMEOUT_MS   450     // No complete measurement for this long = fault
#define CFG_FAULT_DEBOUNCE_MS       180     // A limit violation must persist this long before faulting
#define CFG_FAULT_DECAY_DIVISOR     2       // Time under a limit counts the over-limit time back at 1/2
#define CFG_FAULT_CLEAR_DELAY_MS    1000    // All clear for this long before releasing the fault output
#define CFG_COMMS_RETRIES           3       // Failed cycles in a row before the chain is reset and re-initialised
#define CFG_COMMS_RETRY_DELAY_MS    10      // A failed cycle is retried this soon while the data isn't stale yet
#define CFG_CHECK_COMMAND_COUNTER   1       // Every isoSPI read checks the devices' command counters (lost commands)

#define CFG_WATCHDOG_TIMEOUT_MS     200     // IWDG; main loop must kick it more often than this



/* ===================================================================== */
/*  isoSPI / ADBMS6830B timing                                           */
/* ===================================================================== */

#define CFG_ISOSPI_WAKE_PULSE_US    20      // CS low time of each wake-up pulse
#define CFG_ISOSPI_WAKE_DELAY_US    600     // Datasheet: 500 us <= wait < tIDLE (4.3 ms min)
#define CFG_SPI_TIMEOUT_MS          10
#define CFG_SADC_POLL_TIMEOUT_MS    15      // S-ADC single shot: 8 ms
#define CFG_AUX_POLL_TIMEOUT_MS     30      // AUX all channels: ~18 ms



/* ===================================================================== */
/*  Open wire detection (ADBMS6830B "Cell Open Wire Detection")          */
/* ===================================================================== */

#define CFG_OPEN_WIRE_CHECK_ENABLED 1

// With the open wire switch on, an intact input reads 85..95 % of the normal reading
// (datasheet: gain during open wire detection). A broken wire collapses towards 0 V.
// VERIFY on the bench: the intact ratio depends on the slave input filter.
#define CFG_OPEN_WIRE_MIN_RATIO     0.75f

#define CFG_ADSV_OW_EVEN            0b01    // OW[1:0] = 01: even channels on
#define CFG_ADSV_OW_ODD             0b10    // OW[1:0] = 10: odd channels on



/* ===================================================================== */
/*  Balancing                                                            */
/* ===================================================================== */

#define CFG_BALANCING_ENABLED           BMS_CAN_BALANCE
#define CFG_BALANCE_ONLY_WHILE_CHARGING 0
#define CFG_BALANCE_THRESHOLD_V         0.030f  // Balance cells more than this above the lowest cell
#define CFG_BALANCE_HYSTERESIS_V        0.010f  // ... and keep balancing until within THRESHOLD - HYSTERESIS
#define CFG_BALANCE_RESISTOR_OHM        BMS_PROFILE_BALANCE_RESISTOR_OHM
// Heat budget per slave board. Duty cycles are scaled so that all discharging cells together stay
// below this. VERIFY against the slave board thermal design.
#define CFG_BALANCE_MAX_POWER_PER_SLAVE_W 4.0f
#define CFG_BALANCE_DCTO_MIN            1       // Slave discharge timeout (minutes) if the BMS stops talking



#define CFG_BALANCE_MAX_PWM 15U
#define CFG_CHARGER_STATUS_TIMEOUT_MS 450U
