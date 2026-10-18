/*
 * bms_inverter.h
 *
 * DC current limits the BMS sends to the inverter (DTI HV500) on CAN1
 */

#ifndef INC_BMS_INVERTER_H_
#define INC_BMS_INVERTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "bms_can.h"
#include "bms_settings.h"

#define INVERTER_LIMIT_FRAME_COUNT 2

typedef struct
{
    float discharge_A;      // Max. DC current (drive), >= 0
    float charge_A;         // Max. DC brake current (regen), >= 0 (sent negative)
    float dischargePower_W; // discharge_A x the pack voltage used
    bool  tempDerating;     // Discharge: hottest cell above CFG_INVERTER_DERATE_START_C
    bool  powerLimited;     // Discharge held to CFG_INVERTER_POWER_LIMIT_W
    bool  regenDerating;    // Regen: hottest cell above CFG_INVERTER_REGEN_DERATE_START_C
    bool  boost;            // Moving-window limiter allowing more than CFG_POWER_WINDOW_LIMIT_W
    bool  faulted;          // AMS fault: 0 A both ways
    bool  settingsModified; // Runtime settings differ from the built-in limits
    float windowAvg_W;      // 500 ms moving average of the IVT's discharge power
} InverterLimits;

// Pure calculation, for the tests as well. s: runtime settings (power limit, currents, temperature limit,
// regen). powerCap_W: CFG_INVERTER_POWER_LIMIT_W, or what the moving-window limiter allows; the lower of it
// and the power limit setting applies. packCurrent_A: present discharge current (for the voltage recovery).
InverterLimits BMS_Inverter_ComputeLimits(const BmsSettings* s, float maxCellTemp_C, float packVoltage_V,
                                          float packCurrent_A, bool faulted, float powerCap_W);

// Timer interrupt, every CFG_TIMER_TICK_MS: records the IVT's power for the 500 ms moving average
void BMS_Inverter_Tick(void);

// Timer interrupt, every CFG_INVERTER_LIMIT_PERIOD_MS: works out the limits from the latest measurements
void BMS_Inverter_Update(void);

// The two DTI frames for the limits last worked out
uint32_t BMS_Inverter_GetCanFrames(CanTxMsg msgs[INVERTER_LIMIT_FRAME_COUNT]);

// BMS_Limits frame (CAN1, CFG_CAN_ID_LIMITS) for the limits last worked out
void BMS_Inverter_GetLimitsFrame(CanTxMsg* msg);

// Limits last sent
InverterLimits BMS_Inverter_GetLimits(void);

#endif /* INC_BMS_INVERTER_H_ */
