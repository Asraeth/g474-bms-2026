/*
 * bms_precharge.h
 *
 * Precharge and bypass (AIR) control, started by the TS request on CAN1
 */

#ifndef INC_BMS_PRECHARGE_H_
#define INC_BMS_PRECHARGE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    PRECHARGE_IDLE     = 0,     // Both open
    PRECHARGE_CHARGING = 1,     // Precharge relay closed, waiting for U2 to reach the target
    PRECHARGE_CLOSING  = 2,     // Bypass closed, precharge relay still closed for the overlap
    PRECHARGE_ACTIVE   = 3,     // Bypass closed, precharge relay open: TS active
    PRECHARGE_FAILED   = 4,     // Timed out or lost the IVT voltages; both open until a new request
} PrechargeState;

// CAN1 receive interrupt: TS request frame, true = activate
void BMS_Precharge_HandleRequest(bool activate);

// Every CFG_SAFETY_TICK_MS from the timer interrupt, after BMS_Safety_Tick()
void BMS_Precharge_Tick(void);

PrechargeState BMS_Precharge_GetState(void);
uint8_t        BMS_Precharge_GetPercent(void);     // U2 / U1 in %, as last seen
bool           BMS_Precharge_IsTsActive(void);     // Any relay closed

#endif /* INC_BMS_PRECHARGE_H_ */
