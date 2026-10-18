/*
 * bms_soc.h
 *
 * State of charge and energy: open-circuit estimate at start-up, then the IVT-S energy counter
 */

#ifndef INC_BMS_SOC_H_
#define INC_BMS_SOC_H_

#include <stdint.h>
#include <stdbool.h>
#include "bms_ivt.h"

typedef struct
{
    bool  valid;                // Start-up estimate taken
    float soc_percent;          // Live: remaining energy / pack energy
    float initialSoc_percent;   // Open-circuit estimate at start-up
    float remaining_Wh;
    float used_Wh;              // Since the estimate, positive = discharged
} SocState;

// "Battery Tracker.xlsx" VToSoc model: open-circuit cell voltage -> SoC %, clamped to 0..100
float BMS_SoC_FromCellVoltage(float cellVoltage);

// Main loop, once per cycle.
//   meanCellVoltage  mean of the cells inside CFG_SOC_CELL_MIN_V..MAX_V, valid if meanValid
//   noLoad           no current can flow (TS off) or the current is negligible
void BMS_SoC_Update(float meanCellVoltage, bool meanValid, bool noLoad, const IvtReading* ivt);

// Consistent snapshot, safe from interrupts
SocState BMS_SoC_Get(void);

#endif /* INC_BMS_SOC_H_ */
