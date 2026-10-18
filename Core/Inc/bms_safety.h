/*
 * bms_safety.h
 *
 * Fault evaluation, AMS fault output, TSAC ready relay and watchdog
 */

#ifndef INC_BMS_SAFETY_H_
#define INC_BMS_SAFETY_H_

#include <stdint.h>
#include <stdbool.h>

// Independent watchdog. Must be kicked from the main loop only.
void BMS_Watchdog_Start(void);
void BMS_Watchdog_Kick(void);

// Main loop: result of a complete isoSPI measurement cycle.
//   sampleTick     HAL tick when the cycle's conversions started
//   conditionFlags BMS_StatusTypeDef bits for limits violated in this sample
// (the IVT vs sum-of-cells plausibility is one of the conditionFlags, worked out from the same conversion)
void BMS_Safety_ReportMeasurement(uint32_t sampleTick, uint32_t conditionFlags);

// Every CFG_SAFETY_TICK_MS from the timer interrupt. Evaluates all faults and drives the outputs.
void BMS_Safety_Tick(void);

// Drive the outputs to the safe state immediately (error handlers)
void BMS_Safety_ForceFault(void);

// A change of the fault flags or the fault output, as the safety tick saw it
typedef struct
{
    uint32_t tick;          // HAL tick of the safety tick
    uint32_t flags;         // BMS_StatusTypeDef bits now active
    uint32_t prevFlags;     // ... before
    bool     faultOutput;   // Fault output now
} SafetyEvent;

// Main loop: the oldest change not yet taken, false if none. lost: changes dropped so far (queue full).
bool BMS_Safety_PopEvent(SafetyEvent* event, uint32_t* lost);

bool     BMS_Safety_IsFaultOutputActive(void);
uint32_t BMS_Safety_GetActiveFlags(void);
uint32_t BMS_Safety_GetMeasurementAgeMs(void);

#endif /* INC_BMS_SAFETY_H_ */
