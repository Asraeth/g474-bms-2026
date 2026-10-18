/*
 * bms_ivt.h
 *
 * Isabellenhuette IVT-S current / voltage sensor on CAN2
 */

#ifndef INC_BMS_IVT_H_
#define INC_BMS_IVT_H_

#include <stdint.h>
#include <stdbool.h>
#include "ivt_s.h"

typedef struct
{
    // As the IVT reports them (1 mA, 1 mV), current with the sign convention below applied
    int32_t  current_mA;
    int32_t  voltage_mV;
    int32_t  voltage2_mV;

    float    current_A;         // Positive = discharge
    float    voltage_V;         // U1, battery side of the AIRs (pack voltage)
    float    voltage2_V;        // U2, vehicle side of the AIRs
    float    power_W;           // I x U1, positive = discharge. Valid if powerFresh.
    int32_t  energy_Wh;         // IVT energy counter, counts up on discharge
    uint32_t energyTick;        // HAL tick of the latest energy counter value

    bool     currentFresh;      // Received within CFG_IVT_CURRENT_TIMEOUT_MS
    bool     voltageFresh;      // Received within CFG_IVT_VOLTAGE_TIMEOUT_MS
    bool     voltage2Fresh;
    bool     powerFresh;        // currentFresh && voltageFresh
    bool     energyFresh;       // Received within 3 x CFG_IVT_ENERGY_PERIOD_MS
    bool currentValid, voltageValid, voltage2Valid, energyValid, configurationVerified;
    uint32_t restartEpoch;
    bool     energyReceived;    // At least one energy counter value since start-up
    bool     sensorError;       // Status bits in CFG_IVT_STATUS_FAULT_MASK set on I or U1
    bool     systemError;       // Status bit 3 (system error) set on I or U1
    bool     voltage2Error;     // Same, on U2

    // For the safety evaluation: HAL tick of the latest current / U1 value, and its age (UINT32_MAX if none
    // has arrived yet). Missing for CFG_IVT_FAULT_TIMEOUT_MS is a fault.
    uint32_t currentTick;
    uint32_t voltageTick;
    uint32_t currentAge_ms;
    uint32_t voltageAge_ms;
} IvtReading;

void BMS_IVT_InvalidateTransport(void);
void BMS_IVT_GetSnapshot(IVTS_Snapshot *snapshot);

// Called from the CAN2 receive interrupt
void BMS_IVT_HandleFrame(uint32_t id, const uint8_t* data, uint32_t len);

// Snapshot of the latest values with freshness evaluated at the current tick
IvtReading BMS_IVT_GetReading(void);

// Main loop: (re)configures the IVT after start-up, after it restarts (alive message, or a configuration that isn't
// the BMS's) and while data is missing; clears system errors that are only CAN errors (see bms_ivt.c)
void BMS_IVT_Service(void);

// Timer interrupt, every CFG_TIMER_TICK_MS: feeds an open span
void BMS_IVT_Tick(void);

// Main loop: average U1 and the current range between SpanStart and SpanEnd (e.g. a cell conversion).
// SpanEnd returns false without fresh, error-free IVT data for the whole span; outputs may be NULL.
void BMS_IVT_SpanStart(void);
bool BMS_IVT_SpanEnd(float* u1Avg_V, float* currentRange_A);

#endif /* INC_BMS_IVT_H_ */
