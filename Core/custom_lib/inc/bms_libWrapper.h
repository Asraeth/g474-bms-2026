/*
 * bms_libWrapper.h
 *
 * The measurement cycle on the ADBMS6830B chain and what is built from it (limits, balancing, SoC, CAN frames).
 */

#pragma once

#include "bms_datatypes.h"
#include "main.h"
#include "bms_can.h"

#include <stdbool.h>
#include "bms_utility.h"


// Configures the chain and starts the C-ADC. BMS_ERR_COMMS if a device doesn't answer.
BMS_StatusTypeDef bms_init(void);

// Every device to sleep with its registers reset (before bms_init again)
void bms_softReset(void);

// One complete measurement cycle. Limit violations are reported to the safety module,
// the return value is only BMS_OK or BMS_ERR_COMMS.
BMS_StatusTypeDef BMS_ProgramLoop(void);

// Builds the CAN frames to send. Called from the timer interrupt.
void BMS_GetCanData(CanTxMsg** buff, uint32_t* len);
uint32_t BMS_GetFastCanData(CanTxMsg msgs[CAN_FAST_FRAME_COUNT]);
void BMS_GetChargerCanMsg(CanTxMsg* msg);
float BMS_GetMaxCellTempC(void);

uint32_t BMS_GetLoopTimeMs(void);

void BMS_EnableCharging(bool enabled);
void BMS_ChargingButtonLogic(void);
bool BMS_IsCharging(void);

void BMS_EnableBalancing(bool enabled);

// Slave board LEDs (normally off). Applied on the next measurement cycle.
void BMS_SetSlaveLeds(uint8_t slave, bool red, bool green);

typedef struct {
    bool cellsValid, auxValid, celsiusValid, pecFault, counterFault, stopFailed;
    uint32_t cellTick, auxTick, generation;
    BMS_TransportFault transportFault;
} BMS_AcquisitionStatus;
void BMS_GetAcquisitionStatus(BMS_AcquisitionStatus *out);
bool BMS_ControlDataValid(void);
float BMS_GetMinCellTempC(void);
float BMS_GetMaxCellVoltageV(void);
void BMS_RequestSafeOutputs(void);
