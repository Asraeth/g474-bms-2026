/*
 * bms_libWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "bms_datatypes.h"
#include "main.h"
#include "bms_can.h"
#include "bms_utility.h"

#include <stdbool.h>


typedef enum
{
    VOLTAGE_C,
    VOLTAGE_C_AVG,
    VOLTAGE_C_FIL,
    VOLTAGE_S,
    TOTAL_CELL_VOLTAGE_TYPES,
    VOLTAGE_TEMP = TOTAL_CELL_VOLTAGE_TYPES,
    TOTAL_VOLTAGE_TYPES,
} VoltageTypes;

typedef enum {
    REG_CONFIG_A,
    REG_CONFIG_B,
    REG_PWM_A,
    REG_PWM_B,
    REG_SID,
    TOTAL_REG_TYPES,
} RegisterTypes;


BMS_StatusTypeDef bms_init(void);

BMS_StatusTypeDef bms_startAdcvCont(bool enableRedundant);

BMS_StatusTypeDef bms_readCellVoltage(VoltageTypes voltageType);

BMS_StatusTypeDef bms_readRegister(RegisterTypes regTypes);

BMS_StatusTypeDef bms_getAuxMeasurement(void);

BMS_StatusTypeDef bms_startDischarge(float threshold);

BMS_StatusTypeDef bms_stopDischarge(void);

BMS_StatusTypeDef bms_softReset(void);

typedef struct {
    IVTS_Snapshot ivt;
    float current_a;         // Signed sensor value; vehicle polarity is unassigned.
    float voltage_u1_v;
    float cell_voltage_sum_v; // Independent ADBMS6830 cross-check.
    float voltage_difference_v; // U1 minus cell sum; no invented fault threshold.
    bool cell_voltage_sum_valid;
} BMS_PackMeasurements;

void BMS_GetPackMeasurements(BMS_PackMeasurements *measurements);
BMS_StatusTypeDef BMS_CheckPackSensor(void);
typedef struct {
    bool cell_valid, aux_valid;
    uint32_t cell_last_received_ms, aux_last_received_ms;
    BMS_TransportFault last_acquisition_transport_fault;
    bool last_pec_error, last_counter_error, discharge_stop_failed;
    BMS_CommandCounterChain counters; // Last check; preserves primary failure through stop.
} BMS_AdbmsDataStatus;
void BMS_GetAdbmsDataStatus(BMS_AdbmsDataStatus *status);
BMS_StatusTypeDef BMS_CheckAdbmsData(void);

BMS_StatusTypeDef bms_balancingMeasureVoltage(void);

BMS_StatusTypeDef bms_startBalancing(float deltaThreshold);

// Byte 7 of existing cell/TEMPn frames; n is the logical connector index.
#define BMS_CAN_TEMP_INPUT_VALID  (1U << 0)
#define BMS_CAN_TEMP_C_VALID      (1U << 1)
// Byte 7 of existing board frames. Invalid numeric fields use INT32_MIN
// (VPV in mV) or INT16_MIN (temperature in centidegrees C).
#define BMS_CAN_VPV_VALID         (1U << 0)
#define BMS_CAN_DIE_TEMP_VALID    (1U << 1)
#define BMS_CAN_CELL_VOLTAGE_VALID (1U << 2) // Byte 7 of board/cell frames.

void BMS_GetCanData(CanTxMsg** buff, uint32_t* len);


BMS_StatusTypeDef BMS_ProgramLoop(void);
BMS_StatusTypeDef BMS_UpdateStatusFlags(void);

void BMS_EnableCharging(bool enabled);
void BMS_ChargingButtonLogic(void);
bool BMS_IsCharging(void);

void BMS_EnableBalancing(bool enabled);
void BMS_ToggleBalancing(void);

BMS_StatusTypeDef BMS_CheckTemps(void);
BMS_StatusTypeDef BMS_CheckVoltage(void);
BMS_StatusTypeDef BMS_CheckCurrent(void);
BMS_StatusTypeDef BMS_CheckCommsFault(void);

void BMS_SetCommsFault(bool state);

void BMS_WriteFaultSignal(bool state);

