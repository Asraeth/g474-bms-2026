/*
 * bms_libWrapper.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

/*
 * Compatible commands
 * ADBMS2950 == ADBMS6830
 *
 * RDCFGA
 * RDCFGB
 * ADI1     = ADCV
 * ADI2     = ADSV
 * RDI      = RDFCA or RDCVA
 * RDVB     = RDFCB or RDCVB
 * RDIVB1   = RDFCC or RDCVC
 * RDIACC   = RDACA
 * RDVBACC  = RDACB
 * RDIVB1ACC= RDACC
 *
 */

/*
 * Commands Notes
 *
 * -- 6830 --
 * ADCV : Start ADC
 * ADSV : Start redundancy ADC
 * RDCVA: Read Cell Voltage A
 * RDFCA: Read Filtered Cell A
 * RDACA: Read Averaged Cell A
 *
 * -- 2950 --
 * ADIx: Start IxADC and VBxADC
 * RDI : Read Read I1ADC and I2ADC results
 *
 */


#include "bms_libWrapper.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_cmdlist.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "main.h"
#include "uartDMA.h"
#include "bms_can.h"


typedef struct
{
    // From/For Config Registers
    ad68_cfa_t cfa_Tx   [TOTAL_AD68];
    ad68_cfa_t cfa_Rx   [TOTAL_AD68];
    ad68_cfb_t cfb_Tx   [TOTAL_AD68];
    ad68_cfb_t cfb_Rx   [TOTAL_AD68];

    ad68_pwma_t pwma    [TOTAL_AD68];
    ad68_pwmb_t pwmb    [TOTAL_AD68];

    // From Read Registers
    float v_cell        [TOTAL_CELL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL];
    // Calculated Values
    float v_cell_diff   [TOTAL_CELL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL];
    float v_cell_sum    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_avg    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_min    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_max    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_delta  [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];

    // From AUX measurement
    // Direct TEMP1-TEMP8 input voltages, separate from cell-voltage storage.
    float v_temp        [TOTAL_AD68][TOTAL_TEMP];
    float v_temp_diff   [TOTAL_AD68][TOTAL_TEMP];
    float v_segment     [TOTAL_AD68];
    float temp_cell     [TOTAL_AD68][TOTAL_TEMP];
    float temp_ic       [TOTAL_AD68];
    // Acquisition validity does not imply a confirmed thermistor conversion.
    bool temp_input_valid [TOTAL_AD68][TOTAL_TEMP];
    bool temp_valid       [TOTAL_AD68][TOTAL_TEMP];
    bool v_segment_valid  [TOTAL_AD68];
    bool temp_ic_valid    [TOTAL_AD68];

    // flag stored in bits
    uint16_t isDischarging          [TOTAL_AD68];         // isDischarging Flag
    uint16_t isCellFaultDetected    [TOTAL_AD68];

} Ic_ad68;


typedef struct
{
    bool isFaultDetected    [TOTAL_IC];
    bool isCommsError       [TOTAL_IC];

    // Pack status
    float v_pack_total; // Cell-voltage sum only; never the IVT U1 measurement.
    float v_pack_min;
    float v_pack_max;
    bool cell_voltage_sum_valid;
    bool cell_snapshot_valid[TOTAL_CELL_VOLTAGE_TYPES];
    uint32_t cell_last_received_ms[TOTAL_CELL_VOLTAGE_TYPES];
    bool aux_snapshot_valid;
    uint32_t aux_last_received_ms;
    BMS_TransportFault last_acquisition_transport_fault;
    bool last_pec_error, last_counter_error, discharge_stop_failed;
    BMS_CommandCounterChain last_counter_check;
} Ic_common;


Ic_common   ic_common;
Ic_ad68     ic_ad68;
VoltageTypes dischargeVoltageType = VOLTAGE_S;

// Transaction staging prevents partially updated arrays being observed by ISR
// telemetry/protection. Only a complete checked pass is published.
static float pendingCells[TOTAL_AD68][TOTAL_CELL];
static struct {
    float v_temp[TOTAL_AD68][TOTAL_TEMP];
    float v_segment[TOTAL_AD68], temp_ic[TOTAL_AD68];
} pendingAux;

static BMS_StatusTypeDef bms_acquisitionFailed(void);
bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC]);

static bool bms_cellDataFresh(VoltageTypes type)
{
    return (unsigned int)type < TOTAL_CELL_VOLTAGE_TYPES && ic_common.cell_snapshot_valid[type]
            && (uint32_t)(HAL_GetTick() - ic_common.cell_last_received_ms[type]) < BMS_ADBMS_CELL_MAX_AGE_MS;
}

static bool bms_auxDataFresh(void)
{
    return ic_common.aux_snapshot_valid
            && (uint32_t)(HAL_GetTick() - ic_common.aux_last_received_ms) < BMS_ADBMS_AUX_MAX_AGE_MS;
}

void BMS_GetAdbmsDataStatus(BMS_AdbmsDataStatus *status)
{
    status->cell_valid = bms_cellDataFresh(dischargeVoltageType);
    status->aux_valid = bms_auxDataFresh();
    status->cell_last_received_ms = ic_common.cell_last_received_ms[dischargeVoltageType];
    status->aux_last_received_ms = ic_common.aux_last_received_ms;
    status->last_acquisition_transport_fault = ic_common.last_acquisition_transport_fault;
    status->last_pec_error = ic_common.last_pec_error;
    status->last_counter_error = ic_common.last_counter_error;
    status->counters = ic_common.last_counter_check;
    status->discharge_stop_failed = ic_common.discharge_stop_failed;
}

BMS_StatusTypeDef BMS_CheckAdbmsData(void)
{
    return bms_cellDataFresh(dischargeVoltageType) && bms_auxDataFresh() ? BMS_OK : BMS_ERR_COMMS;
}

uint8_t  txData[TOTAL_IC][DATA_LEN];
uint8_t  rxData[TOTAL_IC][DATA_LEN];
uint16_t rxPec[TOTAL_IC];
uint8_t  rxCc[TOTAL_IC];

CanTxMsg canTxBuffer[CAN_BUFFER_LEN] = {0};

uint32_t BMS_StatusFlags = BMS_ERR_COMMS;          // Stores flags in bits

ChargerConfiguration chargerConfig = {
        .max_current = BMS_MA_TO_A(BMS_CHARGER_MAX_CURRENT_MA),
        .target_voltage = BMS_MV_TO_V(BMS_CHARGER_TARGET_MV),
        .disable_charging = 1,
};

static const float balancingThreshold = BMS_MV_TO_V(BMS_BALANCING_THRESHOLD_MV);

static const bool DEBUG_SERIAL_VOLTAGE_ENABLED = false;
static const bool DEBUG_SERIAL_AUX_ENABLED = false;

volatile bool enableBalancing = BMS_BALANCING_ENABLED_DEFAULT;
static volatile bool dischargeStopRequested;

static void bms_clearDischargeState(void)
{
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        memset(&ic_ad68.pwma[ic], 0, sizeof(ic_ad68.pwma[ic]));
        memset(&ic_ad68.pwmb[ic], 0, sizeof(ic_ad68.pwmb[ic]));
        ic_ad68.cfb_Tx[ic].dcc = 0;
        ic_ad68.cfb_Tx[ic].dcto = 0;
        ic_ad68.cfb_Tx[ic].dtmen = 0;
        ic_ad68.cfb_Tx[ic].dtrng = 0;
        ic_ad68.isDischarging[ic] = 0;
    }
}

static void bms_sanitizeDischargeControls(void)
{
    if (!enableBalancing || !BMS_BALANCING_CONFIG_CONFIRMED || BMS_BALANCING_MAX_PWM == 0) {
        bms_clearDischargeState();
        return;
    }
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        // Direct DCC overrides are never used, including populated positions.
        ic_ad68.cfb_Tx[ic].dcc = 0;
        ic_ad68.isDischarging[ic] &= BMS_POPULATED_CELL_MASK;
        ic_ad68.pwma[ic].pwm9 = ic_ad68.pwma[ic].pwm10 = 0;
        ic_ad68.pwma[ic].pwm11 = ic_ad68.pwma[ic].pwm12 = 0;
        memset(&ic_ad68.pwmb[ic], 0, sizeof(ic_ad68.pwmb[ic]));
#define CAP_PWM(n) if (ic_ad68.pwma[ic].pwm##n > BMS_BALANCING_MAX_PWM) ic_ad68.pwma[ic].pwm##n = BMS_BALANCING_MAX_PWM
        CAP_PWM(1); CAP_PWM(2); CAP_PWM(3); CAP_PWM(4);
        CAP_PWM(5); CAP_PWM(6); CAP_PWM(7); CAP_PWM(8);
#undef CAP_PWM
    }
}


void bms_resetConfig(void)
{
    // Obtained from RDCFG after reset
    // Flipped due to Little endian
//    uint64_t const ad68_cfaDefault = 0x01 00 00 FF 03 00;
    uint64_t const ad68_cfaDefault = 0x0003FF000000ULL | BMS_ADBMS_CS_COMPARISON_CODE;
//    uint64_t const ad68_cfbDefault = 0x00 F8 7F 00 00 00;
    uint64_t const ad68_cfbDefault = ((uint64_t)BMS_ADBMS_HW_VOV_CODE << 12) | BMS_ADBMS_HW_VUV_CODE;

    // Copy defaults to Tx Buffer
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(&ic_ad68.cfa_Tx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ic_ad68.cfb_Tx[ic], &ad68_cfbDefault, DATA_LEN);
    }
    bms_clearDischargeState();

//    ad68_cfa_t ad68_cfaT;
//    memcpy(&ad68_cfaT, &ad68_cfaDefault, DATA_LEN);
}


BMS_StatusTypeDef bms_writeRegister(RegisterTypes regType)
{
    bms_sanitizeDischargeControls();
    uint8_t* command;

    // Use a switch statement to handle the logic for each register type
    switch (regType)
    {
        case REG_CONFIG_A:
            // Prepare data for writing to Configuration Register A
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic], &ic_ad68.cfa_Tx[ic], DATA_LEN);
            }
            command = WRCFGA; // Set the specific command for this operation
            break;

        case REG_CONFIG_B:
            // Prepare data for writing to Configuration Register B
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic], &ic_ad68.cfb_Tx[ic], DATA_LEN);
            }
            command = WRCFGB; // Set the specific command for this operation
            break;

        case REG_PWM_A:
            // Prepare data for writing to PWM Register Group A
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic], &ic_ad68.pwma[ic], DATA_LEN);
            }
            command = WRPWMA; // Set the specific command for this operation
            break;

        case REG_PWM_B:
            // Prepare data for writing to PWM Register Group B
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic], &ic_ad68.pwmb[ic], DATA_LEN);
            }
            command = WRPWMB; // Set the specific command for this operation
            break;

        default:
            // In case of an invalid regType
            Error_Handler();
            return BMS_ERR_COMMS;
    }

    // After preparing the buffer, transmit the data with the selected command.
    // This part is common to all cases.
    return bms_transmitData(command, txData);
}


static void bms_invalidateAuxMeasurement(void)
{
    ic_common.aux_snapshot_valid = false;
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int t = 0; t < TOTAL_TEMP; t++)
        {
            ic_ad68.v_temp[ic][t] = NAN;
            ic_ad68.v_temp_diff[ic][t] = NAN;
            ic_ad68.temp_cell[ic][t] = NAN;
            ic_ad68.temp_input_valid[ic][t] = false;
            ic_ad68.temp_valid[ic][t] = false;
        }
        ic_ad68.v_segment[ic] = NAN;
        ic_ad68.temp_ic[ic] = NAN;
        ic_ad68.v_segment_valid[ic] = false;
        ic_ad68.temp_ic_valid[ic] = false;
        ic_ad68.v_cell_sum[VOLTAGE_TEMP][ic] = NAN;
        ic_ad68.v_cell_avg[VOLTAGE_TEMP][ic] = NAN;
        ic_ad68.v_cell_min[VOLTAGE_TEMP][ic] = NAN;
        ic_ad68.v_cell_max[VOLTAGE_TEMP][ic] = NAN;
        ic_ad68.v_cell_delta[VOLTAGE_TEMP][ic] = NAN;
    }
}


static void bms_invalidateCellVoltage(VoltageTypes type)
{
    ic_common.cell_snapshot_valid[type] = false;
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        for (unsigned int c = 0; c < TOTAL_CELL; c++) {
            ic_ad68.v_cell[type][ic][c] = NAN;
            ic_ad68.v_cell_diff[type][ic][c] = NAN;
        }
        ic_ad68.v_cell_sum[type][ic] = NAN;
        ic_ad68.v_cell_avg[type][ic] = NAN;
        ic_ad68.v_cell_min[type][ic] = NAN;
        ic_ad68.v_cell_max[type][ic] = NAN;
        ic_ad68.v_cell_delta[type][ic] = NAN;
    }
    if (type == dischargeVoltageType) {
        ic_common.cell_voltage_sum_valid = false;
        ic_common.v_pack_total = ic_common.v_pack_min = ic_common.v_pack_max = NAN;
    }
}

static void bms_invalidateAllMeasurements(void)
{
    uint32_t mask = bms_enterCritical();
    for (int type = 0; type < TOTAL_CELL_VOLTAGE_TYPES; type++) bms_invalidateCellVoltage(type);
    bms_invalidateAuxMeasurement();
    bms_exitCritical(mask);
}

static void bms_latchCommunicationFault(void)
{
    bms_invalidateAllMeasurements();
    enableBalancing = false;
    for (unsigned int ic = 0; ic < TOTAL_IC; ic++) ic_common.isCommsError[ic] = true;
    BMS_StatusFlags |= BMS_ERR_COMMS;
    BMS_WriteFaultSignal(true);
    if (BMS_IsCharging()) BMS_EnableCharging(false);
    bms_clearDischargeState();
}

static BMS_StatusTypeDef bms_acquisitionFailed(void)
{
    BMS_TransportFault primaryFault = bms_getTransportFault();
    bool primaryPecError = primaryFault == BMS_TRANSPORT_OK && ic_common.last_pec_error;
    bool primaryCounterError = primaryFault == BMS_TRANSPORT_OK && ic_common.last_counter_error;
    BMS_CommandCounterChain primaryCounters;
    bms_getCounterState(&primaryCounters);
    bms_latchCommunicationFault();
    // Foreground-only, finite best effort. A broken link cannot prove that the
    // physical PWM/discharge registers were cleared; the fault output stays on.
    bms_stopDischarge();
    // Stop/readback transactions must not overwrite the acquisition diagnosis.
    ic_common.last_acquisition_transport_fault = primaryFault;
    ic_common.last_pec_error = primaryPecError;
    ic_common.last_counter_error = primaryCounterError;
    ic_common.last_counter_check = primaryCounters;
    return BMS_ERR_COMMS;
}

static BMS_StatusTypeDef bms_synchronizeCounters(void)
{
    // Do not learn an arbitrary received value after MCU reset or a failed
    // transaction. RSTCC leaves conversion/configuration registers untouched.
    // Validate every device at zero, with PEC, before using the new baseline.
    if (bms_transmitCmd(RSTCC) != BMS_OK
            || bms_receiveData(RDSID, rxData, rxPec, rxCc) != BMS_OK
            || bms_checkRxFault(rxData, rxPec, rxCc)) return BMS_ERR_COMMS;
    return BMS_OK;
}

BMS_StatusTypeDef bms_init(void)
{
    bms_invalidateAllMeasurements();
    enableBalancing = BMS_BALANCING_ENABLED_DEFAULT;
    bms_resetConfig();

    // For 6830 Configs
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].refon = 0b1;
        ic_ad68.cfa_Tx[ic].fc = BMS_ADBMS_FILTER_CODE;
        // Release the analog inputs; preserve GPIO1/2 (LED) control bits.
        ic_ad68.cfa_Tx[ic].gpo1to8 |= 0xFC;
        ic_ad68.cfa_Tx[ic].gpo9to10 |= 0x03;
    }

    printfDma("\n --- BMS INIT --- \n");

    if (bms_wakeupChain() != BMS_OK) return bms_acquisitionFailed();
    if (bms_synchronizeCounters() != BMS_OK)
    {
        return bms_acquisitionFailed();
    }

    if (bms_stopDischarge() != BMS_OK || bms_writeRegister(REG_CONFIG_A) != BMS_OK || bms_startAdcvCont(false) != BMS_OK)
        return bms_acquisitionFailed();
    if (bms_delayMsActive(BMS_CELL_INITIAL_FILL_MS) != BMS_OK) return bms_acquisitionFailed();

    return BMS_OK;
}


void bms_printRawData(uint8_t data[TOTAL_IC][DATA_LEN], uint8_t cc[TOTAL_IC])
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("IC%d: ", ic+1);
        for (int j = 0; j < 6; j++)             // For every byte recieved (6 bytes)
        {
            printfDma("0x%02X, ", data[ic][j]);    // Print each of the bytes
        }
        printfDma("CC: %d |   ", cc[ic]);
    }
    printfDma("\n\n");
}


bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    bool faultDetected = false;
    bool* errorIndex = ic_common.isCommsError;

    if (bms_checkRxPec(data, pec, cc, errorIndex))
    {
        printfDma("WARNING! PEC ERROR - IC:");
        for(int ic = 0; ic < TOTAL_IC; ic++)
        {
            if (errorIndex[ic])
            {
                printfDma(" %d,", ic+1);
            }
        }
        printfDma("\n");
        faultDetected = true;
    }
    ic_common.last_pec_error = faultDetected;

    // Check each PEC-verified counter; corrupt packets cannot synchronize or
    // overwrite that device's last trusted observation. Reads do not increment.
    faultDetected |= bms_checkRxCounter(cc, errorIndex, errorIndex);
    bms_getCounterState(&ic_common.last_counter_check);
    ic_common.last_counter_error = false;
    for (unsigned int ic = 0; ic < TOTAL_IC; ic++) {
        if (ic_common.last_counter_check.device[ic].fault) {
            ic_common.last_counter_error = true;
            printfDma("COUNTER ERROR IC%u: expected %u received %u synced %u\n",
                    ic + 1, ic_common.last_counter_check.device[ic].expected,
                    ic_common.last_counter_check.device[ic].received,
                    ic_common.last_counter_check.device[ic].synchronized);
        }
    }
    return faultDetected;
}


// used mostly for debugging purposes
BMS_StatusTypeDef bms_readRegister(RegisterTypes regType)
{
    char* title;
    uint8_t* cmd;
    switch (regType)
    {
    case REG_CONFIG_A:
        title = "Config A Register";
        cmd = RDCFGA;
        break;
    case REG_CONFIG_B:
        title = "Config B Register";
        cmd = RDCFGB;
        break;
    case REG_PWM_A:
        title = "PWM A Register";
        cmd = RDPWMA;
        break;
    case REG_PWM_B:
        title = "PWM B Register";
        cmd = RDPWMB;
        break;
    case REG_SID:
        title = "SID Register";
        cmd = RDSID;
        break;
    default:
        Error_Handler(); // Invalid Choice
        break;
    }

    if (bms_receiveData(cmd, rxData, rxPec, rxCc) != BMS_OK || bms_checkRxFault(rxData, rxPec, rxCc))
    {
        return bms_acquisitionFailed();
    }

    printfDma("%s: \n", title);
    bms_printRawData(rxData, rxCc);

    return BMS_OK;
}


BMS_StatusTypeDef bms_startAdcvCont(bool enableRedundant)
{
    // 6830
    // For DCP = 0
    // If RD = 0 and CONT = 1, PWM discharge is unaffected
    // If RD = 1 and CONT = 0, (Might be wrong) PWM discharge interrupted temporarily until RD conversion finished (8ms typ)
    // If RD = 1 and CONT = 1, PWM discharge interrupted

    ADCV.CONT = 1;      // Continuous
    ADCV.DCP  = 0;      // Discharge permitted
    ADCV.RSTF = 1;      // Reset filter
    ADCV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    ADCV.RD   = enableRedundant;      // Redundant Measurement

    // Behaviour of 2950 (ADI1 Command)
    //

    return bms_transmitCmd((uint8_t *)&ADCV);
}


void bms_parseVoltage(const uint8_t rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_AD68][TOTAL_CELL], uint8_t register_index)
{
    const int cell_index = register_index * 3;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int word = 0; word < 3 && cell_index + word < TOTAL_CELL; word++)
        {
            const int offset = word * 2;
            const int16_t rawVoltage = (int16_t)((uint16_t)rawData[ic][offset]
                    | ((uint16_t)rawData[ic][offset + 1] << 8));
            // Preserve double intermediates and round once at publication.
            // Single-precision constants can change protection boundaries.
            vArr[ic][cell_index + word] = rawVoltage == INT16_MIN ? NAN
                    : (float)((double)rawVoltage * 0.00015 + 1.5);
        }
    }
}


static int16_t bms_auxRawWord(const uint8_t data[DATA_LEN], uint8_t offset)
{
    return (int16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1] << 8));
}


void bms_parseAuxVoltage(const uint8_t rawData[TOTAL_IC][DATA_LEN], uint8_t group)
{
    // BMS-Slave-V3: logical TEMP1 through TEMP8, not GPIO-number order.
    static const uint8_t tempGpio[TOTAL_TEMP] = {9, 7, 8, 10, 4, 6, 5, 3};

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (group == 4) // RDSTATA: internal die temperature, bytes 2-3.
        {
            const int16_t raw = bms_auxRawWord(rawData[ic], 2);
            // ITMP powers up as 0x7FFF and is cleared to 0x8000.
            // Retain the original double calculation, including the 70 C boundary.
            pendingAux.temp_ic[ic] = (raw == INT16_MIN || raw == INT16_MAX)
                    ? NAN : (float)(((double)raw * 0.00015 + 1.5) / 0.0075 - 273);
            continue;
        }

        for (int t = 0; t < TOTAL_TEMP; t++)
        {
            const uint8_t gpioIndex = tempGpio[t] - 1;
            if (gpioIndex / 3 == group)
            {
                const int16_t raw = bms_auxRawWord(rawData[ic], (gpioIndex % 3) * 2);
                pendingAux.v_temp[ic][t] = (raw == INT16_MIN) ? NAN : raw * 0.00015 + 1.5;
            }
        }

        if (group == 3) // RDAUXD: VPV is bytes 4-5; bytes 2-3 are VMV.
        {
            const int16_t raw = bms_auxRawWord(rawData[ic], 4);
            // The factor 25 is the device's internal V+ measurement scaling.
            // Keep the existing scale and double intermediates; narrow once.
            pendingAux.v_segment[ic] = (raw == INT16_MIN) ? NAN
                    : (float)(((double)raw * 0.00015 + 1.5) * 25);
        }
    }
}


void bms_calculateStats(VoltageTypes voltageType)
{
    const int channelCount = (voltageType == VOLTAGE_TEMP) ? TOTAL_TEMP : TOTAL_CELL;
    float total_voltage = 0;
    float pack_min =  999.0;
    float pack_max = -999.0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float *vArr = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp[ic] : ic_ad68.v_cell[voltageType][ic];
        float *vDiff = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp_diff[ic] : ic_ad68.v_cell_diff[voltageType][ic];
        float min =  999.0;
        float max = -999.0;
        float sum = 0;

        for (int c = 0; c < channelCount; c++)
        {
            float voltage = vArr[c];
            sum += voltage;
            if (voltage > max)
            {
                max = voltage;
            }
            if (voltage < min)
            {
                min = voltage;
            }
            if (voltage > pack_max)
            {
                pack_max = voltage;
            }
            if (voltage < pack_min)
            {
                pack_min = voltage;
            }
        }

        total_voltage += sum;
        ic_ad68.v_cell_min  [voltageType][ic] = min;
        ic_ad68.v_cell_max  [voltageType][ic] = max;
        ic_ad68.v_cell_sum  [voltageType][ic] = sum;
        ic_ad68.v_cell_avg  [voltageType][ic] = sum / channelCount;
        ic_ad68.v_cell_delta[voltageType][ic] = max - min;

        for (int c = 0; c < channelCount; c++)
        {
            vDiff[c] = vArr[c] - min;
        }
    }

    if (voltageType == dischargeVoltageType)
    {
        ic_common.v_pack_total = total_voltage;
        ic_common.v_pack_min = pack_min;
        ic_common.v_pack_max = pack_max;
    }

    // Calculate voltage diff from the lowest voltage cell
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float *vArr = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp[ic] : ic_ad68.v_cell[voltageType][ic];
        float *vDiff = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp_diff[ic] : ic_ad68.v_cell_diff[voltageType][ic];
        for (int c = 0; c < channelCount; c++)
        {
            vDiff[c] = vArr[c] - pack_min;
        }
    }
}


void bms_printVoltage(VoltageTypes voltageType)
{
    const int channelCount = (voltageType == VOLTAGE_TEMP) ? TOTAL_TEMP : TOTAL_CELL;

    char* title;
    switch (voltageType)
    {
    case VOLTAGE_C:
        title = "C Voltage";
        break;
    case VOLTAGE_C_AVG:
        title = "C Average Voltage";
        break;
    case VOLTAGE_C_FIL:
        title = "C Filtered Voltage";
        break;
    case VOLTAGE_S:
        title = "S Voltage";
        break;
    case VOLTAGE_TEMP:
        title = "Temperature Sensors Voltage";
        break;
    default:
        Error_Handler();
        break;
    }
    printfDma("%s: \n", title);

    printfDma("| IC |");
    for (int i = 0; i < channelCount; i++)
    {
        printfDma("   %2d   |", i+1);
    }
    printfDma("  Sum   |  Delta |\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float *vArr = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp[ic] : ic_ad68.v_cell[voltageType][ic];
        float *vDev = (voltageType == VOLTAGE_TEMP) ? ic_ad68.v_temp_diff[ic] : ic_ad68.v_cell_diff[voltageType][ic];
        uint8_t paddingOffset;
        paddingOffset = printfDma("| %2d |", ic+1);
        for (int c = 0; c < channelCount; c++)
        {
            printfDma("%8.5f|", vArr[c]);
        }

        printfDma("%8.5f|", ic_ad68.v_cell_sum  [voltageType][ic]);
        printfDma("%8.5f|", ic_ad68.v_cell_delta[voltageType][ic]);
        printfDma("\n");

        printfDma("%*s", paddingOffset, "");
        for (int c = 0; c < channelCount; c++)
        {
            printfDma("%8.5f|", vDev[c]);
        }
        printfDma("\n");
    }
}


void bms_printTemps(void)
{
    printfDma("TEMPn: input V / acquisition valid / C valid (sensor model unconfirmed)\n");
    printfDma("| IC |");
    for (int i = 0; i < TOTAL_TEMP; i++)
    {
        printfDma("    TEMP%d V/A/C    |", i+1);
    }
    printfDma("  Die Temp / Segment Voltage");
    printfDma("\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printfDma("| %2d |", ic+1);
        for (int c = 0; c < TOTAL_TEMP; c++)
        {
            printfDma("%6.3f/%d/%d |", ic_ad68.v_temp[ic][c],
                    ic_ad68.temp_input_valid[ic][c], ic_ad68.temp_valid[ic][c]);
        }
        printfDma("  %.2f C (valid %d) / %.2f V (valid %d)\n",
                ic_ad68.temp_ic[ic], ic_ad68.temp_ic_valid[ic],
                ic_ad68.v_segment[ic], ic_ad68.v_segment_valid[ic]);
    }
}


uint8_t* readCellVoltageCmdList[TOTAL_CELL_VOLTAGE_TYPES][AD68_CELL_GROUP_COUNT] = {
        {RDCVA, RDCVB, RDCVC}, // VOLTAGE_C
        {RDACA, RDACB, RDACC}, // VOLTAGE_C_AVG
        {RDFCA, RDFCB, RDFCC}, // VOLTAGE_C_FIL
        {RDSVA, RDSVB, RDSVC}, // VOLTAGE_S
};

BMS_StatusTypeDef bms_readCellVoltage(VoltageTypes voltageType)
{
    if ((unsigned int)voltageType >= TOTAL_CELL_VOLTAGE_TYPES)
    {
        Error_Handler();
        return BMS_ERR_VOLTAGE;
    }
    uint8_t** cmdList = readCellVoltageCmdList[voltageType];

    for (int i = 0; i < AD68_CELL_GROUP_COUNT; i++)
    {
        if (bms_receiveData(cmdList[i], rxData, rxPec, rxCc) != BMS_OK || bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return bms_acquisitionFailed();
        }
        bms_parseVoltage(rxData, pendingCells, i);
    }
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++)
        for (unsigned int c = 0; c < TOTAL_CELL; c++)
            if (!isfinite(pendingCells[ic][c])) return bms_acquisitionFailed();
    uint32_t mask = bms_enterCritical();
    memcpy(ic_ad68.v_cell[voltageType], pendingCells, sizeof(pendingCells));
    bms_calculateStats(voltageType);
    ic_common.cell_last_received_ms[voltageType] = HAL_GetTick();
    ic_common.cell_snapshot_valid[voltageType] = true;
    if (voltageType == dischargeVoltageType) ic_common.cell_voltage_sum_valid = true;
    bms_exitCritical(mask);

    if (DEBUG_SERIAL_VOLTAGE_ENABLED) bms_printVoltage(voltageType);

    return BMS_OK;
}


BMS_StatusTypeDef bms_getAuxVoltage(void)
{
    uint8_t* cmdList[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA};

    for (int i = 0; i < 5; i++)
    {
        if (bms_receiveData(cmdList[i], rxData, rxPec, rxCc) != BMS_OK || bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }
        bms_parseAuxVoltage(rxData, i);
    }

    return BMS_OK;
}


// Legacy reference only: the V3 sensor model/divider conversion is unconfirmed.
// Do not call this table to produce a valid temperature for the new hardware.
static float __attribute__((unused)) convertCellTemp(float cellVoltage)
{
    // NaN bypasses ordered comparisons and would overrun the legacy table.
    // Keep it invalid; this function remains unused and unapproved for V3.
    if (isnan(cellVoltage)) return NAN;

    // From datasheet
    static const float tempValues[]    = { -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5,   10,   15,   20,   25,   30,   35,   40,   45,   50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100,  105,  110,  115,  120};
    static const float voltageValues[] = {2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30};
    static const int   numDataPoints   = sizeof(tempValues) / sizeof(tempValues[0]);

    // Check if in range
    if (cellVoltage > 2.44 || cellVoltage < 1.30)
    {
        // Voltage out of range
        return 888.0;
    }

    int idx;
    for (idx = 0; idx < numDataPoints - 1; idx++)
    {
        if (cellVoltage > voltageValues[idx + 1]) break;
    }

    float x1 = voltageValues[idx];
    float x2 = voltageValues[idx + 1];
    float y1 = tempValues[idx];
    float y2 = tempValues[idx + 1];

    return y1 + (cellVoltage - x1) * (y2 - y1) / (x2 - x1);
}


BMS_StatusTypeDef bms_getAuxMeasurement(void)
{
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        for (unsigned int t = 0; t < TOTAL_TEMP; t++) pendingAux.v_temp[ic][t] = NAN;
        pendingAux.v_segment[ic] = pendingAux.temp_ic[ic] = NAN;
    }
    // One normal all-channel conversion; GPIO1/2 results are not temperatures.
    ADAX.OW   = 0b0;
    ADAX.CH   = 0b0000;
    ADAX.CH4  = 0b0;
    ADAX.PUP  = 0b0;

    if (bms_transmitCmd((uint8_t *)&ADAX) != BMS_OK || bms_transmitPoll(PLAUX1) != BMS_OK
            || bms_getAuxVoltage() != BMS_OK)
    {
        // Discard the entire pass, including groups read before the failure.
        return bms_acquisitionFailed();
    }

    // Celsius stays NAN/invalid until the sensor and conversion are confirmed.
    bool allInputsValid = true;
    uint32_t mask = bms_enterCritical();
    bms_invalidateAuxMeasurement();
    memcpy(ic_ad68.v_temp, pendingAux.v_temp, sizeof(pendingAux.v_temp));
    memcpy(ic_ad68.v_segment, pendingAux.v_segment, sizeof(pendingAux.v_segment));
    memcpy(ic_ad68.temp_ic, pendingAux.temp_ic, sizeof(pendingAux.temp_ic));
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int t = 0; t < TOTAL_TEMP; t++)
        {
            ic_ad68.temp_input_valid[ic][t] = isfinite(ic_ad68.v_temp[ic][t]);
            allInputsValid &= ic_ad68.temp_input_valid[ic][t];
        }
        ic_ad68.v_segment_valid[ic] = isfinite(ic_ad68.v_segment[ic]);
        ic_ad68.temp_ic_valid[ic] = isfinite(ic_ad68.temp_ic[ic]);
    }
    // These statistics describe input volts, never degrees C.
    if (allInputsValid) bms_calculateStats(VOLTAGE_TEMP);
    ic_common.aux_last_received_ms = HAL_GetTick();
    ic_common.aux_snapshot_valid = true;
    bms_exitCritical(mask);
    if (DEBUG_SERIAL_VOLTAGE_ENABLED)   bms_printVoltage(VOLTAGE_TEMP);
    if (DEBUG_SERIAL_AUX_ENABLED)      bms_printTemps();

    return BMS_OK;
}


void bms_setPwm(uint8_t ic_index, uint8_t cell, uint8_t dutyCycle)
{
    if (ic_index >= TOTAL_AD68 || cell >= AD68_HARDWARE_CELL_CHANNELS) return;
    if (cell >= TOTAL_CELL || !enableBalancing || !BMS_BALANCING_CONFIG_CONFIRMED) dutyCycle = 0;
    if (dutyCycle > BMS_BALANCING_MAX_PWM) dutyCycle = BMS_BALANCING_MAX_PWM;
    cell++;                                 // Change from 0 indexing to 1 indexing

    switch (cell) {
        case 1:
            ic_ad68.pwma[ic_index].pwm1 = dutyCycle;
            break;
        case 2:
            ic_ad68.pwma[ic_index].pwm2 = dutyCycle;
            break;
        case 3:
            ic_ad68.pwma[ic_index].pwm3 = dutyCycle;
            break;
        case 4:
            ic_ad68.pwma[ic_index].pwm4 = dutyCycle;
            break;
        case 5:
            ic_ad68.pwma[ic_index].pwm5 = dutyCycle;
            break;
        case 6:
            ic_ad68.pwma[ic_index].pwm6 = dutyCycle;
            break;
        case 7:
            ic_ad68.pwma[ic_index].pwm7 = dutyCycle;
            break;
        case 8:
            ic_ad68.pwma[ic_index].pwm8 = dutyCycle;
            break;
        case 9:
            ic_ad68.pwma[ic_index].pwm9 = dutyCycle;
            break;
        case 10:
            ic_ad68.pwma[ic_index].pwm10 = dutyCycle;
            break;
        case 11:
            ic_ad68.pwma[ic_index].pwm11 = dutyCycle;
            break;
        case 12:
            ic_ad68.pwma[ic_index].pwm12 = dutyCycle;
            break;
        case 13:
            ic_ad68.pwmb[ic_index].pwm13 = dutyCycle;
            break;
        case 14:
            ic_ad68.pwmb[ic_index].pwm14 = dutyCycle;
            break;
        case 15:
            ic_ad68.pwmb[ic_index].pwm15 = dutyCycle;
            break;
        case 16:
            ic_ad68.pwmb[ic_index].pwm16 = dutyCycle;
            break;
        default:
            // Handle invalid cases
            break;
    }
}


/*
 * Converts delta threshold (threshold difference between all cell voltages)
 * to
 * discharge threshold (voltage at which cell stop discharging)
 */
float bms_calculateBalancing(float deltaThreshold)
{
    float min = ic_common.v_pack_min;
    float max = ic_common.v_pack_max;

    if (max - min > deltaThreshold)
    {
        return min + deltaThreshold;
    }
    else
    {
        return INFINITY; // No balancing required; the start guard requests a stop.
    }
}


BMS_StatusTypeDef bms_startDischarge(float dischargeThreshold)
{
    if (!enableBalancing || !BMS_BALANCING_CONFIG_CONFIRMED || BMS_BALANCING_MAX_PWM == 0
            || BMS_CheckAdbmsData() != BMS_OK || BMS_CheckPackSensor() != BMS_OK
            || BMS_UpdateStatusFlags() != BMS_OK || !isfinite(dischargeThreshold)) return bms_stopDischarge();
    bms_clearDischargeState();
//    dischargeThreshold = 5;  // Overwrite the discharge aim voltage (for testing)
    uint32_t cellDischargeCount;                    // Keep count of how many cells will be discharged per segment
    uint8_t dutyCycle = 0;                          // 4 bit pwm at 937 ms
    const uint16_t maxDischarge = BMS_BALANCING_PWM_BUDGET;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        cellDischargeCount = 0;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                cellDischargeCount++;
            }
        }

        if (cellDischargeCount == 0) {cellDischargeCount = 1; };

        dutyCycle = maxDischarge / cellDischargeCount;
        if (dutyCycle > BMS_BALANCING_MAX_PWM)
        {
            dutyCycle = BMS_BALANCING_MAX_PWM;
        }

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                printfDma("DISCHARGE: IC %d, CELL %d, DC %d \n", ic+1, c+1, dutyCycle);
                BIT_SET(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, c, dutyCycle);
            }
            else
            {
                BIT_CLEAR(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, c, 0b0000);    // Turn off PWM discharge for that cell
            }
        }

        // The PWM discharge functionality is possible in the standby, REF-UP, extended balancing and in the measure states
        // AND while the discharge timeout has not expired (DCTO ≠ 0)
        ic_ad68.cfb_Tx[ic].dcto = BMS_BALANCING_TIMEOUT_MINUTES;
        ic_ad68.cfb_Tx[ic].dtmen = 0;    // Disables Discharge Timer Monitor (DTM)
        // ic_ad68[0].cfb_Tx.dcc = 0b1; // --- High priority discharge (bypasses PWM)
    }

    // for testing -> enables discharge for cell 1
//    printfDma("DISCHARGE: IC 1, CELL 1 \n");
//    ic_ad68.pwma[0].pwm1 = 0b1111;

    if (bms_writeRegister(REG_CONFIG_B) != BMS_OK || bms_writeRegister(REG_PWM_A) != BMS_OK
            || bms_writeRegister(REG_PWM_B) != BMS_OK || bms_transmitCmd(UNMUTE) != BMS_OK)
        return bms_acquisitionFailed();
    return BMS_OK;
}


static BMS_StatusTypeDef bms_verifyDischargeOff(void)
{
    uint8_t *commands[] = {RDCFGB, RDPWMA, RDPWMB};
    for (int group = 0; group < 3; group++) {
        if (bms_receiveData(commands[group], rxData, rxPec, rxCc) != BMS_OK
                || bms_checkRxFault(rxData, rxPec, rxCc)) return BMS_ERR_COMMS;
        for (unsigned int ic = 0; ic < TOTAL_IC; ic++) {
            // CFGB byte 3: discharge timer/mode; bytes 4-5: all DCC bits.
            for (int byte = group == 0 ? 3 : 0; byte < DATA_LEN; byte++)
                if (rxData[ic][byte] != 0) return BMS_ERR_COMMS;
        }
    }
    return BMS_OK;
}

BMS_StatusTypeDef bms_stopDischarge(void)
{
    bms_clearDischargeState();
    // Try all zero writes even if an earlier transaction fails. Never recurse.
    BMS_StatusTypeDef status = bms_transmitCmd(MUTE);
    status |= bms_writeRegister(REG_CONFIG_B);
    status |= bms_writeRegister(REG_PWM_A);
    status |= bms_writeRegister(REG_PWM_B);
    if (status == BMS_OK) status = bms_verifyDischargeOff();
    ic_common.discharge_stop_failed = status != BMS_OK;
    if (status != BMS_OK) bms_latchCommunicationFault();
    dischargeStopRequested = false;
    return status;
}


BMS_StatusTypeDef bms_softReset(void)
{
    bms_invalidateAllMeasurements();
    enableBalancing = false;
    bms_clearDischargeState();
    if (bms_wakeupChain() != BMS_OK) return bms_acquisitionFailed();
    // Recovery must establish a known sequence before stop/readback, otherwise
    // a previous sequence fault would prevent the reset that repairs it.
    if (bms_synchronizeCounters() != BMS_OK) return bms_acquisitionFailed();
    if (bms_stopDischarge() != BMS_OK) return BMS_ERR_COMMS;
    if (bms_transmitCmd(SRST) != BMS_OK) return bms_acquisitionFailed();
    printfDma("\n  ---  SOFT RESET  ---  \n");
    return BMS_OK;
}


void BMS_GetPackMeasurements(BMS_PackMeasurements *measurements)
{
    BMS_CAN_GetIVTSnapshot(&measurements->ivt);
    measurements->current_a = measurements->ivt.current.valid
            ? measurements->ivt.current.raw_value / 1000.0f : NAN;
    measurements->voltage_u1_v = measurements->ivt.u1.valid
            ? measurements->ivt.u1.raw_value / 1000.0f : NAN;
    measurements->cell_voltage_sum_valid = bms_cellDataFresh(dischargeVoltageType)
            && ic_common.cell_voltage_sum_valid && isfinite(ic_common.v_pack_total);
    measurements->cell_voltage_sum_v = measurements->cell_voltage_sum_valid ? ic_common.v_pack_total : NAN;
    measurements->voltage_difference_v = measurements->ivt.u1.valid && measurements->cell_voltage_sum_valid
            ? measurements->voltage_u1_v - measurements->cell_voltage_sum_v : NAN;
}

BMS_StatusTypeDef BMS_CheckPackSensor(void)
{
    IVTS_Snapshot sensor;
    BMS_CAN_GetIVTSnapshot(&sensor);
    return sensor.current.valid && sensor.u1.valid ? BMS_OK : BMS_ERR_IVT;
}



BMS_StatusTypeDef bms_balancingMeasureVoltage(void)
{
    // 6830
    // ADSV For triggering single shot S conversion (stops PWM) while C in unaffected
    // So this stops PWM and wait for S to finish
    // Then read the S voltage
    // Potential improvement: S vs C ADC comparison

    ADSV.CONT = 0;      // Continuous
    ADSV.DCP  = 0;      // Discharge permitted
    ADSV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    if (bms_transmitCmd((uint8_t *)&ADSV) != BMS_OK || bms_transmitPoll(PLSADC) != BMS_OK)
        return bms_acquisitionFailed();
    if (bms_readCellVoltage(dischargeVoltageType))
    {
        return BMS_ERR_COMMS;
    }

    return BMS_OK;
}


BMS_StatusTypeDef bms_startBalancing(float deltaThreshold)
{
    float dischargeThreshold = bms_calculateBalancing(deltaThreshold);
    return bms_startDischarge(dischargeThreshold);
}


static bool bms_canTemperatureValid(float temperature, bool valid)
{
    // Reserve INT16_MIN for invalid data and avoid undefined float-to-int casts.
    const float centidegrees = temperature * 100.0f;
    return valid && isfinite(centidegrees)
            && centidegrees >= -32767.0f && centidegrees <= 32767.0f;
}


void BMS_GetCanData(CanTxMsg** buff, uint32_t* len)
{
    uint32_t bufferlen = 0;
    uint32_t id = BASE_CAN_ID;
    FDCAN_TxHeaderTypeDef txHeader;

    /* Prepare Tx Header */
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = 8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        const bool cellValid = bms_cellDataFresh(dischargeVoltageType);
        const bool auxFresh = bms_auxDataFresh();
        const bool vpvValid = auxFresh && ic_ad68.v_segment_valid[ic] && isfinite(ic_ad68.v_segment[ic]);
        const bool dieTempValid = bms_canTemperatureValid(ic_ad68.temp_ic[ic], auxFresh && ic_ad68.temp_ic_valid[ic]);
        int32_t v_segment       = vpvValid ? (int32_t)(ic_ad68.v_segment[ic] * 1000) : INT32_MIN;
        int16_t temp_ic         = dieTempValid ? (int16_t)(ic_ad68.temp_ic[ic] * 100) : INT16_MIN;
        uint8_t isCommsError    = ic_common.isCommsError[ic] || !cellValid || !auxFresh;
        uint8_t isFaultDetected = ic_common.isFaultDetected[ic];

        memcpy(&canTxBuffer[bufferlen].data[0], &v_segment, 4);
//        canTxBuffer[bufferlen].data[4] = (uint8_t)(temp_ic & 0xFF);
//        canTxBuffer[bufferlen].data[5] = (uint8_t)((temp_ic >> 8) & 0xFF);
        memcpy(&canTxBuffer[bufferlen].data[4], &temp_ic, 2);
        canTxBuffer[bufferlen].data[6] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));
        canTxBuffer[bufferlen].data[7] = (vpvValid ? BMS_CAN_VPV_VALID : 0)
                | (dieTempValid ? BMS_CAN_DIE_TEMP_VALID : 0)
                | (cellValid ? BMS_CAN_CELL_VOLTAGE_VALID : 0);

        txHeader.Identifier = BMS_BOARD_CAN_ID_FIRST + ic;
        canTxBuffer[bufferlen].header = txHeader;
        bufferlen++;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            int16_t cellVoltage = cellValid ? (int16_t)(ic_ad68.v_cell[dischargeVoltageType][ic][c] * 1000) : INT16_MIN;
            int16_t voltageDiff = cellValid ? (int16_t)(ic_ad68.v_cell_diff[dischargeVoltageType][ic][c] * 1000) : INT16_MIN;
            const bool tempValid = bms_canTemperatureValid(ic_ad68.temp_cell[ic][c], auxFresh && ic_ad68.temp_valid[ic][c]);
            int16_t cellTemp    = tempValid ? (int16_t)(ic_ad68.temp_cell[ic][c] * 100) : INT16_MIN;
            uint8_t isDischarging       = ((ic_ad68.isDischarging[ic]       >> c) & 0x01U);
            uint8_t isCellFaultDetected = ((ic_ad68.isCellFaultDetected[ic] >> c) & 0x01U);

            canTxBuffer[bufferlen].data[0] = (uint8_t)(cellVoltage & 0xFF);
            canTxBuffer[bufferlen].data[1] = (uint8_t)((cellVoltage >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[2] = (uint8_t)(voltageDiff & 0xFF);
            canTxBuffer[bufferlen].data[3] = (uint8_t)((voltageDiff >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[4] = (uint8_t)(cellTemp & 0xFF);
            canTxBuffer[bufferlen].data[5] = (uint8_t)((cellTemp >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[6] = (uint8_t)((isDischarging << 0) | (isCellFaultDetected << 1));
            // TEMPn shares the existing cell-n slot; this is not a claim about
            // where that sensor is physically attached within the segment.
            canTxBuffer[bufferlen].data[7] = (auxFresh && ic_ad68.temp_input_valid[ic][c] ? BMS_CAN_TEMP_INPUT_VALID : 0)
                    | (tempValid ? BMS_CAN_TEMP_C_VALID : 0)
                    | (cellValid ? BMS_CAN_CELL_VOLTAGE_VALID : 0);

            uint32_t id_cell_offset = ic*TOTAL_CELL + c;

            txHeader.Identifier = id + id_cell_offset;
            canTxBuffer[bufferlen].header = txHeader;
            bufferlen++;
        }
    }

    BMS_PackMeasurements pack;
    BMS_GetPackMeasurements(&pack);
    const IVTS_Result *results[] = {&pack.ivt.current, &pack.ivt.u1};
    for (int channel = 0; channel < 2; channel++) {
        const IVTS_Result *result = results[channel];
        int32_t value = result->valid ? result->raw_value : INT32_MIN;
        // Vehicle payload is little-endian, independently of IVT wire format.
        memcpy(canTxBuffer[bufferlen].data, &value, 4);
        canTxBuffer[bufferlen].data[4] = result->raw_status;
        canTxBuffer[bufferlen].data[5] = result->counter;
        canTxBuffer[bufferlen].data[6] = (result->valid ? BMS_PACK_VALID : 0)
                | (result->stale ? BMS_PACK_STALE : 0)
                | (result->received ? BMS_PACK_RECEIVED : 0)
                | (pack.ivt.configuration_verified ? BMS_PACK_CONFIG_VERIFIED : 0);
        canTxBuffer[bufferlen].data[7] = 0;
        txHeader.Identifier = BMS_PACK_CURRENT_CAN_ID + channel;
        canTxBuffer[bufferlen++].header = txHeader;
    }
    int32_t cellSum = pack.cell_voltage_sum_valid ? (int32_t)(pack.cell_voltage_sum_v * 1000) : INT32_MIN;
    const int64_t differenceMv = (int64_t)pack.ivt.u1.raw_value - cellSum;
    int32_t difference = pack.ivt.u1.valid && pack.cell_voltage_sum_valid
            && differenceMv > INT32_MIN && differenceMv <= INT32_MAX ? (int32_t)differenceMv : INT32_MIN;
    memcpy(canTxBuffer[bufferlen].data, &cellSum, 4);
    memcpy(canTxBuffer[bufferlen].data + 4, &difference, 4);
    txHeader.Identifier = BMS_PACK_CROSSCHECK_CAN_ID;
    canTxBuffer[bufferlen++].header = txHeader;

    // --- CHARGER CONFIG CAN MESSAGE --- //
    BMS_CAN_GetChargerMsg(&chargerConfig, canTxBuffer[bufferlen].data);
    txHeader.Identifier = CHARGER_CONFIG_CAN_ID;
    canTxBuffer[bufferlen].header = txHeader;
    bufferlen++;

    *len = bufferlen;
    *buff = canTxBuffer;
}


BMS_StatusTypeDef BMS_CheckTemps(void)
{
    return (BMS_StatusFlags & BMS_ERR_TEMP);
}

BMS_StatusTypeDef BMS_CheckVoltage(void)
{
    return (BMS_StatusFlags & BMS_ERR_VOLTAGE);
}

BMS_StatusTypeDef BMS_CheckCurrent(void)
{
    IVTS_Snapshot sensor;
    BMS_CAN_GetIVTSnapshot(&sensor);
    if (!sensor.current.valid || sensor.current.raw_value < BMS_PACK_CURRENT_MIN_MA
            || sensor.current.raw_value > BMS_PACK_CURRENT_MAX_MA) return BMS_ERR_CURRENT;
    return (BMS_StatusFlags & BMS_ERR_CURRENT);
}

BMS_StatusTypeDef BMS_CheckCommsFault(void)
{
    return (BMS_StatusFlags & BMS_ERR_COMMS) | BMS_CheckAdbmsData();
}

void BMS_SetCommsFault(bool state)
{
    if (state)
        BMS_StatusFlags |= BMS_ERR_COMMS;   // Set fault
    else
        BMS_StatusFlags &= ~BMS_ERR_COMMS;  // Clear fault
}


BMS_StatusTypeDef BMS_UpdateStatusFlags(void)
{
    const float MAX_PACK_VOLTAGE = BMS_MV_TO_V(BMS_PACK_OV_MV);
    const float MIN_PACK_VOLTAGE = BMS_MV_TO_V(BMS_PACK_UV_MV);

    const float MAX_CURRENT = BMS_MA_TO_A(BMS_PACK_CURRENT_MAX_MA);
    const float MIN_CURRENT = BMS_MA_TO_A(BMS_PACK_CURRENT_MIN_MA);

    const float MAX_VOLTAGE = BMS_MV_TO_V(BMS_CELL_OV_MV);
    const float MIN_VOLTAGE = BMS_MV_TO_V(BMS_CELL_UV_MV);

    const float MAX_IC_VOLTAGE = BMS_MV_TO_V(BMS_BOARD_OV_MV);
    const float MIN_IC_VOLTAGE = BMS_MV_TO_V(BMS_BOARD_UV_MV);

    const float MAX_TEMP = BMS_TEMP_MAX_C;
    const float MIN_TEMP = BMS_TEMP_MIN_C;

    const float MAX_IC_TEMP = BMS_DIE_TEMP_MAX_C;
    const float MIN_IC_TEMP = BMS_DIE_TEMP_MIN_C;

    BMS_StatusTypeDef status = BMS_OK;
    BMS_StatusTypeDef returnStatus = BMS_OK;
    const bool cellFresh = bms_cellDataFresh(dischargeVoltageType);
    const bool auxFresh = bms_auxDataFresh();
    if (!cellFresh || !auxFresh) returnStatus |= BMS_ERR_COMMS;

    BMS_PackMeasurements pack;
    BMS_GetPackMeasurements(&pack);
    if (!pack.ivt.u1.valid || pack.voltage_u1_v > MAX_PACK_VOLTAGE || pack.voltage_u1_v < MIN_PACK_VOLTAGE)
        returnStatus |= BMS_ERR_VOLTAGE;
    // Preserve the signed sensor current. The existing limits are symmetric;
    // this does not assign positive current to charge or discharge.
    if (!pack.ivt.current.valid || pack.current_a > MAX_CURRENT || pack.current_a < MIN_CURRENT)
        returnStatus |= BMS_ERR_CURRENT;
    if (!pack.ivt.current.valid || !pack.ivt.u1.valid) returnStatus |= BMS_ERR_IVT;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        // TEMPn is a logical connector index, independent of sensor placement.
        for (int c = 0; c < TOTAL_TEMP; c++)
        {
            float cellTemp = ic_ad68.temp_cell[ic][c];

            if (c < TOTAL_CELL)
            {
                float cellVoltage = ic_ad68.v_cell[dischargeVoltageType][ic][c];
                if (!cellFresh || !isfinite(cellVoltage) || cellVoltage > MAX_VOLTAGE || cellVoltage < MIN_VOLTAGE)
                {
                    printfDma("CELL VOLTAGE FAULT: SEG %d, CELL %d, %f \n", ic+1, c+1, cellVoltage);
                    BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                    status |= BMS_ERR_VOLTAGE;
                }
            }

            if (!auxFresh || !ic_ad68.temp_valid[ic][c] || !isfinite(cellTemp)
                    || cellTemp > MAX_TEMP || cellTemp < MIN_TEMP)
            {
                printfDma("TEMP FAULT: SEG %d, TEMP%d, valid %d, %f C\n",
                        ic+1, c+1, ic_ad68.temp_valid[ic][c], cellTemp);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status |= BMS_ERR_TEMP;
            }

            if (status == BMS_OK)
            {
                BIT_CLEAR(ic_ad68.isCellFaultDetected[ic], c);
            }

            returnStatus |= status;
            status = BMS_OK;
        }

        float icVoltage = ic_ad68.v_segment[ic];
        float icTemp = ic_ad68.temp_ic[ic];

        if (!auxFresh || !ic_ad68.v_segment_valid[ic] || !isfinite(icVoltage)
                || icVoltage > MAX_IC_VOLTAGE || icVoltage < MIN_IC_VOLTAGE)
        {
            printfDma("IC VOLTAGE FAULT: SEG %d, %f \n", ic+1, icVoltage);
            ic_common.isFaultDetected[ic] = true;
            status |= BMS_ERR_VOLTAGE;
        }

        if (!auxFresh || !ic_ad68.temp_ic_valid[ic] || !isfinite(icTemp)
                || icTemp > MAX_IC_TEMP || icTemp < MIN_IC_TEMP)
        {
            printfDma("IC TEMP FAULT: SEG %d, %f \n", ic+1, icTemp);
            ic_common.isFaultDetected[ic] = true;
            status |= BMS_ERR_TEMP;
        }

        if (status == BMS_OK)
        {
            ic_common.isFaultDetected[ic] = false;
        }

        returnStatus |= status;
        status = BMS_OK;
    }

    BMS_StatusFlags = returnStatus;
    return returnStatus;
}



BMS_StatusTypeDef BMS_ProgramLoop(void)
{
    if (dischargeStopRequested && bms_stopDischarge() != BMS_OK) return BMS_ERR_COMMS;
    BMS_StatusTypeDef status;
//    bms_wakeupChain();
//    if ((status = bms_readCellVoltage(VOLTAGE_C_FIL)))  return status;
    if (bms_wakeupChain() != BMS_OK) return bms_acquisitionFailed();
    if ((status = bms_getAuxMeasurement())) return status;

    if (bms_wakeupChain() != BMS_OK) return bms_acquisitionFailed();
    if ((status = bms_balancingMeasureVoltage()))       return status;

    // Only balancing/charging if status is OK
    status = BMS_UpdateStatusFlags();

    if (enableBalancing && (status == BMS_OK)) // Only if everything is OK, we enable balancing
    {
        if (bms_wakeupChain() != BMS_OK) return bms_acquisitionFailed();
        status = bms_startBalancing(balancingThreshold);
    }

    return status;
}




bool BMS_IsCharging(void)
{
    return !chargerConfig.disable_charging;
}


void BMS_EnableCharging(bool enabled)
{
    // A button/API request cannot bypass known software faults or stale data.
    if (enabled && (BMS_StatusFlags != BMS_OK || BMS_CheckPackSensor() != BMS_OK
            || BMS_CheckAdbmsData() != BMS_OK || BMS_CheckCurrent() != BMS_OK)) enabled = false;
    chargerConfig.disable_charging = !enabled;   // Inverted logic because config is disable = 1

    char *state = (chargerConfig.disable_charging)? "Disabled" : "Enabled";
    printfDma("Charger Status: %s\n", state);
}

void BMS_ToggleBalancing(void)
{
    BMS_EnableBalancing(!enableBalancing);
}


void BMS_EnableBalancing(bool enabled)
{
    enableBalancing = enabled && BMS_BALANCING_CONFIG_CONFIRMED && BMS_BALANCING_MAX_PWM > 0;
    if (!enableBalancing) {
        // Button/timer callbacks may request a stop; blocking SPI stays in the
        // foreground, where its HAL deadlines can advance.
        bms_clearDischargeState();
        dischargeStopRequested = true;
    }
}


void BMS_ChargingButtonLogic(void)
{
    // Check for charger status and enables charging
    // or disables charger if charging is enabled

    bool chargerEnabled = !chargerConfig.disable_charging;

    if (chargerEnabled)
    {
        BMS_EnableCharging(false);
        return;
    }

    bool statusOK = true;
    if (!HAL_GPIO_ReadPin(SDC_IN_GPIO_Port, SDC_IN_Pin)) statusOK = false;  // SDC is not connected
//    if (chargerStatus.hardware_fault != false)      statusOK = false;
//    if (chargerStatus.over_temp_fault != false)     statusOK = false;
//    if (chargerStatus.input_voltage_fault != false) statusOK = false;
    if (chargerStatus.output_voltage < BMS_MV_TO_V(BMS_CHARGER_MIN_OUTPUT_MV)) statusOK = false;

    if (statusOK)
    {
        BMS_EnableCharging(true);
    }
    else
    {
        printfDma("Charger NOT OK to start Charging \n");
    }
}




void BMS_WriteFaultSignal(bool state)
{
    static bool currState = 1;
    if (currState != state)
    {
        char *stateStr = (state)? "ON " : "OFF";
        printfDma("FAULT SIGNAL UPDATE: %s\n", stateStr);

        HAL_GPIO_WritePin(FAULT_CTRL_GPIO_Port, FAULT_CTRL_Pin, state); // If mosfet is ON, Fault == TRUE
        currState = state;
    }
}




