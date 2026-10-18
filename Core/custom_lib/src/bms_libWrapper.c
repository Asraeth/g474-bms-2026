/*
 * bms_libWrapper.c
 *
 * The measurement cycle (BMS_ProgramLoop) and what is built from it: limit checks for the safety module,
 * balancing, the SoC update and the CAN frames.
 *
 * One cycle on the ADBMS6830B chain (datasheet Rev. 0):
 *   RSTCC              command counters to 0; every read is checked against the commands sent since
 *   RDCFGA             configuration A read back: a slave with REFON off has reset since the last cycle
 *   WRCFGA             configuration A rewritten every cycle (a slave that reset gets REFON and its GPIOs back),
 *                      ADCV after it if a slave reset (its continuous C-ADC is off)
 *   ADAX               AUX conversion (GPIO temperatures, V+, die temperature), collected at the end
 *   ADSV, RDSVx        S-ADC cell voltages (the C-ADC runs continuously from start-up, but isn't read)
 *   ADSV OW even/odd   open wire check, if enabled
 *   PLAUX1, RDAUXx     AUX results
 *   WRCFGB, WRPWMx     balancing
 *
 * Pack topology, pinout and limits come from bms_config.h.
 */


#include "bms_libWrapper.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_cmdlist.h"
#include "bms_safety.h"
#include "bms_ivt.h"
#include "bms_gpio.h"
#include "bms_soc.h"
#include "bms_precharge.h"
#include "bms_inverter.h"
#include "bms_settings.h"
#include "bms_health.h"
#include "bms_charger.h"
#include "bms_adc.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "main.h"
#include "uartDMA.h"
#include "bms_can.h"


#define NUM_CHANNELS        16      // ADBMS6830B cell inputs
#define NUM_GPIOS           10
#define NUM_VOLTAGE_GROUPS  6       // RDSVA..RDSVF
#define NUM_AUX_GROUPS      5       // RDAUXA..RDAUXD + RDSTATA
#define AUX_GROUP_D         3       // G10, VMV, VPV
#define AUX_GROUP_STATA     4       // VREF2, ITMP

#define CFGA0_REFON         0x80    // Configuration register A, byte 0: reference stays on

#define ADC_LSB_V           0.00015f
#define ADC_OFFSET_V        1.5f
#define PWM_DUTY_MAX        0b1111  // 100 %

#define CAN_MEAS_FRAMES     CFG_CAN_MEAS_FRAMES
#define CAN_VALUES_PER_FRAME 4
#define CAN_INVALID_INT16   INT16_MIN   // Unused slot in a packed frame, or value not available
#define CAN_INVALID_INT32   INT32_MIN   // Value not available
#define CAN_UNKNOWN_UINT16  0xFFFF


typedef enum
{
    REG_CONFIG_A,
    REG_CONFIG_B,
    REG_PWM_A,
    REG_PWM_B,
    REG_SID,
} RegisterTypes;


typedef struct
{
    // From/For Config Registers
    ad68_cfa_t cfa_Tx   [TOTAL_IC];
    ad68_cfb_t cfb_Tx   [TOTAL_IC];

    ad68_pwma_t pwma    [TOTAL_IC];
    ad68_pwmb_t pwmb    [TOTAL_IC];

    // S-ADC cell voltages
    float v_cell        [TOTAL_IC][CFG_CELLS_PER_SLAVE];
    // Calculated Values
    float v_cell_sum    [TOTAL_IC];
    float v_cell_min    [TOTAL_IC];
    float v_cell_max    [TOTAL_IC];

    // From AUX measurement
    float v_segment     [TOTAL_IC];
    float temp_cell     [TOTAL_IC][CFG_TEMPS_PER_SLAVE];
    float temp_ic       [TOTAL_IC];
    float raw_aux[TOTAL_IC][CFG_TEMPS_PER_SLAVE];
    uint8_t raw_valid[TOTAL_IC], celsius_valid[TOTAL_IC];

    // flag stored in bits, one per cell
    uint16_t isDischarging          [TOTAL_IC];
    uint16_t isCellFaultDetected    [TOTAL_IC];
    uint16_t isOpenWire             [TOTAL_IC];
    float    owRatioMin             [TOTAL_IC];     // Lowest open-wire/normal ratio of the intact inputs (for calibration)

    bool ledRed     [TOTAL_IC];
    bool ledGreen   [TOTAL_IC];

} Ic_ad68;


typedef struct
{
    bool isFaultDetected    [TOTAL_IC];
    bool isCommsError       [TOTAL_IC];

    // Pack status
    float v_pack_total;
    float v_pack_min;
    float v_pack_max;
    float t_pack_min;
    float t_pack_max;
} Ic_common;


// IVT U1 averaged over the cell conversion, for the U1 vs sum-of-cells plausibility check. Under load
// the pack voltage moves with the current, so both have to come from the same moment; conversions
// during which the current moved more than CFG_PACK_V_PLAUSIBILITY_MAX_DI_A aren't used.
static float measU1_V = 0.0f;
static bool  measU1Valid = false;

// Measurement data handed from the main loop to the CAN interrupt
typedef struct
{
    uint32_t id;
    uint8_t  data[8];
} CanFrame;

typedef struct
{
    float cellMin_V;
    float cellMax_V;
    float tempMin_C;
    float tempMax_C;
    float sumOfCells_V;
} PackSummary;


static Ic_common   ic_common;
static Ic_ad68     ic_ad68;
static Ic_ad68 committedAd68;
static Ic_common committedCommon;
static BMS_AcquisitionStatus acquisition;
static BMS_CommandCounterChain failedCounterState;
static volatile bool dischargeStopRequested;
static uint32_t pecErrors[TOTAL_IC], counterErrors[TOTAL_IC];

// isoSPI transfer buffers
static uint8_t  txData[TOTAL_IC][DATA_LEN];
static uint8_t  rxData[TOTAL_IC][DATA_LEN];
static uint16_t rxPec[TOTAL_IC];
static uint8_t  rxCc[TOTAL_IC];

// Double buffered: the main loop fills the back buffer, then swaps
static CanFrame    measFrames[2][CAN_MEAS_FRAMES];
static uint32_t    measFrameCount[2] = {0};
static PackSummary packSummary[2] = {0};
static volatile uint8_t measFront = 0;

static CanTxMsg canTxBuffer[CAN_BUFFER_LEN] = {0};

static const uint8_t cellChannels[CFG_CELLS_PER_SLAVE] = CFG_SLAVE_CELL_CHANNELS;
static const uint8_t tempGpios[CFG_TEMPS_PER_SLAVE]    = CFG_SLAVE_TEMP_GPIOS;

static const uint8_t* const readSVoltageCmd[NUM_VOLTAGE_GROUPS] = { RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF };
static const uint8_t* const readAuxCmd[NUM_AUX_GROUPS]          = { RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA };

static uint8_t voltageGroupMask = 0;   // S-ADC groups holding a used cell channel

// Setpoints from the runtime settings (BMS_Settings_Get) every time the command is sent
ChargerConfiguration chargerConfig = {
        .max_current = CFG_CHARGER_CURRENT_A,
        .target_voltage = CFG_CHARGER_VOLTAGE_V,
        .disable_charging = 1,
};

static volatile bool enableBalancing = CFG_BALANCING_ENABLED;

static bool     isDischargeActive = false;     // PWM discharge may be on: cleared once a stop went out without an SPI error
static volatile uint32_t loopTimeMs = 0;


void BMS_GetAcquisitionStatus(BMS_AcquisitionStatus *out)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=acquisition;
    uint32_t now=HAL_GetTick();
    out->cellsValid=out->cellsValid && now-out->cellTick<CFG_MEAS_STALE_TIMEOUT_MS;
    out->auxValid=out->auxValid && now-out->auxTick<CFG_MEAS_STALE_TIMEOUT_MS;
    __set_PRIMASK(mask);
}
bool BMS_ControlDataValid(void)
{
    BMS_AcquisitionStatus status; BMS_GetAcquisitionStatus(&status);
    return status.cellsValid && status.auxValid && status.celsiusValid;
}
float BMS_GetMinCellTempC(void) { return packSummary[measFront].tempMin_C; }
float BMS_GetMaxCellVoltageV(void) { return packSummary[measFront].cellMax_V; }
void BMS_RequestSafeOutputs(void)
{
    chargerConfig.disable_charging=1;
    dischargeStopRequested=true;
}


static float rawToVoltage(const uint8_t* p) {return BMS_ADC_Voltage(p);}


// Configuration registers as a device holds them after reset (read back with RDCFGA / RDCFGB)
static void bms_resetConfig(void)
{
    static const uint8_t cfaReset[DATA_LEN] = { 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00 };
    static const uint8_t cfbReset[DATA_LEN] = { 0x00, 0xF8, 0x7F, 0x00, 0x00, 0x00 };

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        memcpy(&ic_ad68.cfa_Tx[ic], cfaReset, DATA_LEN);
        memcpy(&ic_ad68.cfb_Tx[ic], cfbReset, DATA_LEN);
    }
}


// Writes one register group to every device
static bool bms_writeRegister(RegisterTypes regType)
{
    const uint8_t* command;

    switch (regType)
    {
        case REG_CONFIG_A:
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                memcpy(txData[ic], &ic_ad68.cfa_Tx[ic], DATA_LEN);
            }
            command = WRCFGA;
            break;

        case REG_CONFIG_B:
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                memcpy(txData[ic], &ic_ad68.cfb_Tx[ic], DATA_LEN);
            }
            command = WRCFGB;
            break;

        case REG_PWM_A:
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                memcpy(txData[ic], &ic_ad68.pwma[ic], DATA_LEN);
            }
            command = WRPWMA;
            break;

        case REG_PWM_B:
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                memcpy(txData[ic], &ic_ad68.pwmb[ic], DATA_LEN);
            }
            command = WRPWMB;
            break;

        default:
            Error_Handler();
            return false;
    }

    return bms_transmitData(command, txData);
}


// GPO bits: 0 = pull-down on, 1 = pull-down off. Temperature inputs must stay at 1.
static void bms_applyGpo(void)
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t gpo = 0x3FF;

        if (BMS_PROFILE_LEDS_ENABLED && ic_ad68.ledRed[ic] == CFG_SLAVE_LED_ACTIVE_LOW)
        {
            BIT_CLEAR(gpo, CFG_SLAVE_LED_RED_GPIO - 1);
        }
        if (BMS_PROFILE_LEDS_ENABLED && ic_ad68.ledGreen[ic] == CFG_SLAVE_LED_ACTIVE_LOW)
        {
            BIT_CLEAR(gpo, CFG_SLAVE_LED_GREEN_GPIO - 1);
        }

        gpo |= BMS_TEMP_GPIO_MASK; // analogue inputs always released
        ic_ad68.cfa_Tx[ic].gpo1to8  = gpo & 0xFF;
        ic_ad68.cfa_Tx[ic].gpo9to10 = (gpo >> 8) & 0x3;
    }
}


void BMS_SetSlaveLeds(uint8_t slave, bool red, bool green)
{
    if (slave >= TOTAL_IC)
    {
        return;
    }
    ic_ad68.ledRed[slave]   = red;
    ic_ad68.ledGreen[slave] = green;
    committedAd68.ledRed[slave]=red; committedAd68.ledGreen[slave]=green;
}


static BMS_StatusTypeDef bms_readRegister(RegisterTypes regType);
static bool bms_stopDischarge(void);


// C-ADC in continuous mode (datasheet "Cell Discharge with Cell Measurements and Diagnostics"): with RD = 0 and
// CONT = 1, PWM discharge carries on and isn't affected by DCP
static bool bms_startAdcvCont(void)
{
    ADCV_t cmd = ADCV_default;
    cmd.CONT = 1;
    cmd.RSTF = 1;       // Reset the IIR filter
    cmd.OW   = 0b00;
    cmd.RD   = 0;

    return bms_transmitCmd((const uint8_t *)&cmd);
}


BMS_StatusTypeDef bms_init(void)
{
    bms_resetConfig();

    voltageGroupMask = 0;
    for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
    {
        voltageGroupMask |= (uint8_t)(1U << ((cellChannels[c] - 1) / 3));
    }

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        ic_ad68.cfa_Tx[ic].refon = 0b1;
        ic_ad68.cfa_Tx[ic].fc = 0b001;    // IIR filter corner 110 Hz
    }
    bms_applyGpo();

    printfDma("\n --- BMS INIT --- \n");

    if (!bms_wakeupChain() || !bms_transmitCmd(RSTCC)) return BMS_ERR_COMMS;
    //                             // Command counters from 0 (checked on every read)
    if (bms_readRegister(REG_SID) == BMS_ERR_COMMS)     // Every device answers
    {
        return BMS_ERR_COMMS;
    }

    if (!bms_stopDischarge() || !bms_writeRegister(REG_CONFIG_A) || !bms_startAdcvCont())
    {
        return BMS_ERR_COMMS;
    }
    if (!bms_delayMsActive(12)) return BMS_ERR_COMMS; //              // First C-ADC results after 8 ms; keeps the isoSPI ports awake meanwhile

    committedAd68=ic_ad68; committedCommon=ic_common;
    return BMS_OK;
}


static void bms_printRawData(uint8_t data[TOTAL_IC][DATA_LEN], const uint8_t cc[TOTAL_IC])
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("IC%d: ", ic+1);
        for (int j = 0; j < DATA_LEN; j++)
        {
            printfDma("0x%02X, ", data[ic][j]);
        }
        printfDma("CC: %d |   ", cc[ic]);
    }
    printfDma("\n\n");
}


// PEC and command counter of a read. Marks the devices that failed in ic_common.isCommsError.
static bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], const uint16_t pec[TOTAL_IC], const uint8_t cc[TOTAL_IC])
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

    bool pecFailed[TOTAL_IC];
    memcpy(pecFailed,errorIndex,sizeof(pecFailed));
    bool counterFailed=bms_checkRxCounter(cc,pecFailed,errorIndex);
    for (int ic=0;ic<TOTAL_IC;ic++) {
        if (pecFailed[ic]) pecErrors[ic]++;
        else if (errorIndex[ic]) counterErrors[ic]++;
    }
    if (faultDetected) acquisition.pecFault=true;
    if (counterFailed && !faultDetected) { acquisition.counterFault=true; printfDma("WARNING! COMMAND COUNTER MISMATCH\n"); }
    faultDetected |= counterFailed;
    return faultDetected;
}


// Reads one register group from every device into rxData and checks the PECs and command counters
static bool bms_readGroup(const uint8_t* cmd)
{
    if (!bms_receiveData(cmd, rxData, rxPec, rxCc))
    {
        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            ic_common.isCommsError[ic] = true;
        }
        printfDma("WARNING! SPI ERROR\n");
        return false;
    }

    return !bms_checkRxFault(rxData, rxPec, rxCc);
}


// Reads a register group and prints it (start-up check and debugging)
static BMS_StatusTypeDef bms_readRegister(RegisterTypes regType)
{
    const char* title;
    const uint8_t* cmd;
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
        Error_Handler();
        return BMS_ERR_COMMS;
    }

    if (!bms_readGroup(cmd))
    {
        return BMS_ERR_COMMS;
    }

    printfDma("%s: \n", title);
    bms_printRawData(rxData, rxCc);

    return BMS_OK;
}


// Reads the S-ADC groups that hold used cells. vOut is indexed by channel (0 = C1).
static BMS_StatusTypeDef bms_readSVoltages(float vOut[TOTAL_IC][NUM_CHANNELS])
{
    for (int g = 0; g < NUM_VOLTAGE_GROUPS; g++)
    {
        if (!(voltageGroupMask & (1U << g)))
        {
            continue;
        }

        if (!bms_readGroup(readSVoltageCmd[g]))
        {
            return BMS_ERR_COMMS;
        }

        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            for (int k = 0; k < 3; k++)
            {
                int ch = g * 3 + k;
                if (ch < NUM_CHANNELS)      // Group F only holds C16
                {
                    vOut[ic][ch] = rawToVoltage(&rxData[ic][k * 2]);
                }
            }
        }
    }
    return BMS_OK;
}


// Single shot S-ADC conversion. PWM discharge pauses during the conversion (DCP = 0).
static BMS_StatusTypeDef bms_convertS(uint8_t openWire, float vOut[TOTAL_IC][NUM_CHANNELS])
{
    ADSV_t cmd = ADSV_default;
    cmd.CONT = 0;
    cmd.DCP  = 0;
    cmd.OW   = openWire;

    if (!bms_transmitCmd((const uint8_t *)&cmd))
    {
        return BMS_ERR_COMMS;
    }
    if (!bms_transmitPoll(PLSADC, CFG_SADC_POLL_TIMEOUT_MS))
    {
        printfDma("WARNING! S-ADC POLL TIMEOUT\n");
        return BMS_ERR_COMMS;
    }
    return bms_readSVoltages(vOut);
}


static void bms_calculateStats(void)
{
    float total_voltage = 0;
    float pack_min =  999.0;
    float pack_max = -999.0;

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        float min =  999.0;
        float max = -999.0;
        float sum = 0;

        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            float voltage = ic_ad68.v_cell[ic][c];
            sum += voltage;
            if (voltage > max)
            {
                max = voltage;
            }
            if (voltage < min)
            {
                min = voltage;
            }
        }

        total_voltage += sum;
        ic_ad68.v_cell_min[ic] = min;
        ic_ad68.v_cell_max[ic] = max;
        ic_ad68.v_cell_sum[ic] = sum;

        if (max > pack_max)
        {
            pack_max = max;
        }
        if (min < pack_min)
        {
            pack_min = min;
        }
    }

    ic_common.v_pack_total = total_voltage;
    ic_common.v_pack_min = pack_min;
    ic_common.v_pack_max = pack_max;
}


static void bms_printVoltage(void)
{
    printfDma("S Voltage: \n");

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("| %2d |", ic+1);
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            printfDma("%7.4f|", ic_ad68.v_cell[ic][c]);
        }
        printfDma(" Sum %7.3f | OW 0x%04X, intact ratio min %.3f\n", ic_ad68.v_cell_sum[ic], ic_ad68.isOpenWire[ic],
                  ic_ad68.owRatioMin[ic]);
    }
}


static void bms_printTemps(void)
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("| %2d |", ic+1);
        for (int t = 0; t < CFG_TEMPS_PER_SLAVE; t++)
        {
            printfDma("%6.1f |", ic_ad68.temp_cell[ic][t]);
        }
        printfDma("  %.2f C  /  %.2f V \n", ic_ad68.temp_ic[ic], ic_ad68.v_segment[ic]);
    }
}


// Cell voltages, plus the open wire check from the datasheet "Cell Open Wire Detection":
// an S-ADC conversion with the even, then the odd open wire switches on. An intact input reads
// about 90 % of normal, a broken one collapses towards 0 V.
static BMS_StatusTypeDef bms_measureCells(void)
{
    static float vNormal[TOTAL_IC][NUM_CHANNELS];
    static float vOwEven[TOTAL_IC][NUM_CHANNELS];
    static float vOwOdd [TOTAL_IC][NUM_CHANNELS];

    BMS_IVT_SpanStart();
    if (bms_convertS(0b00, vNormal))
    {
        BMS_IVT_SpanEnd(NULL, NULL);
        return BMS_ERR_COMMS;
    }
    float currentRange_A;
    measU1Valid = BMS_IVT_SpanEnd(&measU1_V, &currentRange_A) && currentRange_A <= CFG_PACK_V_PLAUSIBILITY_MAX_DI_A;

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            float v=vNormal[ic][cellChannels[c]-1];
            if (!isfinite(v)) return BMS_ERR_COMMS;
            ic_ad68.v_cell[ic][c] = v;
        }
    }

    if (CFG_OPEN_WIRE_CHECK_ENABLED)
    {
        if (bms_convertS(CFG_ADSV_OW_EVEN, vOwEven) || bms_convertS(CFG_ADSV_OW_ODD, vOwOdd))
        {
            return BMS_ERR_COMMS;
        }

        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            uint16_t openWire = 0;
            float ratioMin = 1.0f;
            for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
            {
                int ch = cellChannels[c] - 1;
                bool isEvenChannel = ((ch + 1) % 2) == 0;
                float vOw = isEvenChannel ? vOwEven[ic][ch] : vOwOdd[ic][ch];

                if (vOw < vNormal[ic][ch] * CFG_OPEN_WIRE_MIN_RATIO)
                {
                    BIT_SET(openWire, c);
                }
                else if (vNormal[ic][ch] > 0.5f && vOw / vNormal[ic][ch] < ratioMin)
                {
                    ratioMin = vOw / vNormal[ic][ch];
                }
            }
            ic_ad68.isOpenWire[ic] = openWire;
            ic_ad68.owRatioMin[ic] = ratioMin;
        }
    }

    bms_calculateStats();

    if (CFG_DEBUG_PRINT_CELLS)
    {
        bms_printVoltage();
    }

    return BMS_OK;
}


// Sensor voltage to temperature: linear between the points of CFG_TEMP_TABLE_V / _C (voltage falling as the
// temperature rises). Outside the table: CFG_TEMP_INVALID_C.
static float convertCellTemp(float cellVoltage)
{
    static const float tempValues[]    = CFG_TEMP_TABLE_C;
    static const float voltageValues[] = CFG_TEMP_TABLE_V;
    static const int   numDataPoints   = sizeof(tempValues) / sizeof(tempValues[0]);
    _Static_assert(sizeof(tempValues) == sizeof(voltageValues), "Temperature table size mismatch");

    if (!isfinite(cellVoltage) || cellVoltage > voltageValues[0]+0.0000751f || cellVoltage < voltageValues[numDataPoints - 1]-0.0000751f)
    {
        return CFG_TEMP_INVALID_C;      // Open or shorted sensor
    }

    cellVoltage=fminf(voltageValues[0],fmaxf(voltageValues[numDataPoints-1],cellVoltage));
    int idx;
    for (idx = 0; idx < numDataPoints - 2; idx++)
    {
        if (cellVoltage > voltageValues[idx + 1]) break;
    }

    float x1 = voltageValues[idx];
    float x2 = voltageValues[idx + 1];
    float y1 = tempValues[idx];
    float y2 = tempValues[idx + 1];

    return y1 + (cellVoltage - x1) * (y2 - y1) / (x2 - x1);
}


// The AUX ADC is separate from the S-ADCs (datasheet "Polling Methods"), so the temperature
// conversion runs while the cell voltages are measured, and is collected afterwards.
static BMS_StatusTypeDef bms_startAuxMeasurement(void)
{
    ADAX_t cmd = ADAX_default;     // All channels, no open wire current sources

    if (!bms_transmitCmd((const uint8_t *)&cmd))
    {
        return BMS_ERR_COMMS;
    }
    return BMS_OK;
}


static BMS_StatusTypeDef bms_readAuxMeasurement(void)
{
    static float vGpio[TOTAL_IC][NUM_GPIOS];

    if (!bms_transmitPoll(PLAUX1, CFG_AUX_POLL_TIMEOUT_MS))
    {
        printfDma("WARNING! AUX POLL TIMEOUT\n");
        return BMS_ERR_COMMS;
    }

    for (int g = 0; g < NUM_AUX_GROUPS; g++)
    {
        if (!bms_readGroup(readAuxCmd[g]))
        {
            return BMS_ERR_COMMS;
        }

        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            const uint8_t* data = rxData[ic];

            if (g < AUX_GROUP_STATA)
            {
                for (int k = 0; k < 3; k++)
                {
                    int gpio = g * 3 + k;
                    if (gpio < NUM_GPIOS)
                    {
                        vGpio[ic][gpio] = rawToVoltage(&data[k * 2]);
                    }
                }
            }

            if (g == AUX_GROUP_D)
            {
                ic_ad68.v_segment[ic] = rawToVoltage(&data[4]) * 25.0f;                // VPV
            }
            else if (g == AUX_GROUP_STATA)
            {
                int16_t raw=(int16_t)((uint16_t)data[2]|((uint16_t)data[3]<<8));
                ic_ad68.temp_ic[ic] = raw==INT16_MIN || raw==INT16_MAX ? NAN
                    : (float)(((double)raw*0.00015+1.5)/0.0075-273.0);       // ITMP
            }
        }
    }

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        ic_ad68.raw_valid[ic]=ic_ad68.celsius_valid[ic]=0;
        for (int t=0;t<CFG_TEMPS_PER_SLAVE;t++) {
            float raw=vGpio[ic][tempGpios[t]-1];
            ic_ad68.raw_aux[ic][t]=raw;
            bool rawValid=isfinite(raw);
            if (rawValid) ic_ad68.raw_valid[ic]|=(uint8_t)(1U<<t);
            float c=BMS_PROFILE_CALIBRATION_CONFIRMED && rawValid ? convertCellTemp(raw) : NAN;
            ic_ad68.temp_cell[ic][t]=c;
            if (isfinite(c) && c!=CFG_TEMP_INVALID_C) ic_ad68.celsius_valid[ic]|=(uint8_t)(1U<<t);
        }
        if (!isfinite(ic_ad68.v_segment[ic]) || !isfinite(ic_ad68.temp_ic[ic])) return BMS_ERR_COMMS;
    }

    if (CFG_DEBUG_PRINT_TEMPS)
    {
        bms_printTemps();
    }

    return BMS_OK;
}


// Discharge duty (0..15) of one cell input (channel 0 = C1). PWM group A holds inputs 1-12, group B 13-16, two
// per byte with the lower input in the low nibble (datasheet Tables 95, 96; the layout of ad68_pwma_t / _b_t).
static void bms_setPwm(int ic, int channel, uint8_t dutyCycle)
{
    if (channel < 0 || channel >= NUM_CHANNELS)
    {
        return;
    }
    uint8_t* group = (channel < 12) ? (uint8_t*)&ic_ad68.pwma[ic] : (uint8_t*)&ic_ad68.pwmb[ic];
    int n = channel % 12;
    int shift = (n % 2) * 4;
    group[n / 2] = (uint8_t)((group[n / 2] & ~(0x0F << shift)) | ((dutyCycle & 0x0F) << shift));
}


// Balances the cells above dischargeThreshold
static bool bms_startDischarge(float dischargeThreshold)
{
    if (!BMS_CAN_BALANCE || !BMS_ControlDataValid() || !isfinite(dischargeThreshold)) return bms_stopDischarge();
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        memset(&ic_ad68.pwma[ic],0,sizeof(ic_ad68.pwma[ic]));
        memset(&ic_ad68.pwmb[ic],0,sizeof(ic_ad68.pwmb[ic]));
        ic_ad68.cfb_Tx[ic].dcc=0;
        // A cell above the threshold starts balancing; one already balancing carries on until it's
        // CFG_BALANCE_HYSTERESIS_V below it, so a cell sitting at the threshold doesn't switch every cycle.
        // Power all discharging cells on this slave would dissipate at 100 % duty.
        uint16_t balance = 0;
        float fullDutyPower = 0;
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            float v = ic_ad68.v_cell[ic][c];
            bool was = BIT_CHECK(ic_ad68.isDischarging[ic], c);
            if (v > dischargeThreshold || (was && v > dischargeThreshold - CFG_BALANCE_HYSTERESIS_V))
            {
                BIT_SET(balance, c);
                fullDutyPower += v * v / CFG_BALANCE_RESISTOR_OHM;
            }
        }

        // 4 bit PWM, 937 ms period. Scale so the slave stays within its heat budget.
        uint8_t dutyCycle = PWM_DUTY_MAX;
        if (fullDutyPower > CFG_BALANCE_MAX_POWER_PER_SLAVE_W)
        {
            dutyCycle = (uint8_t)(PWM_DUTY_MAX * CFG_BALANCE_MAX_POWER_PER_SLAVE_W / fullDutyPower);
        }

        if (dutyCycle>CFG_BALANCE_MAX_PWM) dutyCycle=CFG_BALANCE_MAX_PWM;
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            if (BIT_CHECK(balance, c) && dutyCycle > 0)
            {
                BIT_SET(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, cellChannels[c] - 1, dutyCycle);
            }
            else
            {
                BIT_CLEAR(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, cellChannels[c] - 1, 0b0000);    // Turn off PWM discharge for that cell
            }
        }

        // PWM discharge only runs while the discharge timer (DCTO, restarted by every WRCFGB) hasn't expired:
        // if the BMS stops talking, the slaves stop balancing CFG_BALANCE_DCTO_MIN minutes later
        ic_ad68.cfb_Tx[ic].dcto = CFG_BALANCE_DCTO_MIN;     // Minutes (DTRNG = 0)
        ic_ad68.cfb_Tx[ic].dtmen = 0;                       // No discharge timer monitor
    }

    bool ok=bms_writeRegister(REG_CONFIG_B);
    ok=bms_writeRegister(REG_PWM_A) && ok;
    ok=bms_writeRegister(REG_PWM_B) && ok;
    ok=bms_transmitCmd(UNMUTE) && ok;
    isDischargeActive=false;
    for(int ic=0;ic<TOTAL_IC;ic++) if(ic_ad68.isDischarging[ic]) isDischargeActive=true;
    if (!ok) { (void)bms_stopDischarge(); return false; }
    return true;
}


// All PWM duties and the discharge timer to 0. Returns false if a write failed (some slaves may still be
// balancing): isDischargeActive stays set and the stop is sent again next cycle.
static bool bms_stopDischarge(void)
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for (int c = 0; c < NUM_CHANNELS; c++)
        {
            bms_setPwm(ic, c, 0b0000);
        }
        ic_ad68.isDischarging[ic] = 0;
        ic_ad68.cfb_Tx[ic].dcto = 0;
        ic_ad68.cfb_Tx[ic].dtmen = 0;
        ic_ad68.cfb_Tx[ic].dcc=0;
        ic_ad68.cfb_Tx[ic].dtrng=0;
    }
    bool ok=bms_transmitCmd(MUTE);
    ok = bms_writeRegister(REG_CONFIG_B) && ok;
    ok = bms_writeRegister(REG_PWM_A) && ok;
    ok = bms_writeRegister(REG_PWM_B) && ok;
    const uint8_t *reads[]={RDCFGB,RDPWMA,RDPWMB};
    if (ok) for (int group=0;group<3;group++) {
        if (!bms_readGroup(reads[group])) { ok=false; break; }
        for (int ic=0;ic<TOTAL_IC;ic++)
            for (int byte=group==0 ? 3 : 0;byte<DATA_LEN;byte++) if (rxData[ic][byte]!=0) ok=false;
    }
    if(ok)BMS_Health_ClearRecovered(BMS_ERR_BALANCE_STOP);
    acquisition.stopFailed=!ok;
    isDischargeActive=!ok && isDischargeActive;
    dischargeStopRequested=false;
    return ok;
}


void bms_softReset(void)
{
    acquisition.cellsValid=acquisition.auxValid=acquisition.celsiusValid=false;
    if (!bms_wakeupChain() || !bms_transmitCmd(SRST)) BMS_RequestSafeOutputs(); //      // Every device to sleep, registers to their reset values
    printfDma("\n  ---  SOFT RESET  ---  \n");
}


// Checks the latest measurements against the limits in bms_config.h.
// Returns the violated BMS_StatusTypeDef bits; debouncing is done by the safety module.
static uint32_t bms_evaluateLimits(void)
{
    const bool charging = BMS_IsCharging();
    const float tempMin = charging ? CFG_CELL_CHARGE_TEMP_MIN_C : CFG_CELL_TEMP_MIN_C;
    const float tempMax = charging ? CFG_CELL_CHARGE_TEMP_MAX_C : CFG_CELL_TEMP_MAX_C;

    uint32_t flags = BMS_OK;
    float tPackMin =  999.0;
    float tPackMax = -999.0;

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t cellFault = 0;
        bool icFault = false;

        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            float cellVoltage = ic_ad68.v_cell[ic][c];
            if (!isfinite(cellVoltage) || cellVoltage > CFG_CELL_OV_V || cellVoltage < CFG_CELL_UV_V)
            {
                BIT_SET(cellFault, c);
                flags |= BMS_ERR_VOLTAGE;
                if(isfinite(cellVoltage))flags|=cellVoltage<CFG_CELL_UV_V ? BMS_ERR_CELL_UV:BMS_ERR_CELL_OV;
            }

            if (BIT_CHECK(ic_ad68.isOpenWire[ic], c))
            {
                BIT_SET(cellFault, c);
                flags |= BMS_ERR_OPEN_WIRE;
            }
        }

        for (int t = 0; t < CFG_TEMPS_PER_SLAVE; t++)
        {
            float cellTemp = ic_ad68.temp_cell[ic][t];
            if (cellTemp > tPackMax) tPackMax = cellTemp;
            if (cellTemp < tPackMin) tPackMin = cellTemp;

            if (!(ic_ad68.celsius_valid[ic] & (1U<<t)) || !isfinite(cellTemp) || cellTemp > tempMax || cellTemp < tempMin)
            {
                /* TEMP connector index is not an assumed mechanical cell location. */
                icFault = true;
                flags |= BMS_ERR_TEMP;
                if(charging)flags|=BMS_ERR_CHARGE_TEMP;
            }
        }

        float icVoltage = ic_ad68.v_segment[ic];
        if (icVoltage > CFG_SLAVE_V_MAX || icVoltage < CFG_SLAVE_V_MIN)
        {
            icFault = true;
            flags |= BMS_ERR_VOLTAGE;
        }

        float icTemp = ic_ad68.temp_ic[ic];
        if (icTemp > CFG_SLAVE_DIE_TEMP_MAX_C || icTemp < CFG_SLAVE_DIE_TEMP_MIN_C)
        {
            icFault = true;
            flags |= BMS_ERR_TEMP;
        }

        // Only print faults when they appear, to keep the UART readable
        uint16_t newCellFaults = cellFault & ~ic_ad68.isCellFaultDetected[ic];
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            if (BIT_CHECK(newCellFaults, c))
            {
                float temp = (c < CFG_TEMPS_PER_SLAVE) ? ic_ad68.temp_cell[ic][c] : 0;
                printfDma("Cell outside limits (sample): SEG %d, CELL %d, %.3f V, %.1f C, OW %d\n", ic+1, c+1,
                          ic_ad68.v_cell[ic][c], temp, BIT_CHECK(ic_ad68.isOpenWire[ic], c) ? 1 : 0);
            }
        }
        if (icFault && !ic_common.isFaultDetected[ic])
        {
            printfDma("Slave outside limits (sample): SEG %d, %.2f V, die %.1f C\n", ic+1, icVoltage, icTemp);
        }

        ic_ad68.isCellFaultDetected[ic] = cellFault;
        ic_common.isFaultDetected[ic] = icFault || (cellFault != 0);
    }

    ic_common.t_pack_min = tPackMin;
    ic_common.t_pack_max = tPackMax;

    return flags;
}


static void putInt16(uint8_t* p, int16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}


static void putInt32(uint8_t* p, int32_t value)
{
    for (int i = 0; i < 4; i++)
    {
        p[i] = (uint8_t)((uint32_t)value >> (8 * i));
    }
}


// Saturating; NaN (never expected) as "not available"
static int16_t toInt16(float value)
{
    if (value != value)    return CAN_INVALID_INT16;
    if (value > 32767.0f)  return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}


// I x U1 in CFG_CAN_POWER_W_PER_BIT steps, rounded. Saturates at +-32767, so INT16_MIN only ever
// means "not available".
static int16_t powerToCan(const IvtReading* ivt)
{
    if (!ivt->powerFresh)
    {
        return CAN_INVALID_INT16;
    }
    float steps = ivt->power_W / CFG_CAN_POWER_W_PER_BIT;
    if (steps >= 32767.0f)  return 32767;
    if (steps <= -32767.0f) return -32767;
    return (int16_t)lroundf(steps);
}


// Remaining energy in Wh, 0xFFFF until the start-up SoC estimate has been taken
static uint16_t remainingToCan(const SocState* soc)
{
    if (!soc->valid)
    {
        return CAN_UNKNOWN_UINT16;
    }
    float wh = soc->remaining_Wh + 0.5f;
    return wh >= (float)(CAN_UNKNOWN_UINT16 - 1) ? CAN_UNKNOWN_UINT16 - 1 : (uint16_t)wh;
}


// Builds the slave, cell voltage and temperature frames in the back buffer and hands it to the CAN interrupt
static void bms_publishCanFrames(const BMS_AcquisitionStatus *next)
{
    uint8_t back = measFront ^ 1;
    CanFrame* frames = measFrames[back];
    uint32_t n = 0;

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for(int k=0;k<CFG_CAN_T_FRAMES_PER_SLAVE;k++) {
            CanFrame *raw=&frames[n++]; raw->id=CFG_CAN_ID_RAW_AUX_BASE+ic*CFG_CAN_T_FRAMES_PER_SLAVE+k;
            for(int j=0;j<4;j++) {int t=k*4+j;putInt16(raw->data+j*2,t<CFG_TEMPS_PER_SLAVE && next->auxValid && (ic_ad68.raw_valid[ic]&(1U<<t)) ? toInt16(ic_ad68.raw_aux[ic][t]*1000):CAN_INVALID_INT16);}
        }
        CanFrame *diag=&frames[n++];diag->id=CFG_CAN_ID_DIAGNOSTIC_BASE+ic;
        putInt16(diag->data,(int16_t)(pecErrors[ic]>65535?65535:pecErrors[ic]));putInt16(diag->data+2,(int16_t)(counterErrors[ic]>65535?65535:counterErrors[ic]));
        diag->data[4]=next->auxValid?ic_ad68.raw_valid[ic]:0;diag->data[5]=next->auxValid?ic_ad68.celsius_valid[ic]:0;
        diag->data[6]=(uint8_t)next->transportFault;diag->data[7]=next->stopFailed;
        uint8_t isCommsError=ic_common.isCommsError[ic] || !next->cellsValid || !next->auxValid;
        uint8_t isFaultDetected = ic_common.isFaultDetected[ic];

        // --- SLAVE MESSAGE --- //
        CanFrame* f = &frames[n++];
        f->id = CFG_CAN_ID_SLAVE_BASE + ic;
        putInt16(&f->data[0], toInt16(ic_ad68.v_segment[ic] * 100));        // 0.01 V
        putInt16(&f->data[2], toInt16(ic_ad68.temp_ic[ic] * 10));           // 0.1 C
        f->data[4] = (uint8_t)ic_ad68.isDischarging[ic];                    // Bit per cell
        f->data[5] = (uint8_t)ic_ad68.isCellFaultDetected[ic];              // Bit per cell
        f->data[6] = (uint8_t)ic_ad68.isOpenWire[ic];                       // Bit per cell
        f->data[7] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));

        if (isCommsError)
        {
            continue; // Does not send cell voltage and temp as its invalid
        }

        // --- CELL VOLTAGE MESSAGES (4 cells per frame) --- //
        for (int k = 0; k < CFG_CAN_V_FRAMES_PER_SLAVE; k++)
        {
            f = &frames[n++];
            f->id = CFG_CAN_ID_CELL_V_BASE + ic * CFG_CAN_V_FRAMES_PER_SLAVE + k;
            for (int j = 0; j < CAN_VALUES_PER_FRAME; j++)
            {
                int c = k * CAN_VALUES_PER_FRAME + j;
                int16_t value = (c < CFG_CELLS_PER_SLAVE) ? toInt16(ic_ad68.v_cell[ic][c] * 1000) : CAN_INVALID_INT16;
                putInt16(&f->data[j * 2], value);                           // mV
            }
        }

        // --- CELL TEMPERATURE MESSAGES (4 sensors per frame) --- //
        for (int k = 0; k < CFG_CAN_T_FRAMES_PER_SLAVE; k++)
        {
            f = &frames[n++];
            f->id = CFG_CAN_ID_CELL_T_BASE + ic * CFG_CAN_T_FRAMES_PER_SLAVE + k;
            for (int j = 0; j < CAN_VALUES_PER_FRAME; j++)
            {
                int t = k * CAN_VALUES_PER_FRAME + j;
                int16_t value = (t < CFG_TEMPS_PER_SLAVE) ? toInt16((ic_ad68.celsius_valid[ic] & (1U<<t)) ? ic_ad68.temp_cell[ic][t]*10 : NAN) : CAN_INVALID_INT16;
                putInt16(&f->data[j * 2], value);                           // 0.1 C
            }
        }
    }

    measFrameCount[back] = n;
    packSummary[back].cellMin_V    = ic_common.v_pack_min;
    packSummary[back].cellMax_V    = ic_common.v_pack_max;
    packSummary[back].tempMin_C    = ic_common.t_pack_min;
    packSummary[back].tempMax_C    = ic_common.t_pack_max;
    packSummary[back].sumOfCells_V = ic_common.v_pack_total;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    acquisition=*next;
    measFront = back;
    __set_PRIMASK(primask);
}


static void setTxFrame(CanTxMsg* msg, uint32_t id, const uint8_t data[8])
{
    msg->header.Identifier          = id;
    msg->header.IdType              = FDCAN_EXTENDED_ID;
    msg->header.TxFrameType         = FDCAN_DATA_FRAME;
    msg->header.DataLength          = FDCAN_DLC_BYTES_8;
    msg->header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    msg->header.BitRateSwitch       = FDCAN_BRS_OFF;
    msg->header.FDFormat            = FDCAN_CLASSIC_CAN;
    msg->header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    msg->header.MessageMarker       = 0;
    memcpy(msg->data, data, 8);
}


void BMS_GetCanData(CanTxMsg** buff, uint32_t* len)
{
    uint32_t bufferlen = 0;
    uint8_t  front = measFront;
    uint8_t  data[8];

    // The TX FIFO sends in this order, so the pack-level frames go first in case a burst is cut short

    const PackSummary* summary = &packSummary[front];
    IvtReading ivt = BMS_IVT_GetReading();

    // --- PACK MESSAGE --- //
    memset(data, 0, sizeof(data));
    putInt16(&data[0], toInt16(ivt.voltageValid ? ivt.voltage_V * 10 : NAN));            // 0.1 V
    putInt16(&data[2], toInt16(ivt.currentValid ? ivt.current_A * 10 : NAN));            // 0.1 A, positive = discharge
    putInt16(&data[4], toInt16(summary->sumOfCells_V * 10));    // 0.1 V
    data[6] = (uint8_t)((!ivt.currentValid << 0) | (!ivt.voltageValid << 1) | (ivt.sensorError << 2));
    setTxFrame(&canTxBuffer[bufferlen++], CFG_CAN_ID_PACK, data);

    // --- STATUS MESSAGE --- //
    uint32_t age = BMS_Safety_GetMeasurementAgeMs();
    InverterLimits limits = BMS_Inverter_GetLimits();
    memset(data, 0, sizeof(data));
    data[0] = (uint8_t)BMS_Safety_GetActiveFlags();
    data[1] = (uint8_t)((BMS_Safety_IsFaultOutputActive() << 0) |
                        (!BMS_Safety_IsFaultOutputActive() << 1) |     // TSAC ready relay
                        (enableBalancing << 2) |
                        (BMS_IsCharging() << 3) |
                        (BMS_GPIO_IsSdcClosed() << 4) |
                        (BMS_PROFILE_IS_BENCH << 5) |                    // Bench build
                        (limits.tempDerating << 6) |                    // Inverter limits derated for temperature
                        (limits.powerLimited << 7));                    // Inverter discharge held to the power limit
    putInt16(&data[2], (int16_t)(age > 0x7FFF ? 0x7FFF : age));          // ms
    putInt16(&data[4], (int16_t)(loopTimeMs > 0x7FFF ? 0x7FFF : loopTimeMs));
    data[6] = (uint8_t)BMS_Precharge_GetState();
    data[7] = BMS_Precharge_GetPercent();                               // U2 / U1 in %
    setTxFrame(&canTxBuffer[bufferlen++], CFG_CAN_ID_STATUS, data);

    // --- SUMMARY MESSAGE --- //
    memset(data, 0, sizeof(data));
    putInt16(&data[0], toInt16(summary->cellMin_V * 1000));     // mV
    putInt16(&data[2], toInt16(summary->cellMax_V * 1000));     // mV
    putInt16(&data[4], toInt16(summary->tempMin_C * 10));       // 0.1 C
    putInt16(&data[6], toInt16(summary->tempMax_C * 10));       // 0.1 C
    setTxFrame(&canTxBuffer[bufferlen++], CFG_CAN_ID_SUMMARY, data);

    // --- ENERGY MESSAGE --- //
    SocState soc = BMS_SoC_Get();
    memset(data, 0, sizeof(data));
    putInt16(&data[0], powerToCan(&ivt));                                               // + = discharge
    putInt16(&data[2], (int16_t)remainingToCan(&soc));                                  // Wh, 0xFFFF = unknown
    putInt16(&data[4], (int16_t)(uint16_t)(soc.valid ? soc.soc_percent * 10.0f + 0.5f : 0xFFFF));  // 0.1 %, 0xFFFF = unknown
    putInt16(&data[6], toInt16(soc.used_Wh));                                           // Wh since start-up
    setTxFrame(&canTxBuffer[bufferlen++], CFG_CAN_ID_ENERGY, data);

    memset(data,0,8);data[0]=CFG_PROTOCOL_VERSION;data[1]=CFG_CONFIG_VERSION;data[2]=BMS_BOARD_PROFILE;data[3]=TOTAL_IC;data[4]=CFG_CELLS_PER_SLAVE;data[5]=CFG_TEMPS_PER_SLAVE;data[6]=NUM_CHANNELS;data[7]=BMS_PROFILE_IS_BENCH;
    setTxFrame(&canTxBuffer[bufferlen++],CFG_CAN_ID_IDENTITY,data);
    BMS_AcquisitionStatus a;BMS_GetAcquisitionStatus(&a);
    memset(data,0,8);putInt32(data,(int32_t)BMS_Safety_GetActiveFlags());data[4]=(uint8_t)(a.cellsValid|(a.auxValid<<1)|(a.celsiusValid<<2)|(ivt.configurationVerified<<3)|(ivt.currentValid<<4)|(ivt.voltageValid<<5)|(ivt.voltage2Valid<<6)|(ivt.energyValid<<7));
    putInt16(data+5,(int16_t)(age>65535?65535:age));data[7]=(uint8_t)a.generation;
    setTxFrame(&canTxBuffer[bufferlen++],CFG_CAN_ID_QUALITY,data);
    IVTS_Snapshot raw;BMS_IVT_GetSnapshot(&raw);memset(data,0,8);
    data[0]=raw.current.raw_status;data[1]=raw.u1.raw_status;data[2]=raw.u2.raw_status;data[3]=raw.energy.raw_status;
    data[4]=(uint8_t)(ivt.currentValid|(ivt.voltageValid<<1)|(ivt.voltage2Valid<<2)|(ivt.energyValid<<3)|(raw.transport_error<<4));data[5]=(uint8_t)raw.restart_epoch;
    uint32_t gaps=raw.current.counter_gap_count+raw.u1.counter_gap_count+raw.u2.counter_gap_count+raw.energy.counter_gap_count;
    putInt16(data+6,(int16_t)(gaps>65535?65535:gaps));setTxFrame(&canTxBuffer[bufferlen++],CFG_CAN_ID_IVT_DIAGNOSTIC,data);
    memset(data,0,8);data[0]=chargerStatus.state;putInt16(data+1,(int16_t)chargerStatus.fault_bits);data[3]=(uint8_t)(BMS_Charger_Healthy()|(BMS_IsCharging()<<1));
    putInt16(data+4,toInt16(chargerStatus.output_voltage*10));putInt16(data+6,toInt16(chargerStatus.output_current*10));setTxFrame(&canTxBuffer[bufferlen++],CFG_CAN_ID_CHARGER,data);

    // --- SLAVE, CELL VOLTAGE AND TEMPERATURE MESSAGES (from the last measurement cycle) --- //
    for (uint32_t i = 0; i < measFrameCount[front]; i++)
    {
        setTxFrame(&canTxBuffer[bufferlen++], measFrames[front][i].id, measFrames[front][i].data);
    }

    *len = bufferlen;
    *buff = canTxBuffer;
}


// Hottest cell of the last measurement cycle
float BMS_GetMaxCellTempC(void)
{
    return BMS_ControlDataValid() ? packSummary[measFront].tempMax_C : NAN;
}


// Charger command for CAN3, sent with every CAN1 burst
void BMS_GetChargerCanMsg(CanTxMsg* msg)
{
    uint8_t data[8];
    BmsSettings settings = BMS_Settings_Get();
    chargerConfig.max_current    = settings.chargeCurrent_A;
    chargerConfig.target_voltage = settings.chargeVoltage_V;
    BMS_CAN_GetChargerMsg(&chargerConfig, data);
    setTxFrame(msg, CHARGER_CONFIG_CAN_ID, data);
}


// Fast frames, built in the timer interrupt every CFG_CAN_FAST_PERIOD_MS from the latest IVT values.
// Current and voltages at the IVT's own resolution (1 mA, 1 mV); INT32_MIN / INT16_MIN = not available.
uint32_t BMS_GetFastCanData(CanTxMsg msgs[CAN_FAST_FRAME_COUNT])
{
    IvtReading ivt = BMS_IVT_GetReading();
    SocState   soc = BMS_SoC_Get();
    uint8_t    data[8];

    putInt32(&data[0], ivt.currentValid ? ivt.current_mA : CAN_INVALID_INT32);      // mA, + = discharge
    putInt32(&data[4], ivt.voltageValid ? ivt.voltage_mV : CAN_INVALID_INT32);      // mV, U1 = pack
    setTxFrame(&msgs[0], CFG_CAN_ID_FAST_1, data);

    putInt32(&data[0], ivt.voltage2Valid ? ivt.voltage2_mV : CAN_INVALID_INT32);    // mV, U2 = vehicle side
    putInt16(&data[4], powerToCan(&ivt));                                           // + = discharge
    putInt16(&data[6], (int16_t)remainingToCan(&soc));                              // Wh, 0xFFFF = unknown
    setTxFrame(&msgs[1], CFG_CAN_ID_FAST_2, data);

    return CAN_FAST_FRAME_COUNT;
}


uint32_t BMS_GetLoopTimeMs(void)
{
    return loopTimeMs;
}


// Mean of the cells inside CFG_SOC_CELL_MIN_V..MAX_V, as the Battery Tracker's Pack SoC
static bool bms_meanCellVoltage(float* mean)
{
    float sum = 0;
    int count = 0;
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            float v = ic_ad68.v_cell[ic][c];
            if (v > CFG_SOC_CELL_MIN_V && v < CFG_SOC_CELL_MAX_V)
            {
                sum += v;
                count++;
            }
        }
    }
    *mean = count > 0 ? sum / (float)count : 0.0f;
    return count > 0;
}


// Reads configuration register A back: REFON is set from bms_init on, so a slave reading it off has been reset
// (power or sleep). Found on the bench: its readings carry on, PECs and command counters fine, with its C-ADC
// stopped. Returns false if the read failed.
static bool bms_findResetSlaves(bool* anyReset)
{
    *anyReset = false;
    if (!bms_readGroup(RDCFGA))
    {
        return false;
    }
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        if (!(rxData[ic][0] & CFGA0_REFON))
        {
            printfDma("Slave %d was reset (supply dip?): configuration and C-ADC restored\n", ic + 1);
            *anyReset = true;
        }
    }
    return true;
}


static bool bms_isBalancingAllowed(uint32_t limitFlags)
{
    if (!CFG_BALANCING_ENABLED || !enableBalancing)
    {
        return false;
    }
    if (CFG_BALANCE_ONLY_WHILE_CHARGING && !BMS_IsCharging())
    {
        return false;
    }
    IvtReading ivt=BMS_IVT_GetReading();
    // Only if every asynchronous control input is usable.
    return ivt.currentValid && ivt.voltageValid && BMS_CAN_BALANCE && BMS_ControlDataValid() && (limitFlags == BMS_OK) && !BMS_Safety_IsFaultOutputActive();
}


BMS_StatusTypeDef BMS_ProgramLoop(void)
{
    uint32_t sampleTick = HAL_GetTick();
    BMS_StatusTypeDef status = BMS_OK;
    uint32_t sampleFlags=BMS_OK;
    BMS_AcquisitionStatus pending=acquisition;

    ic_ad68=committedAd68; ic_common=committedCommon;
    acquisition.pecFault=false; acquisition.counterFault=false;
    if (!bms_wakeupChain() || !bms_transmitCmd(RSTCC)) status=BMS_ERR_COMMS;
    if (status==BMS_OK && dischargeStopRequested && !bms_stopDischarge()) status=BMS_ERR_COMMS;

    // A slave that reset since the last cycle (a dip of its supply) answers again at once, but at its reset values
    bool slaveReset = false;
    if (status==BMS_OK && !bms_findResetSlaves(&slaveReset))
    {
        status = BMS_ERR_COMMS;
    }

    // Rewritten every cycle: restores REFON and the GPIO outputs if a slave has reset, and applies LED changes.
    // The continuous C-ADC is otherwise only started by bms_init.
    bms_applyGpo();
    if (status == BMS_OK && !bms_writeRegister(REG_CONFIG_A))
    {
        status = BMS_ERR_COMMS;
    }
    if (status == BMS_OK && slaveReset && !bms_startAdcvCont())
    {
        status = BMS_ERR_COMMS;
    }

    if (status == BMS_OK)
    {
        status = bms_startAuxMeasurement();     // ~18 ms conversion, runs in the background
    }
    if (status == BMS_OK)
    {
        status = bms_measureCells();            // ~30 ms with the open wire check
    }
    if (status == BMS_OK)
    {
        status = bms_readAuxMeasurement();      // Normally finished by now
    }

    if (status == BMS_OK)
    {
        pending.cellsValid=pending.auxValid=true;pending.celsiusValid=true;
        for(int ic=0;ic<TOTAL_IC;ic++)if(ic_ad68.celsius_valid[ic]!=((1U<<CFG_TEMPS_PER_SLAVE)-1U))pending.celsiusValid=false;
        pending.cellTick=pending.auxTick=sampleTick;
        uint32_t limitFlags = bms_evaluateLimits();

        // IVT U1 vs the sum of cells, both from this conversion (T 11.9 plausibility). A conversion that
        // can't be compared (current moving) holds the last result, so it neither starts nor clears a violation.
        static bool implausibleReported = false;
        static bool implausible = false;
        if (measU1Valid)
        {
            implausible = CFG_PACK_V_PLAUSIBILITY_V > 0 &&
                          fabsf(measU1_V - ic_common.v_pack_total) > CFG_PACK_V_PLAUSIBILITY_V;
        }
        if (implausible)
        {
            limitFlags |= BMS_ERR_VOLTAGE;
            if (!implausibleReported)
            {
                printfDma("PACK VOLTAGE IMPLAUSIBLE: IVT U1 %.1f V, sum of cells %.1f V\n",
                          (double)measU1_V, (double)ic_common.v_pack_total);
            }
        }
        implausibleReported = implausible;



        if (bms_isBalancingAllowed(limitFlags))
        {
            if (!bms_startDischarge(ic_common.v_pack_min + CFG_BALANCE_THRESHOLD_V)) status=BMS_ERR_COMMS;
        }
        else if (isDischargeActive)
        {
            if (!bms_stopDischarge()) status=BMS_ERR_COMMS;
        }
        if (status==BMS_OK) {
            committedAd68=ic_ad68; committedCommon=ic_common;
            pending.generation++;
            pending.pecFault=pending.counterFault=false;pending.transportFault=BMS_TRANSPORT_OK;pending.stopFailed=acquisition.stopFailed;
            sampleFlags=limitFlags;
        }
    }
    if (status!=BMS_OK) {
        BMS_TransportFault primary=bms_getTransportFault();
        bms_getCounterState(&failedCounterState);
        bool pec=acquisition.pecFault, counter=acquisition.counterFault;
        acquisition.cellsValid=acquisition.auxValid=acquisition.celsiusValid=false;
        bool mayBeDischarging=isDischargeActive;
        bool stopped=bms_stopDischarge();
        if (!stopped && mayBeDischarging) BMS_Health_Latch(BMS_ERR_BALANCE_STOP);
        ic_ad68=committedAd68; ic_common=committedCommon;
        /* Recovery never restores a previous actuation request. */
        for(int ic=0;ic<TOTAL_IC;ic++) {
            memset(&ic_ad68.pwma[ic],0,sizeof(ic_ad68.pwma[ic]));
            memset(&ic_ad68.pwmb[ic],0,sizeof(ic_ad68.pwmb[ic]));
            ic_ad68.cfb_Tx[ic].dcc=0; ic_ad68.cfb_Tx[ic].dcto=0;
            ic_ad68.isDischarging[ic]=0;
        }
        committedAd68=ic_ad68;
        for (int ic=0;ic<TOTAL_IC;ic++) ic_common.isCommsError[ic]=true;
        acquisition.transportFault=primary; acquisition.pecFault=pec; acquisition.counterFault=counter;
        pending=acquisition;
        BMS_RequestSafeOutputs();
    }

    // SoC: open-circuit estimate needs a complete measurement and no load; energy is tracked regardless
    IvtReading ivt = BMS_IVT_GetReading();
    float meanCell = 0.0f;
    bool meanValid = (status == BMS_OK) && bms_meanCellVoltage(&meanCell);
    bool noLoad=ivt.currentValid && !BMS_IsCharging() && fabsf(ivt.current_A)<CFG_SOC_NO_LOAD_CURRENT_A;
    BMS_SoC_Update(meanCell, meanValid, noLoad, &ivt);

    bms_publishCanFrames(&pending);
    if(status==BMS_OK)BMS_Safety_ReportMeasurement(sampleTick,sampleFlags);
    loopTimeMs = HAL_GetTick() - sampleTick;

    return status;
}


bool BMS_IsCharging(void)
{
    return !chargerConfig.disable_charging;
}


void BMS_EnableCharging(bool enabled)
{
    IvtReading ivt=BMS_IVT_GetReading();
    enabled=enabled && ivt.currentValid && ivt.voltageValid && BMS_GPIO_IsSdcClosed() && BMS_CAN_ENABLE_CHARGER && BMS_Charger_Healthy() && BMS_GetMinCellTempC()>=CFG_CELL_CHARGE_TEMP_MIN_C && BMS_GetMaxCellTempC()<=CFG_CELL_CHARGE_TEMP_MAX_C && BMS_ControlDataValid() && !BMS_Safety_IsFaultOutputActive();
    chargerConfig.disable_charging = !enabled;   // Inverted logic because config is disable = 1

    char *state = (chargerConfig.disable_charging)? "Disabled" : "Enabled";
    printfDma("Charger Status: %s\n", state);
}

void BMS_EnableBalancing(bool enabled)
{
    enableBalancing=enabled && BMS_CAN_BALANCE;
    if (!enableBalancing) dischargeStopRequested=true;
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
    if (!BMS_GPIO_IsSdcClosed()) statusOK = false;                  // SDC is not connected
    if (BMS_Safety_IsFaultOutputActive()) statusOK = false;
//    if (chargerStatus.hardware_fault != false)      statusOK = false;
//    if (chargerStatus.over_temp_fault != false)     statusOK = false;
//    if (chargerStatus.input_voltage_fault != false) statusOK = false;
    if (chargerStatus.output_voltage < 300.0f)      statusOK = false;

    if (statusOK)
    {
        BMS_EnableCharging(true);
    }
    else
    {
        printfDma("Charger NOT OK to start Charging \n");
    }
}
