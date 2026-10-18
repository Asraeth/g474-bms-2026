/*
 * bms_inverter.c
 *
 * DC current limits for the inverter (DTI HV500), which the BMS owns. DTI Inverter CAN Manual v2.4:
 * 0x0A "Set max. DC current" and 0x0B "Set max. DC brake current", int16 big-endian in bytes 0-1,
 * A x 10, unused bytes 0xFF, brake current negative.
 *
 *   discharge = min(CFG_MAX_DISCHARGE_CURRENT_A derated, CFG_INVERTER_POWER_LIMIT_W / pack voltage)
 *               (FS 2027 EV 2.2.1: 80 kW at the TSAC outlet), derated linearly to
 *               CFG_INVERTER_DERATE_CURRENT_A between CFG_INVERTER_DERATE_START_C and _END_C (hottest cell)
 *   regen     = CFG_INVERTER_REGEN_CURRENT_A, no power limit (EV 2.2.3: regen is unrestricted), derated
 *               the same way between the CFG_INVERTER_REGEN_DERATE temperatures
 *   0 A both ways while the AMS fault output is active
 *
 * BMS_Limits reports the same limits on CAN1, with the discharge power limit (current limit x pack
 * voltage): 80 kW when the power limit applies, less when the current limit or derating does.
 *
 * With CFG_POWER_WINDOW_LIMITER_ENABLED the flat power limit is replaced by a moving-window limiter
 * (bms_config.h): power may go above CFG_POWER_WINDOW_LIMIT_W as long as the 500 ms moving average
 * stays at or below it, which is how FS 2027 D 10.4.1 judges a power violation.
 *
 * Runtime settings (bms_settings.h, from the GUI) can lower these: the power limit (and with it the
 * window limit and floor), the discharge and regen currents, the temperature at which both are fully
 * derated (both ramps move down with it), and switch regen off (0 A brake current).
 *
 * Pack voltage is IVT U1; without a fresh U1 the highest possible pack voltage is assumed, which gives
 * the lowest current.
 */

#include "bms_inverter.h"
#include "bms_ivt.h"
#include "bms_safety.h"
#include "bms_libWrapper.h"
#include "bms_settings.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define DTI_PACKET_MAX_DC_CURRENT       0x0A
#define DTI_PACKET_MAX_DC_BRAKE_CURRENT 0x0B

static InverterLimits lastLimits;       // Timer interrupt only

// Moving window, one bin per timer tick, oldest at binHead
#define WINDOW_BINS     (CFG_POWER_WINDOW_MS / CFG_TIMER_TICK_MS)
#define TICK_S          (CFG_TIMER_TICK_MS / 1000.0f)
#define WINDOW_S        (CFG_POWER_WINDOW_MS / 1000.0f)
static float    energyBins[WINDOW_BINS];   // Discharge energy per tick (J)
static float    creditBins[WINDOW_BINS];   // Energy left unused (+) or used above the limit (-) per tick (J)
static uint32_t binHead = 0;
static float    sentPower_W = 0.0f;        // Power the inverter may draw under the limit last sent
static float    windowLimit_W = CFG_POWER_WINDOW_LIMIT_W;   // Lowered with the power limit setting
static float    windowFloor_W = CFG_POWER_WINDOW_FLOOR_W;


// Linear from base_A at start_C down to end_A at end_C
static float derate(float base_A, float temp_C, float start_C, float end_C, float end_A)
{
    if (base_A <= end_A)
    {
        return base_A;
    }
    float f = (temp_C - start_C) / (end_C - start_C);
    f = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
    return base_A - (base_A - end_A) * f;
}


// Current that draws at most power_W. The pack voltage moves with the current (U = OCV - R I): above the
// present current it sags (at least CFG_PACK_RESISTANCE_MIN_OHM), below it it recovers (at most
// CFG_PACK_RESISTANCE_MAX_OHM). Returns the current and, in maxPower_W, the most power it can draw.
static float currentForPower(float power_W, float voltage_V, float current_A, float* maxPower_W)
{
    float i = power_W / voltage_V;
    float r = i < current_A ? CFG_PACK_RESISTANCE_MAX_OHM : CFG_PACK_RESISTANCE_MIN_OHM;
    if (r > 0.0f)
    {
        // I x (U + R (I_now - I)) = P  ->  R I^2 - (U + R I_now) I + P = 0, smaller root. No root: the
        // pack can't deliver that power at any current, so the power limit doesn't constrain it
        float b    = voltage_V + r * current_A;
        float disc = b * b - 4.0f * r * power_W;
        i = disc >= 0.0f ? (b - sqrtf(disc)) / (2.0f * r) : INFINITY;
    }
    *maxPower_W = power_W;
    return i;
}


// Most power a current limit can draw: the voltage at that current is at most the present voltage plus
// the recovery from the present current, minus the least sag above it
static float powerAtCurrent(float limit_A, float voltage_V, float current_A)
{
    float change_V = current_A > limit_A ? CFG_PACK_RESISTANCE_MAX_OHM * (current_A - limit_A)
                                         : -CFG_PACK_RESISTANCE_MIN_OHM * (limit_A - current_A);
    return limit_A * (voltage_V + change_V);
}


InverterLimits BMS_Inverter_ComputeLimits(const BmsSettings* s, float maxCellTemp_C, float packVoltage_V, float packCurrent_A,
                                          bool faulted, float powerCap_W)
{
    InverterLimits l = {0};
    l.settingsModified = s->modified;
    if (faulted || !isfinite(maxCellTemp_C) || !isfinite(packVoltage_V) || !isfinite(packCurrent_A) || packVoltage_V<=0 || !isfinite(powerCap_W) || powerCap_W<0)
    {
        l.faulted = true;
        return l;
    }

    // The temperature limit setting moves both derating ramps down by the same amount
    float shift_C   = CFG_INVERTER_DERATE_END_C - s->tempLimit_C;
    l.tempDerating  = maxCellTemp_C > CFG_INVERTER_DERATE_START_C - shift_C;
    l.regenDerating = s->regenEnabled && maxCellTemp_C > CFG_INVERTER_REGEN_DERATE_START_C - shift_C;
    l.discharge_A   = derate(s->dischargeCurrent_A, maxCellTemp_C, CFG_INVERTER_DERATE_START_C - shift_C,
                             CFG_INVERTER_DERATE_END_C - shift_C, CFG_INVERTER_DERATE_CURRENT_A);
    l.charge_A      = !s->regenEnabled ? 0.0f
                    : derate(s->regenCurrent_A, maxCellTemp_C, CFG_INVERTER_REGEN_DERATE_START_C - shift_C,
                             CFG_INVERTER_REGEN_DERATE_END_C - shift_C, CFG_INVERTER_REGEN_DERATE_CURRENT_A);

    if (maxCellTemp_C<CFG_CELL_CHARGE_TEMP_MIN_C || maxCellTemp_C>CFG_CELL_CHARGE_TEMP_MAX_C) l.charge_A=0;
    float capPower_W;
    float power_A = currentForPower(fminf(powerCap_W, s->powerLimit_W), packVoltage_V, packCurrent_A, &capPower_W);
    if (power_A < l.discharge_A)
    {
        l.discharge_A      = power_A;
        l.powerLimited     = true;
        l.dischargePower_W = capPower_W;
    }
    else
    {
        l.dischargePower_W = powerAtCurrent(l.discharge_A, packVoltage_V, packCurrent_A);
    }
    return l;
}


static int16_t toInt16(float value)
{
    if (!isfinite(value)) return 0;
    long v = (long)truncf(value);
    return (int16_t)(v > 32767 ? 32767 : (v < -32767 ? -32767 : v));
}


static void setHeader(CanTxMsg* msg, uint32_t id)
{
    memset(msg, 0, sizeof(*msg));
    msg->header.Identifier          = id;
    msg->header.IdType              = FDCAN_EXTENDED_ID;
    msg->header.TxFrameType         = FDCAN_DATA_FRAME;
    msg->header.DataLength          = FDCAN_DLC_BYTES_8;
    msg->header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    msg->header.BitRateSwitch       = FDCAN_BRS_OFF;
    msg->header.FDFormat            = FDCAN_CLASSIC_CAN;
    msg->header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    msg->header.MessageMarker       = 0;
}


static void setLimitFrame(CanTxMsg* msg, uint8_t packet, float amps)
{
    int16_t value = toInt16(amps * 10.0f);

    setHeader(msg, ((uint32_t)packet << 8) | CFG_INVERTER_NODE_ID);
    memset(msg->data, 0xFF, sizeof(msg->data));                 // Unused bytes 0xFF
    msg->data[0] = (uint8_t)((uint16_t)value >> 8);             // Big-endian
    msg->data[1] = (uint8_t)value;
}


void BMS_Inverter_Tick(void)
{
    IvtReading ivt = BMS_IVT_GetReading();

    // Without fresh IVT data assume the inverter draws its whole limit (the AMS faults shortly anyway)
    float p = ivt.powerFresh ? fmaxf(ivt.power_W, 0.0f) : sentPower_W;

    // Credit for boosting: power the driver left unused while the inverter was below its limit.
    // Power held down by the limit (after a boost, say) earns nothing, so a boost isn't repeated.
    bool  belowLimit = ivt.currentFresh && ivt.current_A < 0.95f * lastLimits.discharge_A;
    float credit_W   = (belowLimit ? fmaxf(windowLimit_W - p, 0.0f) : 0.0f) - fmaxf(p - windowLimit_W, 0.0f);

    energyBins[binHead] = p * TICK_S;
    creditBins[binHead] = credit_W * TICK_S;
    binHead = (binHead + 1) % WINDOW_BINS;
}


// Highest discharge power for the next limit period that keeps every 500 ms window at or below
// the window limit (CFG_POWER_WINDOW_LIMIT_W, or lower with the power limit setting) x 500 ms. Windows ending at t + h hold the measured bins still inside them,
// the limit already sent (until this one takes effect, CFG_POWER_WINDOW_LAG_MS), this limit (until the
// next one takes effect) and then the floor.
static float windowAllowance(float maxPower_W,float total_J,float credit_J)
{
    (void)total_J;(void)credit_J;
    /* Every instantaneous command obeys the electrical ceiling. Unlike the
       former credit-spending predictor this cannot echo a dip 500 ms later. */
    return fminf(maxPower_W,windowLimit_W);
}


void BMS_Inverter_Update(void)
{
    IvtReading ivt = BMS_IVT_GetReading();
    float packVoltage_V = (ivt.voltageValid && ivt.voltage_V > 1.0f) ? ivt.voltage_V : CFG_PACK_V_MAX;
    float packCurrent_A = ivt.currentValid ? fmaxf(ivt.current_A, 0.0f) : 0.0f;
    float maxCellTemp_C = BMS_GetMaxCellTempC();
    bool  faulted       = !BMS_CAN_DRIVE_INVERTER || !ivt.currentValid || !ivt.voltageValid || !BMS_ControlDataValid() || BMS_Safety_IsFaultOutputActive();
    BmsSettings settings = BMS_Settings_Get();

    // A lower power limit setting lowers the window limit, and the floor in proportion
    windowLimit_W = fminf(CFG_POWER_WINDOW_LIMIT_W, settings.powerLimit_W);
    windowFloor_W = CFG_POWER_WINDOW_FLOOR_W * windowLimit_W / CFG_POWER_WINDOW_LIMIT_W;

    float total_J = 0.0f, credit_J = 0.0f;
    for (int i = 0; i < WINDOW_BINS; i++)
    {
        total_J  += energyBins[i];
        credit_J += creditBins[i];
    }

    float powerCap_W = CFG_INVERTER_POWER_LIMIT_W;
    if (CFG_POWER_WINDOW_LIMITER_ENABLED)
    {
        // The window limit already includes the power limit setting, and boosting goes above it
        settings.powerLimit_W = 1e9f;
        // The most the current limit allows, so the boost check knows the ceiling
        InverterLimits uncapped = BMS_Inverter_ComputeLimits(&settings, maxCellTemp_C, packVoltage_V, packCurrent_A,
                                                             faulted, 1e9f);
        powerCap_W = windowAllowance(uncapped.dischargePower_W, total_J, credit_J);
    }

    lastLimits = BMS_Inverter_ComputeLimits(&settings, maxCellTemp_C, packVoltage_V, packCurrent_A, faulted, powerCap_W);
    if (BMS_GetMinCellTempC()<CFG_CELL_CHARGE_TEMP_MIN_C || BMS_GetMaxCellVoltageV()>=CFG_CELL_OV_V) lastLimits.charge_A=0;
    lastLimits.boost       = CFG_POWER_WINDOW_LIMITER_ENABLED && lastLimits.dischargePower_W > windowLimit_W + 1.0f;
    lastLimits.windowAvg_W = total_J / WINDOW_S;
    sentPower_W = lastLimits.dischargePower_W;
}


uint32_t BMS_Inverter_GetCanFrames(CanTxMsg msgs[INVERTER_LIMIT_FRAME_COUNT])
{
    setLimitFrame(&msgs[0], DTI_PACKET_MAX_DC_CURRENT, lastLimits.discharge_A);
    setLimitFrame(&msgs[1], DTI_PACKET_MAX_DC_BRAKE_CURRENT, -lastLimits.charge_A);
    return INVERTER_LIMIT_FRAME_COUNT;
}


// Little-endian like the other BMS frames:
//   int16 discharge power limit (CFG_CAN_POWER_W_PER_BIT), int16 discharge current limit 0.1 A,
//   int16 regen current limit 0.1 A, byte 6 flags, byte 7 500 ms average discharge power 0.5 kW
void BMS_Inverter_GetLimitsFrame(CanTxMsg* msg)
{
    int16_t values[3] = {
        toInt16(lastLimits.dischargePower_W / CFG_CAN_POWER_W_PER_BIT),
        toInt16(lastLimits.discharge_A * 10.0f),
        toInt16(lastLimits.charge_A * 10.0f),
    };

    setHeader(msg, CFG_CAN_ID_LIMITS);
    for (int i = 0; i < 3; i++)
    {
        msg->data[2 * i]     = (uint8_t)((uint16_t)values[i] & 0xFF);
        msg->data[2 * i + 1] = (uint8_t)((uint16_t)values[i] >> 8);
    }
    msg->data[6] = (uint8_t)((lastLimits.tempDerating << 0) | (lastLimits.powerLimited << 1) |
                             (lastLimits.regenDerating << 2) | (lastLimits.faulted << 3) | (lastLimits.boost << 4) |
                             (lastLimits.settingsModified << 5));
    float avg = lastLimits.windowAvg_W / 500.0f + 0.5f;
    msg->data[7] = (uint8_t)(avg > 255.0f ? 255.0f : avg);
}


InverterLimits BMS_Inverter_GetLimits(void)
{
    return lastLimits;
}
