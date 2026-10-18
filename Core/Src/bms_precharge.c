/*
 * bms_precharge.c
 *
 * Precharge and bypass (AIR) control.
 *
 * A 0 -> 1 change of the TS request (CAN1) starts precharge if the AMS isn't faulted and the SDC is
 * closed. Once the vehicle side of the AIRs (IVT U2) reaches CFG_PRECHARGE_TARGET_PERCENT of a
 * plausible battery voltage (IVT U1), the bypass closes, and the precharge relay opens
 * CFG_PRECHARGE_OVERLAP_MS later. Both open immediately on a deactivate request, an AMS fault or an
 * open SDC; a timeout or lost IVT voltages during precharge also open both and need a new request.
 *
 * Runs in the safety tick; the request arrives in the CAN interrupt. Both share priority 0, so they
 * never interrupt each other.
 */

#include "bms_precharge.h"
#include "bms_safety.h"
#include "bms_health.h"
#include "bms_gpio.h"
#include "bms_ivt.h"
#include "bms_libWrapper.h"
#include "main.h"
#include "uartDMA.h"
#include <math.h>


static volatile PrechargeState state = PRECHARGE_IDLE;
static volatile uint8_t        percent = 0;
static uint32_t                stateTick = 0;

static volatile bool     requestActive = false;
static volatile bool     requestRising = false;
static volatile bool     requestSeen = false;
static volatile uint32_t requestTick = 0;


void BMS_Precharge_HandleRequest(bool activate)
{
    if (activate && !requestActive)
    {
        requestRising = true;
    }
    if (!activate) BMS_Health_ClearRecovered(BMS_ERR_PRECHARGE);
    requestActive = activate;
    requestSeen   = true;
    requestTick   = HAL_GetTick();
}


static void enter(PrechargeState next, const char* reason)
{
    if (next != state && reason)
    {
        printfDma("Precharge: %s\n", reason);
    }
    if (next == PRECHARGE_FAILED) BMS_Health_Latch(BMS_ERR_PRECHARGE);
    state = next;
    stateTick = HAL_GetTick();
}


void BMS_Precharge_Tick(void)
{
    uint32_t now = HAL_GetTick();
    IvtReading ivt = BMS_IVT_GetReading();

    if (CFG_TS_REQUEST_TIMEOUT_MS > 0 && requestSeen && requestActive &&
        now - requestTick > CFG_TS_REQUEST_TIMEOUT_MS)
    {
        requestActive = false;      // VCU stopped sending
    }

    bool rising = requestRising;
    requestRising = false;

    bool allowed = BMS_CAN_ACTUATE_CONTACTORS && BMS_ControlDataValid() && !BMS_Safety_IsFaultOutputActive() && BMS_GPIO_IsSdcClosed();
    bool batteryPlausible = ivt.voltageValid && isfinite(ivt.voltage_V) && !ivt.sensorError &&
                            ivt.voltage_V >= CFG_PACK_V_MIN && ivt.voltage_V <= CFG_PACK_V_MAX;
    bool vehicleValid = ivt.voltage2Valid && isfinite(ivt.voltage2_V) && ivt.voltage2_V>=0 && ivt.voltage2_V<=CFG_PACK_V_MAX && !ivt.voltage2Error;

    if (batteryPlausible && vehicleValid)
    {
        float p = ivt.voltage2_V / ivt.voltage_V * 100.0f;
        percent = (uint8_t)(p < 0.0f ? 0.0f : (p > 255.0f ? 255.0f : p));
    }

    if (state!=PRECHARGE_IDLE && state!=PRECHARGE_FAILED && (!batteryPlausible || !vehicleValid)) {
        enter(PRECHARGE_FAILED,"IVT invalid during precharge/active");
    }
    switch (state)
    {
    case PRECHARGE_IDLE:
    case PRECHARGE_FAILED:
        if (rising && requestActive && allowed && batteryPlausible && vehicleValid)
        {
            enter(PRECHARGE_CHARGING, "started");
        }
        else if(rising && requestActive && allowed && (!batteryPlausible || !vehicleValid)) {enter(PRECHARGE_FAILED,"invalid input at request");}
        else if (rising)
        {
            printfDma("Precharge: request ignored, AMS faulted or SDC open\n");
        }
        break;

    case PRECHARGE_CHARGING:
        if (!requestActive || !allowed)
        {
            enter(PRECHARGE_IDLE, !requestActive ? "deactivated" : "aborted, AMS fault or SDC open");
        }
        else if (!batteryPlausible || !vehicleValid)
        {
            enter(PRECHARGE_FAILED, "failed, IVT voltages missing or implausible");
        }
        else if (now - stateTick > CFG_PRECHARGE_TIMEOUT_MS)
        {
            enter(PRECHARGE_FAILED, "failed, timeout");
        }
        else if (ivt.voltage2_V * 100.0f >= ivt.voltage_V * CFG_PRECHARGE_TARGET_PERCENT)
        {
            enter(CFG_PRECHARGE_OVERLAP_MS > 0 ? PRECHARGE_CLOSING : PRECHARGE_ACTIVE, "complete, bypass closed");
        }
        break;

    case PRECHARGE_CLOSING:
        if (!requestActive || !allowed)
        {
            enter(PRECHARGE_IDLE, !requestActive ? "deactivated" : "aborted, AMS fault or SDC open");
        }
        else if (now - stateTick >= CFG_PRECHARGE_OVERLAP_MS)
        {
            enter(PRECHARGE_ACTIVE, NULL);
        }
        break;

    case PRECHARGE_ACTIVE:
        if (!requestActive || !allowed)
        {
            enter(PRECHARGE_IDLE, !requestActive ? "deactivated" : "aborted, AMS fault or SDC open");
        }
        break;
    }

    bool precharge = (state == PRECHARGE_CHARGING || state == PRECHARGE_CLOSING);
    bool bypass    = (state == PRECHARGE_CLOSING  || state == PRECHARGE_ACTIVE);

    // Open before close, so the outputs never pass through an unintended combination
    if (!bypass)    BMS_GPIO_WriteBypass(false);
    if (!precharge) BMS_GPIO_WritePrecharge(false);
    if (precharge)  BMS_GPIO_WritePrecharge(true);
    if (bypass)     BMS_GPIO_WriteBypass(true);
}


PrechargeState BMS_Precharge_GetState(void)
{
    return state;
}


uint8_t BMS_Precharge_GetPercent(void)
{
    return percent;
}


bool BMS_Precharge_IsTsActive(void)
{
    return state != PRECHARGE_IDLE && state != PRECHARGE_FAILED;
}
