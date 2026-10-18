/*
 * bms_soc.c
 *
 * State of charge and energy.
 *
 * Start-up: with no load, SoC comes from the open-circuit voltage model in "Battery Tracker.xlsx"
 * (Chebyshev fit of the cell's voltage-SoC curve) applied to the mean cell voltage, and the
 * remaining energy is SoC x pack energy.
 * Running: the remaining energy follows the IVT-S energy counter (from I and U1). The counter is
 * only trusted between consecutive readings; after a gap (IVT restart reloads a count stored up
 * to 15 minutes earlier) or an impossible jump, counting restarts from the new value.
 */

#include "bms_soc.h"
#include "main.h"
#include "uartDMA.h"

#include <math.h>


static SocState soc = {0};          // Written by the main loop, read by the CAN interrupt

static bool     haveEnergyRef = false;
static int32_t  lastEnergyWh = 0;
static uint32_t lastEnergyTick = 0;
static uint32_t lastRestartEpoch;


static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}


float BMS_SoC_FromCellVoltage(float cellVoltage)
{
    static const float coeffs[] = CFG_SOC_COEFFS;
    static const int   numCoeffs = sizeof(coeffs) / sizeof(coeffs[0]);

    float t = 2.0f * (cellVoltage - CFG_SOC_VMIN_V) / (CFG_SOC_VMAX_V - CFG_SOC_VMIN_V) - 1.0f;
    t = clampf(t, -1.0f, 1.0f);

    // Chebyshev recurrence T(k+1) = 2t T(k) - T(k-1), equal to the sheet's COS(k * ACOS(t))
    float tPrev = 1.0f;
    float tCur  = t;
    float sum   = coeffs[0] * tPrev + coeffs[1] * tCur;
    for (int k = 2; k < numCoeffs; k++)
    {
        float tNext = 2.0f * t * tCur - tPrev;
        sum  += coeffs[k] * tNext;
        tPrev = tCur;
        tCur  = tNext;
    }

    return clampf(100.0f * sum, 0.0f, 100.0f);
}


static void trackEnergy(SocState* s, const IvtReading* ivt)
{
    if (lastRestartEpoch!=ivt->restartEpoch) { haveEnergyRef=false; printfDma("Energy counter restart epoch changed, counting restarted\n"); lastRestartEpoch=ivt->restartEpoch; }
    if (!ivt->energyValid || ivt->energyTick == lastEnergyTick)
    {
        return;     // No new counter value
    }

    if (s->valid && haveEnergyRef)
    {
        uint32_t gap_ms = ivt->energyTick - lastEnergyTick;
        int64_t  delta  = (int64_t)ivt->energy_Wh - lastEnergyWh;     // A corrupted counter may jump its whole range

        // Most the pack can deliver or take in that time, plus the counter's 1 Wh resolution
        float maxPower_W = fmaxf(CFG_MAX_DISCHARGE_CURRENT_A, CFG_MAX_CHARGE_CURRENT_A) * CFG_PACK_V_MAX;
        float maxDelta   = maxPower_W * (float)gap_ms / 3600000.0f + 2.0f;

        if (gap_ms > CFG_ENERGY_MAX_GAP_MS)
        {
            printfDma("Energy counter gap of %lu ms, counting restarted\n", (unsigned long)gap_ms);
        }
        else if (fabsf((float)delta) > maxDelta)
        {
            printfDma("Energy counter jumped %.0f Wh, counting restarted\n", (double)delta);
        }
        else
        {
            s->used_Wh += (float)delta;
        }
    }

    lastEnergyWh   = ivt->energy_Wh;
    lastEnergyTick = ivt->energyTick;
    haveEnergyRef  = s->valid;
}


void BMS_SoC_Update(float meanCellVoltage, bool meanValid, bool noLoad, const IvtReading* ivt)
{
    SocState s = BMS_SoC_Get();

    if (!s.valid && meanValid && noLoad)
    {
        s.initialSoc_percent = BMS_SoC_FromCellVoltage(meanCellVoltage);
        s.used_Wh = 0.0f;
        s.valid = true;
        printfDma("SoC estimate %.1f %% from mean cell voltage %.3f V\n",
                  (double)s.initialSoc_percent, (double)meanCellVoltage);
    }

    trackEnergy(&s, ivt);

    if (s.valid)
    {
        float initial_Wh = s.initialSoc_percent / 100.0f * CFG_PACK_ENERGY_WH;
        s.remaining_Wh = clampf(initial_Wh - s.used_Wh, 0.0f, CFG_PACK_ENERGY_WH);
        s.soc_percent  = s.remaining_Wh / CFG_PACK_ENERGY_WH * 100.0f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    soc = s;
    __set_PRIMASK(primask);
}


SocState BMS_SoC_Get(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    SocState s = soc;
    __set_PRIMASK(primask);
    return s;
}
