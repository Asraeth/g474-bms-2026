/*
 * bms_safety.c
 *
 * Fault evaluation, AMS fault output, TSAC ready relay and watchdog
 *
 * Two kinds of fault conditions:
 *  - Limit violations (voltage, temperature, current, open wire). Each has an over-limit timer: time
 *    over the limit counts up, time under it counts back down at 1/CFG_FAULT_DECAY_DIVISOR of the rate.
 *    A violation faults once its timer reaches CFG_FAULT_DEBOUNCE_MS, so a steady one faults after the
 *    debounce, one interrupted by short spells under the limit (noise around the limit, pulses) only a
 *    little later, and isolated glitches drain away. It also faults once the last sample without it is
 *    CFG_FAULT_DEADLINE_MS old. A sample that doesn't arrive doesn't clear it.
 *  - Missing data (stale isoSPI measurements, IVT timeouts, no data since start-up). These fault
 *    immediately, the timeout (at most the deadline) already acts as the debounce.
 *
 * The fault output feeds a hardware latch, so it is released again once everything has been clear
 * for CFG_FAULT_CLEAR_DELAY_MS. The TSAC ready relay is only on while the fault output is off.
 * See bms_config.h for the worst-case timing.
 */

#include "bms_safety.h"
#include "bms_gpio.h"
#include "bms_ivt.h"
#include "main.h"
#include "bms_health.h"
#include "bms_libWrapper.h"

#include <math.h>


#define NUM_FLAG_BITS       20

#define IWDG_KEY_RELOAD     0xAAAAU
#define IWDG_KEY_UNLOCK     0x5555U
#define IWDG_KEY_START      0xCCCCU
#define IWDG_PRESCALER_32   0x3U        // LSI (32 kHz) / 32 = ~1 kHz, 1 count per ms


// Over-limit timer of one violation, brought up to date at each sample
typedef struct
{
    uint32_t level;                         // ms, as of tick (at most CFG_FAULT_DEBOUNCE_MS)
    uint32_t tick;                          // Sample it was last brought up to date at
    bool     violated;                      // In that sample
} OverLimitTimer;

typedef struct
{
    bool     measured;                      // At least one complete measurement cycle
    uint32_t lastSampleTick;
    uint32_t flags;                         // Limit violations in the latest sample
    OverLimitTimer timer[NUM_FLAG_BITS];
    uint32_t lastClearTick[NUM_FLAG_BITS];  // Latest sample without it
} MeasurementState;

// Written by the main loop with interrupts disabled, read by the timer interrupt
static MeasurementState meas = {0};

// Only used by the timer interrupt
static bool     ivtStarted = false;
static uint32_t ivtFlagsPrev = 0;
static OverLimitTimer ivtTimer[NUM_FLAG_BITS] = {0};
static uint32_t ivtLastClearTick[NUM_FLAG_BITS] = {0};
static uint32_t lastFaultTick = 0;


// The timer's level at time 'at' (not before its tick), the latest sample's state holding until then
static uint32_t timerLevel(const OverLimitTimer* t, uint32_t at)
{
    uint32_t dt = at - t->tick;
    if ((int32_t)dt < 0)
    {
        dt = 0;
    }
    uint32_t level;
    if (t->violated)
    {
        level = t->level + (dt < CFG_FAULT_DEBOUNCE_MS ? dt : CFG_FAULT_DEBOUNCE_MS);
    }
    else
    {
        uint32_t drained = dt / CFG_FAULT_DECAY_DIVISOR;
        level = t->level > drained ? t->level - drained : 0;
    }
    return level < CFG_FAULT_DEBOUNCE_MS ? level : CFG_FAULT_DEBOUNCE_MS;
}

// A new sample: the previous state counts up to it, then this one holds
static void timerSample(OverLimitTimer* t, uint32_t sampleTick, bool violated)
{
    if ((int32_t)(sampleTick - t->tick) > 0)
    {
        t->level = timerLevel(t, sampleTick);
        t->tick  = sampleTick;
    }
    t->violated = violated;
}

static void timerReset(OverLimitTimer* t, uint32_t tick)
{
    t->level    = 0;
    t->tick     = tick;
    t->violated = false;
}

// Violated in the latest sample, and over the limit long enough or past the deadline from the last good sample
static bool violationDue(uint32_t now, const OverLimitTimer* t, uint32_t lastClearTick)
{
    return t->violated &&
           (timerLevel(t, now) >= CFG_FAULT_DEBOUNCE_MS || now - lastClearTick >= CFG_FAULT_DEADLINE_MS);
}

static volatile uint32_t activeFlags = BMS_ERR_STARTUP;
static volatile bool     faultOutput = true;

// Changes of the fault flags or output, timed by the safety tick, for the UART log
#define EVENT_QUEUE_LEN     16
static SafetyEvent       events[EVENT_QUEUE_LEN];
static volatile uint32_t eventHead = 0;     // Written by the timer interrupt
static volatile uint32_t eventTail = 0;     // Written by the main loop
static volatile uint32_t eventsLost = 0;


void BMS_Watchdog_Start(void)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;     // Don't reset while halted by the debugger

    IWDG->KR  = IWDG_KEY_START;
    IWDG->KR  = IWDG_KEY_UNLOCK;
    IWDG->PR  = IWDG_PRESCALER_32;
    IWDG->RLR = CFG_WATCHDOG_TIMEOUT_MS;

    uint32_t start = HAL_GetTick();
    while ((IWDG->SR != 0) && (HAL_GetTick() - start < 50))
    {
        // Wait for the prescaler and reload registers to update
    }

    IWDG->KR = IWDG_KEY_RELOAD;
}


void BMS_Watchdog_Kick(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}


void BMS_Safety_ReportMeasurement(uint32_t sampleTick, uint32_t conditionFlags)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (int b = 0; b < NUM_FLAG_BITS; b++)
    {
        uint32_t bit = (1UL << b);
        if (!meas.measured)
        {
            meas.lastClearTick[b] = sampleTick;     // Deadlines count from the first measurement
            timerReset(&meas.timer[b], sampleTick);
        }
        if (!(conditionFlags & bit))
        {
            meas.lastClearTick[b] = sampleTick;
        }
        timerSample(&meas.timer[b], sampleTick, (conditionFlags & bit) != 0);
    }

    meas.flags          = conditionFlags;
    meas.lastSampleTick = sampleTick;
    meas.measured       = true;

    __set_PRIMASK(primask);
}


static uint32_t evaluateIvt(uint32_t now, uint32_t* immediate)
{
    IvtReading ivt = BMS_IVT_GetReading();

    if (!ivtStarted)
    {
        ivtStarted = true;
        for (int b = 0; b < NUM_FLAG_BITS; b++)
        {
            ivtLastClearTick[b] = now;          // Deadlines count from start-up
            timerReset(&ivtTimer[b], now);
        }
    }

    static uint32_t lastValidatedTick;
    static bool deadlineStarted;
    if(!deadlineStarted) {lastValidatedTick=now;deadlineStarted=true;}
    if(ivt.currentValid && ivt.voltageValid)lastValidatedTick=now-(ivt.currentAge_ms>ivt.voltageAge_ms?ivt.currentAge_ms:ivt.voltageAge_ms);
    // Acquisition validity includes protocol and configuration. Consumers inhibit immediately;
    // the AMS missing-data deadline retains the lead bounded ride-through policy.
    if (now-lastValidatedTick>=CFG_IVT_FAULT_TIMEOUT_MS)
    {
        *immediate |= BMS_ERR_IVT;
    }

    // Each check only on a fresh reading; without one it keeps its last result
    uint32_t flags = ivtFlagsPrev;
    uint32_t checked = 0;
    uint32_t sampleTick[NUM_FLAG_BITS] = {0};

    if (ivt.currentFresh || ivt.voltageFresh)
    {
        flags = (flags & ~BMS_ERR_IVT) | ((ivt.sensorError) ? BMS_ERR_IVT : 0);
        checked |= BMS_ERR_IVT;
    }
    if (ivt.currentFresh)
    {
        bool over = ivt.current_A > CFG_MAX_DISCHARGE_CURRENT_A || ivt.current_A < -CFG_MAX_CHARGE_CURRENT_A;
        flags = (flags & ~BMS_ERR_CURRENT) | (over ? BMS_ERR_CURRENT : 0);
        checked |= BMS_ERR_CURRENT;
    }
    if (ivt.voltageFresh)
    {
        bool out = ivt.voltage_V > CFG_PACK_V_MAX || ivt.voltage_V < CFG_PACK_V_MIN;
        flags = (flags & ~BMS_ERR_VOLTAGE) | (out ? BMS_ERR_VOLTAGE : 0);
        checked |= BMS_ERR_VOLTAGE;
    }

    if(ivt.currentValid && BMS_ControlDataValid()) {
        bool chargingFlow=ivt.current_A<0;
        bool tempBad=BMS_GetMinCellTempC()<CFG_CELL_CHARGE_TEMP_MIN_C || BMS_GetMaxCellTempC()>CFG_CELL_CHARGE_TEMP_MAX_C;
        flags=(flags & ~BMS_ERR_CHARGE_TEMP) | (chargingFlow && tempBad ? BMS_ERR_CHARGE_TEMP:0);
        checked|=BMS_ERR_CHARGE_TEMP;
    }
    uint32_t confirmed = 0;
    for (int b = 0; b < NUM_FLAG_BITS; b++)
    {
        uint32_t bit = (1UL << b);
        sampleTick[b] = (bit == BMS_ERR_VOLTAGE) ? ivt.voltageTick : ivt.currentTick;
        if (checked & bit)
        {
            // Timed from the reading (a reading already counted doesn't move the timer again)
            timerSample(&ivtTimer[b], sampleTick[b], (flags & bit) != 0);
            if (!(flags & bit))
            {
                ivtLastClearTick[b] = sampleTick[b];
            }
        }
        if ((flags & bit) && violationDue(now, &ivtTimer[b], ivtLastClearTick[b]))
        {
            confirmed |= bit;
        }
    }
    ivtFlagsPrev = flags;

    return confirmed;
}


void BMS_Safety_Tick(void)
{
    uint32_t now       = HAL_GetTick();
    uint32_t confirmed = 0;
    uint32_t immediate = 0;

    // isoSPI measurements
    if (!meas.measured)
    {
        immediate |= BMS_ERR_STARTUP;
    }
    else
    {
        if (now - meas.lastSampleTick > CFG_MEAS_STALE_TIMEOUT_MS)
        {
            immediate |= BMS_ERR_COMMS;
        }

        for (int b = 0; b < NUM_FLAG_BITS; b++)
        {
            uint32_t bit = (1UL << b);
            if ((meas.flags & bit) && violationDue(now, &meas.timer[b], meas.lastClearTick[b]))
            {
                confirmed |= bit;
            }
        }
    }

    // Current sensor
    confirmed |= evaluateIvt(now, &immediate);
    IvtReading controlIvt=BMS_IVT_GetReading();
    if(!BMS_ControlDataValid() || !controlIvt.currentValid || !controlIvt.voltageValid)BMS_RequestSafeOutputs();

    BMS_AcquisitionStatus a; BMS_GetAcquisitionStatus(&a);
    /* Loss inhibits consumers immediately; the documented measurement deadline
       debounces the external AMS latch. Never refresh it on a failed cycle. */
    if(meas.measured && now-a.cellTick>=CFG_MEAS_STALE_TIMEOUT_MS) immediate|=BMS_ERR_CELL_STALE;
    if(meas.measured && now-a.auxTick>=CFG_MEAS_STALE_TIMEOUT_MS) immediate|=BMS_ERR_AUX_STALE;
    if(immediate&BMS_ERR_COMMS) {
        if(a.pecFault) immediate|=BMS_ERR_PEC;
        if(a.counterFault) immediate|=BMS_ERR_COUNTER;
    }
    if(meas.measured && !a.celsiusValid && a.cellsValid && a.auxValid) immediate|=BMS_ERR_SENSOR;
    uint32_t active = confirmed | immediate | BMS_Health_Get();
    uint32_t previousFlags  = activeFlags;
    bool     previousOutput = faultOutput;
    activeFlags = active;

    if (active != BMS_OK)
    {
        faultOutput   = true;
        lastFaultTick = now;
    }
    else if (faultOutput && (now - lastFaultTick >= CFG_FAULT_CLEAR_DELAY_MS))
    {
        faultOutput = false;
    }

    if (active != previousFlags || faultOutput != previousOutput)
    {
        if (eventHead - eventTail < EVENT_QUEUE_LEN)
        {
            SafetyEvent* e = &events[eventHead % EVENT_QUEUE_LEN];
            e->tick        = now;
            e->flags       = active;
            e->prevFlags   = previousFlags;
            e->faultOutput = faultOutput;
            eventHead++;
        }
        else
        {
            eventsLost++;
        }
    }

    // No relay is ever on while the fault output is: drop them first (precharge follows in the same
    // tick), raise the TSAC ready relay last
    if (faultOutput)
    {
        BMS_RequestSafeOutputs();
        BMS_GPIO_WriteBypass(false);
        BMS_GPIO_WritePrecharge(false);
        BMS_GPIO_WriteTsacReady(false);
        BMS_GPIO_WriteFault(true);
    }
    else
    {
        BMS_GPIO_WriteFault(false);
        BMS_GPIO_WriteTsacReady(true);
    }
}


void BMS_Safety_ForceFault(void)
{
    BMS_GPIO_EmergencySafe();
    BMS_Health_Latch(BMS_ERR_INTERNAL);
    activeFlags |= BMS_ERR_INTERNAL;
    BMS_RequestSafeOutputs();
    faultOutput = true;
    BMS_GPIO_WriteBypass(false);
    BMS_GPIO_WritePrecharge(false);
    BMS_GPIO_WriteTsacReady(false);
    BMS_GPIO_WriteFault(true);
}


bool BMS_Safety_IsFaultOutputActive(void)
{
    return faultOutput;
}


uint32_t BMS_Safety_GetActiveFlags(void)
{
    return activeFlags;
}


bool BMS_Safety_PopEvent(SafetyEvent* event, uint32_t* lost)
{
    if (eventTail == eventHead)
    {
        return false;
    }
    *event = events[eventTail % EVENT_QUEUE_LEN];
    eventTail++;
    *lost = eventsLost;
    return true;
}


uint32_t BMS_Safety_GetMeasurementAgeMs(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool measured = meas.measured;
    uint32_t lastSampleTick = meas.lastSampleTick;
    __set_PRIMASK(primask);

    return measured ? (HAL_GetTick() - lastSampleTick) : UINT32_MAX;
}
