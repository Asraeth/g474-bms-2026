/*
 * bms_ivt.c
 *
 * Isabellenhuette IVT-S current / voltage sensor on CAN2 (IVT-S Datasheet v1.01, chapter 8)
 *
 * Result frames (DLC 6):
 *   DB0     mux ID (0x00 = I, 0x01 = U1, 0x02 = U2, 0x07 = Wh, ...)
 *   DB1     low nibble: message counter, high nibble: status bits
 *   DB2..5  signed 32-bit value, big-endian. mA, mV or Wh.
 */

#include "bms_ivt.h"
#include "bms_can.h"
#include "main.h"
#include "uartDMA.h"

#include <stdio.h>
#include <string.h>
#include "ivt_s.h"


#define IVT_MUX_RESULT_I        0x00
#define IVT_MUX_RESULT_U1       0x01
#define IVT_MUX_RESULT_U2       0x02
#define IVT_MUX_RESULT_U3       0x03
#define IVT_MUX_RESULT_T        0x04
#define IVT_MUX_RESULT_W        0x05
#define IVT_MUX_RESULT_AS       0x06
#define IVT_MUX_RESULT_WH       0x07

#define IVT_CMD_SET_CONFIG      0x20    // + result number (0 = I, 1 = U1, ...)
#define IVT_CMD_RESET_ERRORS    0x30    // Reset Error- and Logdata (stop mode): DB1 0x01 system errors, DB2 0x00
                                        // all their counters, DB3..6 serial number. Answered ~0.8 s later.
#define IVT_CMD_SET_MODE        0x34
#define IVT_CMD_GET_SYS_ERRORS  0x41    // DB1 0x00: bitmask of the system errors logged. Answered in stop mode only.
#define IVT_CMD_GET_SERIAL      0x7B    // Answered in stop mode only
#define IVT_MODE_STOP           0x00
#define IVT_MODE_RUN            0x01
#define IVT_CONFIG_DISABLED     0x00
#define IVT_CONFIG_CYCLIC       0x02    // Low nibble: cyclic. High nibble 0: big-endian, sign unchanged.
#define IVT_COMMAND_GAP_MS      3       // Datasheet: commands no faster than every 2 ms
#define IVT_STATUS_SYSTEM_ERROR 0x08    // Result status bit 3: system error, sensor functionality not ensured

typedef enum
{
    RESULT_I,
    RESULT_U1,
    RESULT_U2,
    RESULT_WH,
    RESULT_COUNT
} IvtResult;

typedef struct
{
    int32_t  value;
    uint8_t  status;
    uint32_t tick;
    bool     received;
} IvtValue;

static const struct
{
    uint32_t id;
    uint8_t  mux;
} resultIds[RESULT_COUNT] = {
    [RESULT_I]  = { CFG_IVT_ID_RESULT_I,  IVT_MUX_RESULT_I  },
    [RESULT_U1] = { CFG_IVT_ID_RESULT_U1, IVT_MUX_RESULT_U1 },
    [RESULT_U2] = { CFG_IVT_ID_RESULT_U2, IVT_MUX_RESULT_U2 },
    [RESULT_WH] = { CFG_IVT_ID_RESULT_WH, IVT_MUX_RESULT_WH },
};

// Every result is set explicitly, so a configuration stored in the IVT can't add messages (the team's IVT
// comes with all eight results cyclic). Disabled results still carry a time: the IVT ignores a Set Config
// Result whose time is 0x0000 (datasheet 8.3), so they would stay on. The datasheet's default times are used.
static const struct
{
    uint8_t  mux;
    uint8_t  mode;
    uint16_t period_ms;
} resultConfig[] = {
    { IVT_MUX_RESULT_I,  IVT_CONFIG_CYCLIC,   CFG_IVT_CURRENT_PERIOD_MS },
    { IVT_MUX_RESULT_U1, IVT_CONFIG_CYCLIC,   CFG_IVT_U1_PERIOD_MS },
    { IVT_MUX_RESULT_U2, IVT_CONFIG_CYCLIC,   CFG_IVT_U2_PERIOD_MS },
    { IVT_MUX_RESULT_U3, IVT_CONFIG_DISABLED, 60 },
    { IVT_MUX_RESULT_T,  IVT_CONFIG_DISABLED, 100 },
    { IVT_MUX_RESULT_W,  IVT_CONFIG_DISABLED, 30 },     // Power: the BMS works out I x U1 itself
    { IVT_MUX_RESULT_AS, IVT_CONFIG_DISABLED, 30 },
    { IVT_MUX_RESULT_WH, IVT_CONFIG_CYCLIC,   CFG_IVT_ENERGY_PERIOD_MS },
};

// Written by the CAN receive interrupt, read with interrupts disabled
static volatile IvtValue ivt[RESULT_COUNT] = {0};

// Signs that the IVT is running another configuration than the BMS's: it restarted (supply dip, power cycle)
// with the configuration stored in it, which keeps its results coming, so no data goes missing. Frames for
// results the BMS switches off (U3, T, W, As: the IDs between U2 and Wh), or current arriving at well over
// twice the configured period several times in a row. Counted by the CAN receive interrupt.
#define IVT_SLOW_CURRENT_FRAMES 5
#define IVT_SETTLE_MS           50      // Frames from the old configuration may still arrive after reconfiguring
static volatile uint32_t foreignFrames  = 0;
static volatile uint32_t slowCurrent    = 0;
static volatile uint32_t settledTick    = 0;

// Last time the current had a gap (bus errors, the IVT stopped or reconfigured), for the system error check
#define IVT_GAP_MS              25
static volatile uint32_t lastGapTick    = 0;

// Alive message (0xBF on CFG_IVT_ID_RESPONSE): the IVT has just (re)started and accepts commands
#define IVT_RESPONSE_ALIVE      0xBF
static volatile bool     alivePending   = false;

// Answers the system error check waits for (DB0 of the response), the latest of each, counted
#define IVT_RESPONSE_SYS_ERRORS 0x81
#define IVT_RESPONSE_SERIAL     0xBB
#define IVT_RESPONSE_RESET      0xB0
typedef enum { RESPONSE_SYS_ERRORS, RESPONSE_SERIAL, RESPONSE_RESET, RESPONSE_COUNT } IvtResponse;
static volatile uint8_t  responseData[RESPONSE_COUNT][8];
static volatile uint32_t responseCount[RESPONSE_COUNT];

// System errors (bitmask of GET system errors, datasheet 8.6). Bus errors (e.g. a node on CAN2 at the wrong
// bitrate) log CAN receive / transmit errors and can set the status's system error bit, which stays until
// the IVT is power cycled or its errors are cleared.
#define IVT_SYS_ERRORS_CAN      0x000CU     // CAN receive data, CAN transmit data
static const char* const systemErrorNames[16] = {
    "code CRC", "parameter CRC", "CAN receive", "CAN transmit", "overtemperature", "undertemperature",
    "power failure", "system clock", "system init", "configuration", "overcurrent detection", "EEPROM",
    "ADC clock", "reset illegal opcode", "reset watchdog", "reset EMC",
};


static IVTS_Driver strictIvt;
static bool strictStarted;
static volatile bool commandAwaiting, commandAnswered;
static uint8_t expectedAnswer[8];
static void ensureStrict(void)
{
    if (strictStarted) return;
    IVTS_Config cfg=IVTS_DefaultConfig();
    cfg.current_id=CFG_IVT_ID_RESULT_I; cfg.u1_id=CFG_IVT_ID_RESULT_U1;
    cfg.u2_id=CFG_IVT_ID_RESULT_U2; cfg.energy_id=CFG_IVT_ID_RESULT_WH;
    cfg.response_id=CFG_IVT_ID_RESPONSE;
    cfg.current_timeout_ms=CFG_IVT_CURRENT_TIMEOUT_MS;
    cfg.u1_timeout_ms=cfg.u2_timeout_ms=CFG_IVT_VOLTAGE_TIMEOUT_MS;
    cfg.energy_timeout_ms=3U*CFG_IVT_ENERGY_PERIOD_MS;
    if (!IVTS_Init(&strictIvt,&cfg)) Error_Handler();
    strictStarted=true;
}
void BMS_IVT_InvalidateTransport(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ensureStrict(); IVTS_InvalidateTransport(&strictIvt);
    __set_PRIMASK(mask);
}
void BMS_IVT_GetSnapshot(IVTS_Snapshot *snapshot)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ensureStrict(); IVTS_GetSnapshot(&strictIvt,HAL_GetTick(),snapshot);
    __set_PRIMASK(mask);
}
static int32_t negate(int32_t value)
{
    return value==INT32_MIN ? INT32_MAX : -value;
}
void BMS_IVT_HandleFrame(uint32_t id,const uint8_t *data,uint32_t len)
{
    ensureStrict();
    IVTS_Frame frame={.id=id,.dlc=(uint8_t)len};
    if (len>8) return;
    memcpy(frame.data,data,len);
    IVTS_RxEvent event=IVTS_Receive(&strictIvt,&frame,HAL_GetTick());
    if (event==IVTS_REJECTED) return;
    uint32_t now=HAL_GetTick();
    bool settled=(int32_t)(now-settledTick)>=0;
    if (id==CFG_IVT_ID_RESPONSE) {
        if (commandAwaiting && memcmp(data,expectedAnswer,8)==0) commandAnswered=true;
        int slot=data[0]==IVT_RESPONSE_SYS_ERRORS ? RESPONSE_SYS_ERRORS
                : data[0]==IVT_RESPONSE_SERIAL ? RESPONSE_SERIAL
                : data[0]==IVT_RESPONSE_RESET ? RESPONSE_RESET : -1;
        if (event==IVTS_ALIVE) alivePending=true;
        else if (slot>=0) {
            memcpy((void*)responseData[slot],data,8);
            responseCount[slot]++;
        }
        return;
    }
    if (id>CFG_IVT_ID_RESULT_U2 && id<CFG_IVT_ID_RESULT_WH) {
        if (settled && len==6) foreignFrames++;
        return;
    }
    IVTS_Snapshot raw; IVTS_GetSnapshot(&strictIvt,now,&raw);
    const IVTS_Result *values[]={&raw.current,&raw.u1,&raw.u2,&raw.energy};
    for (int r=0;r<RESULT_COUNT;r++) if (id==resultIds[r].id && event!=IVTS_IGNORED) {
        const IVTS_Result *v=values[r];
        if (r==RESULT_I && (!ivt[r].received || now-ivt[r].tick>IVT_GAP_MS)) lastGapTick=now;
        if (r==RESULT_I && ivt[r].received && settled)
            slowCurrent=(now-ivt[r].tick>2*CFG_IVT_CURRENT_PERIOD_MS+1) ? slowCurrent+1 : 0;
        ivt[r].value=v->raw_value; ivt[r].status=v->result_state;
        ivt[r].tick=v->last_measurement_ms; ivt[r].received=v->has_result;
        return;
    }
}
IvtReading BMS_IVT_GetReading(void)
{
    IVTS_Snapshot raw; BMS_IVT_GetSnapshot(&raw);
    IvtReading r={0};
    r.current_mA=CFG_IVT_CURRENT_DISCHARGE_POSITIVE ? raw.current.raw_value : negate(raw.current.raw_value);
    r.voltage_mV=raw.u1.raw_value; r.voltage2_mV=raw.u2.raw_value;
    r.current_A=r.current_mA/1000.0f; r.voltage_V=r.voltage_mV/1000.0f;
    r.voltage2_V=r.voltage2_mV/1000.0f; r.power_W=r.current_A*r.voltage_V;
    r.energy_Wh=CFG_IVT_CURRENT_DISCHARGE_POSITIVE ? raw.energy.raw_value : negate(raw.energy.raw_value);
    r.energyTick=raw.energy.last_measurement_ms;
    r.currentFresh=raw.current.fresh; r.voltageFresh=raw.u1.fresh; r.voltage2Fresh=raw.u2.fresh;
    r.energyFresh=raw.energy.fresh; r.energyReceived=raw.energy.has_result;
    r.configurationVerified=raw.configuration_verified;
    r.currentValid=raw.current.valid && raw.configuration_verified;
    r.voltageValid=raw.u1.valid && raw.configuration_verified;
    r.voltage2Valid=raw.u2.valid && raw.configuration_verified;
    r.energyValid=raw.energy.valid && raw.configuration_verified;
    r.powerFresh=r.currentValid && r.voltageValid;
    r.sensorError=((raw.current.result_state|raw.u1.result_state)&IVTS_STATE_INVALID_MASK)!=0
        || raw.current.peer_error || raw.u1.peer_error
        || (raw.current.received && !raw.current.protocol_valid) || (raw.u1.received && !raw.u1.protocol_valid)
        || raw.current.duplicate || raw.u1.duplicate;
    r.systemError=((raw.current.result_state|raw.u1.result_state)&IVTS_STATE_SYSTEM_ERROR)!=0;
    r.voltage2Error=!raw.u2.status_valid || !raw.u2.protocol_valid || raw.u2.peer_error || raw.u2.duplicate;
    uint32_t now=HAL_GetTick();
    r.currentTick=raw.current.last_measurement_ms; r.voltageTick=raw.u1.last_measurement_ms;
    r.currentAge_ms=raw.current.received ? now-r.currentTick : UINT32_MAX;
    r.voltageAge_ms=raw.u1.received ? now-r.voltageTick : UINT32_MAX;
    r.restartEpoch=raw.restart_epoch;
    return r;
}

// Span: written by the timer interrupt, started and read by the main loop with interrupts disabled
static volatile bool     spanActive = false;
static volatile bool     spanBad = false;
static volatile float    spanU1Sum_V = 0.0f;
static volatile uint32_t spanTicks = 0;
static volatile float    spanCurrentMin_A = 0.0f;
static volatile float    spanCurrentMax_A = 0.0f;


void BMS_IVT_Tick(void)
{
    if (!spanActive)
    {
        return;
    }
    IvtReading r = BMS_IVT_GetReading();
    if (!r.voltageValid || !r.currentValid || r.sensorError)
    {
        spanBad = true;
    }
    if (spanTicks == 0 || r.current_A < spanCurrentMin_A) spanCurrentMin_A = r.current_A;
    if (spanTicks == 0 || r.current_A > spanCurrentMax_A) spanCurrentMax_A = r.current_A;
    spanU1Sum_V += r.voltage_V;
    spanTicks++;
}


void BMS_IVT_SpanStart(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    spanU1Sum_V = 0.0f;
    spanTicks   = 0;
    spanBad     = false;
    spanActive  = true;
    __set_PRIMASK(primask);
}


bool BMS_IVT_SpanEnd(float* u1Avg_V, float* currentRange_A)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    spanActive = false;
    float    sum   = spanU1Sum_V;
    uint32_t ticks = spanTicks;
    bool     bad   = spanBad;
    float    range = spanCurrentMax_A - spanCurrentMin_A;
    __set_PRIMASK(primask);

    if (ticks == 0 || bad)
    {
        return false;
    }
    if (u1Avg_V)        *u1Avg_V = sum / (float)ticks;
    if (currentRange_A) *currentRange_A = range;
    return true;
}


// Returns false if the previous command was never acknowledged, i.e. the IVT isn't on the bus
static bool sendCommandData(const uint8_t data[8])
{
    if (!BMS_CAN_IsIvtTxIdle()) return false;
    if (data[0]==IVT_CMD_SET_MODE && data[1]==IVT_MODE_STOP) BMS_IVT_InvalidateTransport();
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    memcpy(expectedAnswer,data,8); expectedAnswer[0]|=0x80;
    commandAnswered=false; commandAwaiting=true;
    __set_PRIMASK(mask);
    BMS_CAN_SendIvtCommand(CFG_IVT_ID_COMMAND,data);
    HAL_Delay(IVT_COMMAND_GAP_MS);
    /* SET responses echo the complete accepted command (v1.01 p20).
       GET/error-reset responses are handled asynchronously below. */
    bool await=data[0]>=0x20 && data[0]<=0x27;
    await=await || data[0]==IVT_CMD_SET_MODE;
    uint32_t start=HAL_GetTick();
    while (await && !commandAnswered && HAL_GetTick()-start<CFG_IVT_CURRENT_TIMEOUT_MS) HAL_Delay(1);
    bool ok=!await || commandAnswered;
    commandAwaiting=false;
    if (!ok) BMS_CAN_AbortIvtTx();
    return ok;
}

static bool sendCommand(uint8_t db0, uint8_t db1, uint8_t db2, uint8_t db3)
{
    uint8_t data[8] = { db0, db1, db2, db3, 0x00, 0x00, 0x00, 0x00 };   // Unused bytes must be 0
    return sendCommandData(data);
}

static uint32_t responses(IvtResponse r)
{
    return responseCount[r];
}

// The latest answer of that kind, if one arrived since 'since' (a responses() count)
static bool getResponse(IvtResponse r, uint32_t since, uint8_t out[8])
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool fresh = responseCount[r] != since;
    for (uint32_t i = 0; i < 8; i++)
    {
        out[i] = responseData[r][i];
    }
    __set_PRIMASK(primask);
    return fresh;
}


static bool configureResult(uint8_t mux, uint8_t mode, uint16_t period_ms)
{
    return sendCommand(IVT_CMD_SET_CONFIG | mux, mode, (uint8_t)(period_ms >> 8), (uint8_t)period_ms);
}


// Returns false if it didn't get through. Commands nobody acknowledged are dropped, so they aren't delivered
// late when the IVT reappears, except a final SET_MODE run after a part-way failure (harmless late).
static bool configureIvt(void)
{
    // Results can only be configured in stop mode. Nothing is stored to the IVT's EEPROM.
    // SET_MODE: DB1 = mode now, DB2 = start-up mode (only applied by STORE, left as run)
    if (!sendCommand(IVT_CMD_SET_MODE, IVT_MODE_STOP, IVT_MODE_RUN, 0x00))
    {
        BMS_CAN_AbortIvtTx();
        return false;
    }
    for (uint32_t i = 0; i < sizeof(resultConfig) / sizeof(resultConfig[0]); i++)
    {
        if (!configureResult(resultConfig[i].mux, resultConfig[i].mode, resultConfig[i].period_ms))
        {
            // Stuck part way (bus busy or errors): don't leave it stopped. Whatever configuration it ends up
            // running, the checks in BMS_IVT_Service() see it.
            BMS_CAN_AbortIvtTx();
            sendCommand(IVT_CMD_SET_MODE, IVT_MODE_RUN, IVT_MODE_RUN, 0x00);
            return false;
        }
    }
    return sendCommand(IVT_CMD_SET_MODE, IVT_MODE_RUN, IVT_MODE_RUN, 0x00);
}


static bool     configured     = false;
static bool configurationAttempted;
static uint32_t lastConfigTick = 0;

static void startConfiguration(const char* why)
{
    alivePending = false;
    printfDma("%s", why);
    bool verified=configureIvt();
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    IVTS_SetConfigurationVerified(&strictIvt,verified);
    __set_PRIMASK(mask);
    if (!verified)
    {
        printfDma("IVT not acknowledging on CAN2, configuration skipped\n");
    }
    configured     = verified;
    configurationAttempted=true;
    lastConfigTick = HAL_GetTick();
    settledTick    = lastConfigTick + IVT_SETTLE_MS;
    foreignFrames  = 0;
    slowCurrent    = 0;
}


/*
 * System error check. A system error (status bit 3) that persists past the debounce faults the AMS; the BMS
 * then stops the IVT briefly and asks which system errors it has logged. If they're all CAN errors (bus
 * disturbances, which don't touch the measurement), it clears them, at most CFG_IVT_AUTO_CLEAR_MAX times per
 * power-up, and the AMS fault clears once the IVT's status is clean again. Anything else stays: power cycle
 * the IVT. The logged errors are lifetime counters, so one left over from earlier (say a watchdog reset) also
 * keeps a later CAN error from being cleared.
 * The check waits for a clean bus (current without a gap for IVT_CLEAN_BUS_MS): during a disturbance its
 * commands wouldn't get through, and errors cleared would come straight back. It's made once per episode
 * of the status bit, again after a clear, and retried if it got no answer.
 */
#define IVT_SYS_ERROR_PERSIST_MS    (CFG_FAULT_DEBOUNCE_MS + 2 * CFG_SAFETY_TICK_MS)
#define IVT_CLEAN_BUS_MS            2000    // Disturbances come in bursts (bench: 0.5 s quiet spells inside one)
#define IVT_GET_TIMEOUT_MS          100     // Gets are answered at once in stop mode
#define IVT_RESET_TIMEOUT_MS        2000    // ~0.8 s measured, "at least 1.2 s" in the datasheet

typedef enum { CHECK_IDLE, CHECK_MASK, CHECK_SERIAL, CHECK_RESET } SystemErrorCheck;
static SystemErrorCheck check = CHECK_IDLE;
static uint32_t checkTick = 0;
static uint32_t checkMark = 0;
static uint32_t autoClears = 0;
static bool     checkedEpisode = false;     // Done with this episode of the status bit
static uint32_t checkFailures = 0;          // Checks this episode that got no answer

#define IVT_CHECK_TRIES             3

typedef enum { CHECK_FAILED, CHECK_VERDICT, CHECK_CLEARED } CheckOutcome;

// Failed: tried again (IVT_CHECK_TRIES per episode). Verdict: done for this episode. Cleared: checked again if
// the status bit stays or comes back.
static void finishCheck(const char* why, CheckOutcome outcome)
{
    check = CHECK_IDLE;
    if (outcome == CHECK_FAILED && ++checkFailures >= IVT_CHECK_TRIES)
    {
        printfDma("IVT system error check failed %d times, not tried again until its status changes\n", IVT_CHECK_TRIES);
    }
    checkedEpisode = outcome == CHECK_VERDICT || checkFailures >= IVT_CHECK_TRIES;
    startConfiguration(why);
}

// Main loop, while a check is under way: one step per answer, without waiting for it
static void serviceCheck(uint32_t now)
{
    uint8_t answer[8];
    switch (check)
    {
    case CHECK_MASK:
        if (getResponse(RESPONSE_SYS_ERRORS, checkMark, answer) && answer[1] == 0x00)
        {
            uint32_t mask = answer[2] | ((uint32_t)answer[3] << 8);
            char names[160];
            int n = 0;
            for (uint32_t b = 0; b < 16 && n < (int)sizeof(names) - 24; b++)
            {
                if (mask & (1UL << b))
                {
                    n += snprintf(&names[n], sizeof(names) - n, "%s%s", n ? ", " : "", systemErrorNames[b]);
                }
            }
            names[n] = '\0';
            if (mask == 0)
            {
                finishCheck("IVT reports a system error but has none logged: not cleared, reconfiguring\n", CHECK_VERDICT);
            }
            else if (mask & ~IVT_SYS_ERRORS_CAN)
            {
                printfDma("IVT system errors logged: %s. Not cleared automatically: power cycle the IVT\n", names);
                finishCheck("IVT reconfiguring\n", CHECK_VERDICT);
            }
            else
            {
                printfDma("IVT system errors logged: %s. Clearing them (%lu of %d)\n", names, (unsigned long)autoClears + 1,
                          CFG_IVT_AUTO_CLEAR_MAX);
                checkMark = responses(RESPONSE_SERIAL);
                sendCommand(IVT_CMD_GET_SERIAL, 0x00, 0x00, 0x00);
                check = CHECK_SERIAL;
                checkTick = now;
            }
        }
        else if (now - checkTick >= IVT_GET_TIMEOUT_MS)
        {
            finishCheck("IVT didn't answer the system error check, reconfiguring\n", CHECK_FAILED);
        }
        break;

    case CHECK_SERIAL:
        if (getResponse(RESPONSE_SERIAL, checkMark, answer))
        {
            uint8_t reset[8] = { IVT_CMD_RESET_ERRORS, 0x01, 0x00, answer[1], answer[2], answer[3], answer[4], 0x00 };
            checkMark = responses(RESPONSE_RESET);
            sendCommandData(reset);
            check = CHECK_RESET;
            checkTick = now;
        }
        else if (now - checkTick >= IVT_GET_TIMEOUT_MS)
        {
            finishCheck("IVT didn't send its serial number, errors not cleared, reconfiguring\n", CHECK_FAILED);
        }
        break;

    case CHECK_RESET:
        if (getResponse(RESPONSE_RESET, checkMark, answer))
        {
            autoClears++;
            finishCheck("IVT system errors cleared, reconfiguring\n", CHECK_CLEARED);
        }
        else if (now - checkTick >= IVT_RESET_TIMEOUT_MS)
        {
            finishCheck("IVT didn't confirm clearing its errors, reconfiguring\n", CHECK_FAILED);
        }
        break;

    default:
        check = CHECK_IDLE;
        break;
    }
}


void BMS_IVT_Service(void)
{
    if (!CFG_IVT_CONFIGURE_ON_BOOT)
    {
        return;
    }

    static bool     systemErrorSeen = false;
    static uint32_t systemErrorTick = 0;
    static bool     started = false;

    uint32_t now = HAL_GetTick();
    if (!started)
    {
        if (now < CFG_IVT_STARTUP_MS)
        {
            return;     // IVT still starting up after power-up (latched: the tick wraps after 49.7 days)
        }
        started = true;
    }
    if (check != CHECK_IDLE)
    {
        serviceCheck(now);
        return;
    }

    IvtReading reading = BMS_IVT_GetReading();

    // A system error that persists (not a one-off), on a clean bus: check what it is
    bool due = configured && !checkedEpisode && now - systemErrorTick >= IVT_SYS_ERROR_PERSIST_MS &&
               now - lastGapTick >= IVT_CLEAN_BUS_MS && now - lastConfigTick >= IVT_CLEAN_BUS_MS;
    if (reading.currentFresh && reading.systemError)
    {
        if (!systemErrorSeen)
        {
            systemErrorSeen = true;
            systemErrorTick = now;
        }
        else if (due && autoClears >= CFG_IVT_AUTO_CLEAR_MAX)
        {
            checkedEpisode = true;
            printfDma("IVT reports a system error: already cleared %d times since power-up, not again: "
                      "power cycle the IVT\n", CFG_IVT_AUTO_CLEAR_MAX);
        }
        else if (due)
        {
            printfDma("IVT reports a system error: checking what it logged\n");
            checkMark = responses(RESPONSE_SYS_ERRORS);
            if (sendCommand(IVT_CMD_SET_MODE, IVT_MODE_STOP, IVT_MODE_RUN, 0x00) &&
                sendCommand(IVT_CMD_GET_SYS_ERRORS, 0x00, 0x00, 0x00))
            {
                check = CHECK_MASK;
                checkTick = now;
                return;
            }
            BMS_CAN_AbortIvtTx();
            finishCheck("IVT not acknowledging on CAN2, reconfiguring\n", CHECK_FAILED);
            return;
        }
    }
    else if (reading.currentFresh)
    {
        systemErrorSeen = false;
        checkedEpisode  = false;
        checkFailures   = 0;
    }

    bool dataMissing = !reading.currentFresh || !reading.voltageFresh || !reading.voltage2Fresh || !reading.energyFresh || !reading.configurationVerified;
    bool otherConfig = foreignFrames > 0 || slowCurrent >= IVT_SLOW_CURRENT_FRAMES;
    // Current stopped only just now: it's probably restarting, so wait a little for its alive message rather than
    // send commands it drops while it boots
    bool justStopped = !reading.currentFresh && reading.currentAge_ms < CFG_IVT_RESTART_WAIT_MS;

    // Restarted: configure straight away. Its stored configuration sends U1 only every 60 ms, so waiting for
    // its data would leave the longer gap. A real restart leaves a gap in the data: an alive message while the
    // current is still arriving isn't one, and stopping the IVT to reconfigure it would only cost data.
    bool spurious = false;
    bool alive = alivePending && !spurious;
    if (spurious)
    {
        alivePending = false;
        printfDma("IVT alive message while its data is arriving, ignored\n");
    }

    // Silent (stopped, or gone): retry often. Running, but not as configured: now and then.
    uint32_t interval = configured && reading.currentFresh ? CFG_IVT_RECONFIG_INTERVAL_MS : CFG_IVT_RETRY_INTERVAL_MS;

    if (!configurationAttempted || alive ||
        ((!configured || dataMissing || otherConfig) && !justStopped && (now - lastConfigTick >= interval)))
    {
        startConfiguration(!configured ? "Configuring IVT\n" : alive ? "IVT restarted, configuring\n"
                           : dataMissing ? "IVT data missing, reconfiguring\n"
                           : "IVT running another configuration (restarted?), reconfiguring\n");
    }
}
