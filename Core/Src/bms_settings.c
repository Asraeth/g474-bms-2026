/*
 * bms_settings.c
 *
 * Runtime settings (bms_settings.h). Requests arrive in the CAN1 RX interrupt and are queued; the main
 * loop works through them, so a save (flash erase ~20 ms) never runs in an interrupt. Responses are
 * queued for the timer interrupt, which sends at most one per fast period on a tick with no other frame.
 *
 * The settings page holds one record: magic, layout version, the raw values and a CRC-32. Anything else
 * (erased page, older layout, bad CRC) means the built-in values; a stored value outside this build's range
 * falls back to its built-in value.
 */

#include "bms_settings.h"
#include "bms_precharge.h"
#include "bms_libWrapper.h"
#include "main.h"
#include "uartDMA.h"

#include <stddef.h>
#include <string.h>

// Flash reads go through this, so the PC simulation can point them at its own memory
#ifndef BMS_FLASH_PTR
#define BMS_FLASH_PTR(addr)     ((const void*)(uintptr_t)(addr))
#endif

#define SETTINGS_MAGIC          0x53534D42u     // "BMSS"
#define SETTINGS_VERSION        CFG_CONFIG_VERSION

#define REQUEST_QUEUE_LEN       8
#define RESPONSE_QUEUE_LEN      16

#define WHOLE(x)    ((int32_t)((x) + ((x) >= 0 ? 0.5f : -0.5f)))
#define TENTHS(x)   ((int32_t)((x) * 10.0f + ((x) >= 0 ? 0.5f : -0.5f)))

typedef struct
{
    int32_t def, min, max;      // Raw units
    float   scale;              // Raw -> engineering units
} SettingInfo;

static const SettingInfo info[SETTING_COUNT] = {
    [SETTING_POWER_LIMIT]       = { WHOLE(CFG_INVERTER_POWER_LIMIT_W), WHOLE(CFG_SETTING_POWER_MIN_W),
                                    WHOLE(CFG_INVERTER_POWER_LIMIT_W), 1.0f },
    [SETTING_DISCHARGE_CURRENT] = { TENTHS(CFG_MAX_DISCHARGE_CURRENT_A), TENTHS(CFG_SETTING_CURRENT_MIN_A),
                                    TENTHS(CFG_MAX_DISCHARGE_CURRENT_A), 0.1f },
    [SETTING_TEMP_LIMIT]        = { TENTHS(CFG_INVERTER_DERATE_END_C), TENTHS(CFG_SETTING_TEMP_LIMIT_MIN_C),
                                    TENTHS(CFG_INVERTER_DERATE_END_C), 0.1f },
    [SETTING_REGEN_ENABLED]     = { 1, 0, 1, 1.0f },
    [SETTING_REGEN_CURRENT]     = { TENTHS(CFG_INVERTER_REGEN_CURRENT_A), 0, TENTHS(CFG_INVERTER_REGEN_CURRENT_A), 0.1f },
    [SETTING_CHARGE_CURRENT]    = { TENTHS(CFG_CHARGER_CURRENT_A), 0, TENTHS(CFG_CHARGER_CURRENT_MAX_A), 0.1f },
    [SETTING_CHARGE_VOLTAGE]    = { TENTHS(CFG_CHARGER_VOLTAGE_V), TENTHS(CFG_PACK_V_MIN),
                                    TENTHS(CFG_CHARGER_VOLTAGE_MAX_V), 0.1f },
};

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    int32_t  values[SETTING_COUNT];     // [0] unused
    uint32_t crc;
    uint32_t pad;
} StoredSettings;

_Static_assert(sizeof(StoredSettings) % 8 == 0, "Settings record must be whole double words (flash programming unit)");

static volatile int32_t values[SETTING_COUNT];     // In force. Main loop writes, interrupts read.
static int32_t savedValues[SETTING_COUNT];          // What flash holds (built-in values if nothing)
static bool    stored = false;                      // Flash holds a valid record

static uint8_t responseToken;
static uint8_t           requests[REQUEST_QUEUE_LEN][8];
static volatile uint32_t requestHead = 0;           // CAN1 RX interrupt
static volatile uint32_t requestTail = 0;           // Main loop
static volatile uint32_t requestsDropped = 0;

static bool     written = false;                    // The page was erased and written since start-up
static uint32_t lastWriteTick = 0;

static uint8_t           responses[RESPONSE_QUEUE_LEN][8];
static volatile uint32_t responseHead = 0;          // Main loop
static volatile uint32_t responseTail = 0;          // Timer interrupt


static uint32_t crc32(const void* data, uint32_t length)
{
    const uint8_t* p = data;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
        }
    }
    return ~crc;
}


static bool inRange(SettingId id, int32_t value)
{
    return value >= info[id].min && value <= info[id].max;
}


static bool differsFromSaved(void)
{
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        if (values[id] != savedValues[id])
        {
            return true;
        }
    }
    return false;
}


static bool differsFromBuiltIn(void)
{
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        if (values[id] != info[id].def)
        {
            return true;
        }
    }
    return false;
}


static uint8_t flags(void)
{
    return (differsFromSaved() ? SETTINGS_FLAG_UNSAVED : 0) | (differsFromBuiltIn() ? SETTINGS_FLAG_MODIFIED : 0) |
           (stored ? SETTINGS_FLAG_STORED : 0);
}


static void record(StoredSettings* rec)
{
    memset(rec, 0, sizeof(*rec));
    rec->magic   = SETTINGS_MAGIC;
    rec->version = SETTINGS_VERSION;
    rec->count   = SETTING_COUNT;
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        rec->values[id] = values[id];
    }
    rec->crc = crc32(rec, offsetof(StoredSettings, crc));
}


// Erases the settings page and writes the record. Bank 2's last page in dual-bank mode (the program runs
// from bank 1, so it keeps running), the last 4 KB page in single-bank mode.
static bool flashWrite(const StoredSettings* rec)
{
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.NbPages   = 1;
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK))
    {
        erase.Banks = FLASH_BANK_2;
        erase.Page  = (CFG_SETTINGS_FLASH_ADDR - (FLASH_BASE + 0x40000u)) / 0x800u;
    }
    else
    {
        erase.Banks = FLASH_BANK_1;
        erase.Page  = (CFG_SETTINGS_FLASH_ADDR - FLASH_BASE) / 0x1000u;
    }

    bool ok = HAL_FLASH_Unlock() == HAL_OK;
    if (ok)
    {
        uint32_t pageError = 0;
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        ok = HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;

        const uint64_t* words = (const uint64_t*)rec;
        for (uint32_t i = 0; ok && i < sizeof(*rec) / 8; i++)
        {
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CFG_SETTINGS_FLASH_ADDR + 8 * i, words[i]) == HAL_OK;
        }
        HAL_FLASH_Lock();
    }
    return ok && memcmp(BMS_FLASH_PTR(CFG_SETTINGS_FLASH_ADDR), rec, sizeof(*rec)) == 0;
}


void BMS_Settings_Init(void)
{
    written = false;
    const StoredSettings* rec = BMS_FLASH_PTR(CFG_SETTINGS_FLASH_ADDR);
    stored = rec->magic == SETTINGS_MAGIC && rec->version == SETTINGS_VERSION && rec->count == SETTING_COUNT &&
             rec->crc == crc32(rec, offsetof(StoredSettings, crc));

    int outside = 0;
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        int32_t v = info[id].def;
        if (stored)
        {
            if (inRange(id, rec->values[id]))
            {
                v = rec->values[id];
            }
            else
            {
                outside++;
            }
        }
        values[id] = v;
        savedValues[id] = stored ? rec->values[id] : v;
    }
    requestHead = requestTail = 0;
    responseHead = responseTail = 0;

    if (!stored)
    {
        printfDma("Settings: built-in values (none saved)\n");
    }
    else
    {
        printfDma("Settings: loaded from flash%s\n", differsFromBuiltIn() ? ", limits changed from the built-in values" : "");
        if (outside)
        {
            printfDma("Settings: %d saved values outside this build's range, built-in values used\n", outside);
        }
    }
}


BmsSettings BMS_Settings_Get(void)
{
    int32_t v[SETTING_COUNT];
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        v[id] = values[id];
    }
    __set_PRIMASK(primask);

    BmsSettings s;
    s.powerLimit_W       = v[SETTING_POWER_LIMIT] * info[SETTING_POWER_LIMIT].scale;
    s.dischargeCurrent_A = v[SETTING_DISCHARGE_CURRENT] * info[SETTING_DISCHARGE_CURRENT].scale;
    s.tempLimit_C        = v[SETTING_TEMP_LIMIT] * info[SETTING_TEMP_LIMIT].scale;
    s.regenEnabled       = v[SETTING_REGEN_ENABLED] != 0;
    s.regenCurrent_A     = v[SETTING_REGEN_CURRENT] * info[SETTING_REGEN_CURRENT].scale;
    s.chargeCurrent_A    = v[SETTING_CHARGE_CURRENT] * info[SETTING_CHARGE_CURRENT].scale;
    s.chargeVoltage_V    = v[SETTING_CHARGE_VOLTAGE] * info[SETTING_CHARGE_VOLTAGE].scale;
    s.modified = false;
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        s.modified = s.modified || v[id] != info[id].def;
    }
    return s;
}


void BMS_Settings_HandleRequest(const uint8_t* data, uint32_t length)
{
    if(length!=8 || !data)return;
    uint32_t next = (requestHead + 1) % REQUEST_QUEUE_LEN;
    if (next == requestTail)
    {
        requestsDropped++;          // The GUI times out and asks again
        return;
    }
    memset(requests[requestHead], 0, 8);
    memcpy(requests[requestHead], data, length < 8 ? length : 8);
    requestHead = next;
}


static uint32_t responseSpace(void)
{
    return (responseTail + RESPONSE_QUEUE_LEN - responseHead - 1) % RESPONSE_QUEUE_LEN;
}


static void respond(uint8_t command, uint8_t id, SettingsResult result)
{
    uint8_t* r = responses[responseHead];
    int32_t value = (id > 0 && id < SETTING_COUNT) ? values[id] : 0;
    r[0] = command;
    r[1] = id;
    r[2] = (uint8_t)result;
    r[3] = (uint8_t)(flags() | (responseToken<<3));
    r[4] = (uint8_t)value;
    r[5] = (uint8_t)(value >> 8);
    r[6] = (uint8_t)(value >> 16);
    r[7] = (uint8_t)(value >> 24);
    responseHead = (responseHead + 1) % RESPONSE_QUEUE_LEN;
}


static void process(const uint8_t* req)
{
    uint8_t command = req[0];
    uint8_t id      = req[1];
    int32_t value   = (int32_t)((uint32_t)req[4] | ((uint32_t)req[5] << 8) | ((uint32_t)req[6] << 16) |
                                ((uint32_t)req[7] << 24));
    bool known = id > 0 && id < SETTING_COUNT;
    responseToken=req[3]&31U;
    if(req[2]!=CFG_CONFIG_VERSION) {respond(command,id,SETTINGS_ERR_VERSION);return;}
    if(req[3]>31 || (command!=SETTINGS_CMD_WRITE && value!=0)) {respond(command,id,SETTINGS_ERR_MALFORMED);return;}

    switch (command)
    {
    case SETTINGS_CMD_READ:
        respond(command, id, known ? SETTINGS_OK : SETTINGS_ERR_UNKNOWN_SETTING);
        break;

    case SETTINGS_CMD_WRITE:
        if (!known)
        {
            respond(command, id, SETTINGS_ERR_UNKNOWN_SETTING);
        }
        else if (!inRange(id, value))
        {
            respond(command, id, SETTINGS_ERR_RANGE);
        }
        else
        {
            if (values[id] != value)
            {
                printfDma("Setting %d: %ld -> %ld\n", id, (long)values[id], (long)value);
            }
            values[id] = value;
            respond(command, id, SETTINGS_OK);
        }
        break;

    case SETTINGS_CMD_SAVE:
    {
        // The page survives ~10,000 erases: one already holding these settings isn't erased again, and a
        // GUI or script saving over and over is held to one erase per CFG_SETTINGS_SAVE_INTERVAL_MS
        StoredSettings rec;
        record(&rec);
        if (BMS_Precharge_IsTsActive() || BMS_IsCharging())
        {
            respond(command, 0, SETTINGS_ERR_TS_ACTIVE);
        }
        else if (stored && memcmp(BMS_FLASH_PTR(CFG_SETTINGS_FLASH_ADDR), &rec, sizeof(rec)) == 0)
        {
            respond(command, 0, SETTINGS_OK);
        }
        else if (written && HAL_GetTick() - lastWriteTick < CFG_SETTINGS_SAVE_INTERVAL_MS)
        {
            respond(command, 0, SETTINGS_ERR_BUSY);
        }
        else
        {
            bool ok = flashWrite(&rec);
            written       = true;
            lastWriteTick = HAL_GetTick();
            stored        = ok;
            if (ok)
            {
                memcpy(savedValues, rec.values, sizeof(savedValues));
            }
            printfDma("Settings %s\n", ok ? "saved" : "NOT saved: flash error");
            respond(command, 0, ok ? SETTINGS_OK : SETTINGS_ERR_FLASH);
        }
        break;
    }

    case SETTINGS_CMD_DEFAULTS:
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        for (int i = 1; i < SETTING_COUNT; i++)
        {
            values[i] = info[i].def;
        }
        __set_PRIMASK(primask);
        printfDma("Settings: built-in values restored\n");
        respond(command, 0, SETTINGS_OK);
        break;
    }

    case SETTINGS_CMD_READ_ALL:
        for (int i = 1; i < SETTING_COUNT; i++)
        {
            respond(command, (uint8_t)i, SETTINGS_OK);
        }
        break;

    default:
        respond(command, id, SETTINGS_ERR_UNKNOWN_COMMAND);
        break;
    }
}


void BMS_Settings_Service(void)
{
    // A request is only taken when all its responses fit
    while (requestTail != requestHead && responseSpace() >= SETTING_COUNT - 1)
    {
        process(requests[requestTail]);
        requestTail = (requestTail + 1) % REQUEST_QUEUE_LEN;
    }
}


bool BMS_Settings_GetResponse(CanTxMsg* msg)
{
    if (responseTail == responseHead)
    {
        return false;
    }

    memset(msg, 0, sizeof(*msg));
    msg->header.Identifier          = CFG_CAN_ID_CONFIG_RESPONSE;
    msg->header.IdType              = FDCAN_EXTENDED_ID;
    msg->header.TxFrameType         = FDCAN_DATA_FRAME;
    msg->header.DataLength          = FDCAN_DLC_BYTES_8;
    msg->header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    msg->header.BitRateSwitch       = FDCAN_BRS_OFF;
    msg->header.FDFormat            = FDCAN_CLASSIC_CAN;
    msg->header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    msg->header.MessageMarker       = 0;
    memcpy(msg->data, responses[responseTail], 8);
    responseTail = (responseTail + 1) % RESPONSE_QUEUE_LEN;
    return true;
}
