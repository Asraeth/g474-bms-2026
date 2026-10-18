/*
 * bms_settings.h
 *
 * Runtime settings: inverter limits below the built-in ones and the charger setpoints, read and changed
 * over CAN1 (the GUI) and kept in flash. Ranges and defaults are in bms_config.h ("Runtime settings").
 *
 * CAN1, extended IDs, little-endian:
 *   BMS_Config_Request  (CFG_CAN_ID_CONFIG_REQUEST, to the BMS)
 *     byte 0 command, byte 1 setting (SettingId, 0 for save/defaults/read all), bytes 4-7 int32 value (write)
 *   BMS_Config_Response (CFG_CAN_ID_CONFIG_RESPONSE, from the BMS, one per setting answered)
 *     byte 0 command, byte 1 setting, byte 2 result (SettingsResult), byte 3 flags (SETTINGS_FLAG_*),
 *     bytes 4-7 int32 value now in force
 * Values are integers: power in W, currents in 0.1 A, temperature in 0.1 C, voltage in 0.1 V, regen 0/1.
 */

#ifndef INC_BMS_SETTINGS_H_
#define INC_BMS_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>
#include "bms_can.h"

typedef enum
{
    SETTING_POWER_LIMIT = 1,        // W
    SETTING_DISCHARGE_CURRENT,      // 0.1 A
    SETTING_TEMP_LIMIT,             // 0.1 C
    SETTING_REGEN_ENABLED,          // 0 = off, 1 = on
    SETTING_REGEN_CURRENT,          // 0.1 A
    SETTING_CHARGE_CURRENT,         // 0.1 A
    SETTING_CHARGE_VOLTAGE,         // 0.1 V
    SETTING_COUNT                   // One past the last
} SettingId;

typedef enum
{
    SETTINGS_CMD_READ     = 1,      // One setting
    SETTINGS_CMD_WRITE    = 2,      // One setting, applies at once (not saved)
    SETTINGS_CMD_SAVE     = 3,      // Store all settings in flash
    SETTINGS_CMD_DEFAULTS = 4,      // All settings back to the built-in values (not saved)
    SETTINGS_CMD_READ_ALL = 5,      // One response per setting
} SettingsCommand;

typedef enum
{
    SETTINGS_OK                   = 0,
    SETTINGS_ERR_UNKNOWN_SETTING  = 1,
    SETTINGS_ERR_RANGE            = 2,  // Value outside the setting's range: nothing changed
    SETTINGS_ERR_TS_ACTIVE        = 3,  // Save refused while the TS is active or charging
    SETTINGS_ERR_FLASH            = 4,  // Flash erase, write or read-back failed
    SETTINGS_ERR_UNKNOWN_COMMAND  = 5,
    SETTINGS_ERR_VERSION          = 7,
    SETTINGS_ERR_MALFORMED        = 8,
    SETTINGS_ERR_BUSY             = 6,  // Save refused: the page was written less than CFG_SETTINGS_SAVE_INTERVAL_MS ago
} SettingsResult;

#define SETTINGS_FLAG_UNSAVED   0x01    // Settings differ from what flash holds
#define SETTINGS_FLAG_MODIFIED  0x02    // Settings differ from the built-in values
#define SETTINGS_FLAG_STORED    0x04    // Flash holds saved settings

// The settings in force, in the units the firmware works in
typedef struct
{
    float powerLimit_W;
    float dischargeCurrent_A;
    float tempLimit_C;
    bool  regenEnabled;
    float regenCurrent_A;
    float chargeCurrent_A;
    float chargeVoltage_V;
    bool  modified;                 // Any setting differs from the built-in value
} BmsSettings;

// Start-up, before CAN: loads the saved settings (built-in values if there are none or they're invalid)
void BMS_Settings_Init(void);

// Consistent snapshot, safe from interrupts
BmsSettings BMS_Settings_Get(void);

// CAN1 RX interrupt: queues a BMS_Config_Request
void BMS_Settings_HandleRequest(const uint8_t* data, uint32_t length);

// Main loop: works through the queued requests (a save erases and writes the settings page)
void BMS_Settings_Service(void);

// Timer interrupt: the next BMS_Config_Response to send, false if none is waiting
bool BMS_Settings_GetResponse(CanTxMsg* msg);

#endif /* INC_BMS_SETTINGS_H_ */
