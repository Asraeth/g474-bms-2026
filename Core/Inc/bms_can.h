/*
 * bms_can.h
 *
 *  Created on: Jun 8, 2025
 *      Author: amrlxyz
 */

#ifndef INC_BMS_CAN_H_
#define INC_BMS_CAN_H_

#include "main.h"
#include "ivt_s.h"


#define INC_BMS_CAN_H_

#define BASE_CAN_ID 0xB000
#define BMS_CELL_CAN_ID_FIRST BASE_CAN_ID
#define BMS_CELL_CAN_ID_LAST (BMS_CELL_CAN_ID_FIRST + TOTAL_PACK_CELLS - 1U)
#define BMS_BOARD_CAN_ID_FIRST (BMS_CELL_CAN_ID_LAST + 1U)
#define BMS_BOARD_CAN_ID_LAST (BMS_BOARD_CAN_ID_FIRST + TOTAL_AD68 - 1U)

// Vehicle telemetry, after the 112 cell and 14 board IDs.
#define BMS_PACK_CURRENT_CAN_ID  (BASE_CAN_ID + TOTAL_PACK_CELLS + TOTAL_AD68)
#define BMS_PACK_U1_CAN_ID       (BMS_PACK_CURRENT_CAN_ID + 1U)
#define BMS_PACK_CROSSCHECK_CAN_ID (BMS_PACK_CURRENT_CAN_ID + 2U)
#define BMS_PACK_VALID          (1U << 0)
#define BMS_PACK_STALE          (1U << 1)
#define BMS_PACK_RECEIVED       (1U << 2)
#define BMS_PACK_CONFIG_VERIFIED (1U << 3)

#define CAN_BUFFER_LEN (TOTAL_PACK_CELLS + TOTAL_AD68 + 3U + 1U)

typedef struct
{
    uint8_t data[8];
    FDCAN_RxHeaderTypeDef header;
} CanRxMsg;


typedef struct
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header;
} CanTxMsg;



// --- Charger-Specific Definitions ---
// Define the CAN IDs for charger communication.
// Replace these with the actual IDs specified by your charger's manufacturer.
#define CHARGER_CONFIG_CAN_ID 0x1806E5F4
#define CHARGER_STATUS_CAN_ID 0x18FF50E5

// Structure to hold the parsed status data from the charger.
typedef struct {
    float output_voltage; // Volts
    float output_current; // Amps
    uint8_t hardware_fault      : 1;
    uint8_t over_temp_fault     : 1;
    uint8_t input_voltage_fault : 1;
    uint8_t charging_state      : 1;
    uint8_t comms_state         : 1;
    uint8_t reserved            : 3;
} ChargerStatus;

// Structure to hold the configuration we want to send to the charger.
typedef struct {
    float    target_voltage;        // Volts
    float    max_current;           // Amps
    uint8_t  disable_charging;      // 1 = Disable, 0 = Enable
} ChargerConfiguration;

extern ChargerStatus chargerStatus;


void BMS_CAN_Config(void);
void BMS_CAN_GetIVTSnapshot(IVTS_Snapshot *snapshot);

void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len);

void BMS_CAN_Test(void);

void BMS_CAN_SendMsg(CanTxMsg msg);

void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config, uint8_t* data);

_Static_assert(BMS_CELL_CAN_ID_LAST < BMS_BOARD_CAN_ID_FIRST, "Cell/board CAN ID overlap");
_Static_assert(BMS_BOARD_CAN_ID_LAST < BMS_PACK_CURRENT_CAN_ID, "Board/pack CAN ID overlap");
_Static_assert(BMS_PACK_CURRENT_CAN_ID < BMS_PACK_U1_CAN_ID && BMS_PACK_U1_CAN_ID < BMS_PACK_CROSSCHECK_CAN_ID,
        "Pack CAN ID overlap");
_Static_assert(BMS_PACK_CROSSCHECK_CAN_ID < CHARGER_CONFIG_CAN_ID && CHARGER_CONFIG_CAN_ID != CHARGER_STATUS_CAN_ID
        && BMS_PACK_CROSSCHECK_CAN_ID < CHARGER_STATUS_CAN_ID, "Charger CAN ID overlap");
_Static_assert(CAN_BUFFER_LEN == TOTAL_PACK_CELLS + TOTAL_AD68 + 4U, "Telemetry capacity mismatch");

#endif /* INC_BMS_CAN_H_ */
