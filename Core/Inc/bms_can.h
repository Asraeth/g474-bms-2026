/*
 * bms_can.h
 *
 * The three FDCANs (classic CAN): CAN1 car bus, CAN2 the IVT-S inside the TSAC, CAN3 the charger. Filters,
 * transmit queues and the receive interrupt, which hands frames to the modules they're for.
 */

#ifndef INC_BMS_CAN_H_
#define INC_BMS_CAN_H_

#include "main.h"
#include <stdbool.h>


// CAN1 burst: pack + status + summary + energy + cell, temperature and slave frames
#define CAN_BUFFER_LEN CFG_CAN_BURST_FRAMES

// Sent every CFG_CAN_FAST_PERIOD_MS; this many TX buffers are always kept free for them
#define CAN_FAST_FRAME_COUNT CFG_CAN_FAST_FRAMES

typedef struct
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header;
} CanTxMsg;



// --- Charger-Specific Definitions ---
// Define the CAN IDs for charger communication.
// Replace these with the actual IDs specified by your charger's manufacturer.
#define CHARGER_CONFIG_CAN_ID 0x00FFFFFF
#define CHARGER_FAULT_CAN_ID 0x001FFFFF
#define CHARGER_MEAS_CAN_ID 0x01000000
#define CHARGER_TEMP_CAN_ID 0x02000000
#define CHARGER_STATUS_CAN_ID 0x01100000

// Structure to hold the parsed status data from the charger.
typedef struct {
    uint16_t fault_bits;
    uint8_t state;
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


// Filters, start, receive interrupts. Start-up, before the timers run.
void BMS_CAN_Config(void);

// CAN1 measurement burst (timer interrupt, every CFG_CAN_TX_PERIOD_MS): copied, then sent one TX buffer at a
// time from the TX-FIFO-empty interrupt. A burst still going out when the next one comes is dropped and logged.
void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len);

// CAN1 frames that mustn't wait for the burst (fast frames, inverter limits, settings responses): straight into
// the TX FIFO, where CAN_FAST_FRAME_COUNT buffers are kept free for them. Timer interrupt.
void BMS_CAN_SendFast(const CanTxMsg* msgs, uint32_t count);

// CAN2: a command to the IVT; true once everything sent to it has been acknowledged; drop what hasn't been
void BMS_CAN_SendIvtCommand(uint32_t id, const uint8_t data[8]);
bool BMS_CAN_IsIvtTxIdle(void);
void BMS_CAN_AbortIvtTx(void);

// CAN3: the charger command
void BMS_CAN_SendCharger(const CanTxMsg* msg);

// Restarts any FDCAN that has gone bus-off. Safety tick.
void BMS_CAN_RecoverBusOff(void);

void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config, uint8_t* data);

#endif /* INC_BMS_CAN_H_ */
