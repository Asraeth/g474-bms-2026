/*
 * bms_utility.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "main.h"
#include <stdbool.h>
#include "bms_command_counter.h"

#define DATA_LEN       (6)       // Data
#define DATAPKT_LEN    (6 + 2)   // Data + DPEC
#define CMD_LEN        (2)       // Cmd
#define CMDPKT_LEN     (2 + 2)   // Cmd + PEC


//uint16_t bms_calcPec15(uint8_t *data, uint8_t len);
//
//uint16_t bms_calcPec10(uint8_t *pDataBuf, int nLength, uint8_t *commandCounter);

//void bms_spiTransmitCmd(uint8_t cmd[CMD_LEN]);
//
//void bms_spiTransmitData(uint8_t data[TOTAL_IC][DATA_LEN]);
//
//void bms_spiReceiveData(uint8_t rxData[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC]);


bool bms_checkRxPec(uint8_t rxData[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC], bool errorIndex[TOTAL_IC]);

typedef enum {
    BMS_TRANSPORT_OK, BMS_TRANSPORT_CONTEXT, BMS_TRANSPORT_TX,
    BMS_TRANSPORT_RX, BMS_TRANSPORT_POLL_TIMEOUT, BMS_TRANSPORT_COMMAND
} BMS_TransportFault;
BMS_TransportFault bms_getTransportFault(void);
bool bms_checkRxCounter(uint8_t cc[TOTAL_IC], const bool pecErrors[TOTAL_IC], bool errors[TOTAL_IC]);
void bms_getCounterState(BMS_CommandCounterChain *state);

BMS_StatusTypeDef bms_transmitCmd(uint8_t cmd[CMD_LEN]);

BMS_StatusTypeDef bms_transmitPoll(uint8_t cmd[CMD_LEN]);

BMS_StatusTypeDef bms_transmitData(uint8_t cmd[CMD_LEN], uint8_t txBuffer[TOTAL_IC][DATA_LEN]);

BMS_StatusTypeDef bms_receiveData(uint8_t cmd[CMD_LEN], uint8_t rxBuffer[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC]);



//void bms_transmitCmd(uint8_t cmd[CMD_LEN]);
//
//void bms_transmitPoll(uint8_t cmd[CMD_LEN]);
//
//void bms_transmitData(uint8_t cmd[CMD_LEN], uint8_t txBuffer[TOTAL_IC][DATA_LEN]);
//
//void bms_receiveData(uint8_t cmd[CMD_LEN], uint8_t rxBuffer[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC]);
