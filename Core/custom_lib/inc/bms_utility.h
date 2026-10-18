/*
 * bms_utility.h
 *
 * isoSPI transfers to the ADBMS6830B daisy chain (through the ADBMS6822): commands with their PEC15, register
 * group writes and reads with the data PEC10, conversion polls, and the command counter bookkeeping.
 *
 * Data order: index 0 is the device nearest the master (the first one on the chain). Reads arrive in that order;
 * writes are sent last device first, which bms_transmitData takes care of.
 */

#pragma once

#include "main.h"
#include <stdbool.h>
#include "bms_command_counter.h"

typedef enum { BMS_TRANSPORT_OK, BMS_TRANSPORT_CONTEXT, BMS_TRANSPORT_TX,
    BMS_TRANSPORT_RX, BMS_TRANSPORT_POLL_TIMEOUT, BMS_TRANSPORT_COMMAND } BMS_TransportFault;
BMS_TransportFault bms_getTransportFault(void);
void bms_getCounterState(BMS_CommandCounterChain *state);
bool bms_checkRxCounter(const uint8_t cc[TOTAL_IC], const bool pecErrors[TOTAL_IC], bool errors[TOTAL_IC]);

#define DATA_LEN       (6)       // Register group
#define DATAPKT_LEN    (6 + 2)   // Register group + command counter / PEC10
#define CMD_LEN        (2)       // Command
#define CMDPKT_LEN     (2 + 2)   // Command + PEC15


// All return false on an SPI error or timeout (bms_transmitPoll also when the conversion doesn't finish in
// timeoutMs). Each sends one command in one chip select frame. The register group arrays aren't const:
// ISO C before C23 doesn't convert uint8_t (*)[6] to const uint8_t (*)[6].
bool bms_transmitCmd(const uint8_t cmd[CMD_LEN]);
bool bms_transmitPoll(const uint8_t cmd[CMD_LEN], uint32_t timeoutMs);
bool bms_transmitData(const uint8_t cmd[CMD_LEN], uint8_t txBuffer[TOTAL_IC][DATA_LEN]);
bool bms_receiveData(const uint8_t cmd[CMD_LEN], uint8_t rxBuffer[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC],
                     uint8_t rxCc[TOTAL_IC]);

// Checks each device's data PEC10 (which covers its command counter too). Sets errorIndex[ic] for every device
// that failed; returns true if any did.
bool bms_checkRxPec(uint8_t rxData[TOTAL_IC][DATA_LEN], const uint16_t rxPec[TOTAL_IC],
                    const uint8_t rxCc[TOTAL_IC], bool errorIndex[TOTAL_IC]);

// Command counter every device should report now, from the commands sent since the last RSTCC / SRST
uint8_t bms_expectedCc(void);
