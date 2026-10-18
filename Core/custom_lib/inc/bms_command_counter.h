/* ADBMS6830B Rev. 0, p53 and Table 50 (p57-59). No HAL dependency. */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "bms_topology.h"

typedef enum {
    BMS_CC_UNSUPPORTED, BMS_CC_READ, BMS_CC_INCREMENT,
    BMS_CC_WRITE, BMS_CC_RESET
} BMS_CounterCommand;

typedef struct {
    uint8_t expected, received;
    bool synchronized, received_valid, fault;
} BMS_CommandCounterDevice;

/* One instance per physical chain; device indices match received wire order. */
typedef struct {
    BMS_CommandCounterDevice device[TOTAL_IC];
} BMS_CommandCounterChain;

BMS_CounterCommand BMS_CC_Classify(const uint8_t command[2]);
void BMS_CC_Reset(BMS_CommandCounterChain *chain);
void BMS_CC_Advance(BMS_CommandCounterChain *chain);
void BMS_CC_Invalidate(BMS_CommandCounterChain *chain);
/* PEC errors are inputs: never learn a counter from a corrupt packet.
 * Sequence faults latch until an explicit, successfully transmitted reset. */
bool BMS_CC_Check(BMS_CommandCounterChain *chain, const uint8_t received[TOTAL_IC],
        const bool pec_errors[TOTAL_IC], bool errors[TOTAL_IC]);
