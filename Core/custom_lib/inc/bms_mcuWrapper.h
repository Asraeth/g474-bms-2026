/*
 * bms_mcuWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "stdint.h"
#include <stdbool.h>
#include "main.h"

uint32_t bms_enterCritical(void);
void bms_exitCritical(uint32_t previousMask);
bool bms_spiContextAllowed(void);

// Chip select
void bms_csLow(void);
void bms_csHigh(void);


// Timer Functions
void bms_startTimer(void);
void bms_stopTimer(void);
uint32_t bms_getTimCount(void);  // Get time elapsed in us


// Wake up all the IC in the daisy chain
BMS_StatusTypeDef bms_wakeupChain(void);

// Microsecond delay
BMS_StatusTypeDef bms_delayUs(uint32_t us);

// Microsecond delay while preventing isospi sleep
BMS_StatusTypeDef bms_delayMsActive(uint32_t ms);
