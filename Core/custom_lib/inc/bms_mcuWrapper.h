/*
 * bms_mcuWrapper.h
 *
 * The MCU side of the isoSPI link: chip select (to the ADBMS6822) and the microsecond delays it needs.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Chip select
void bms_csLow(void);
void bms_csHigh(void);

// Wakes every device in the daisy chain from sleep (and their isoSPI ports from idle)
bool bms_wakeupChain(void);

// Busy wait on TIM2 (1 us per count)
bool bms_delayUs(uint32_t us);

// Busy wait of ms milliseconds that keeps the isoSPI ports awake: a chip select pulse every millisecond
bool bms_delayMsActive(uint32_t ms);
bool bms_spiContextAllowed(void);
