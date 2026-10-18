/*
 * bms_mcuWrapper.c
 *
 * The MCU side of the isoSPI link, see bms_mcuWrapper.h.
 */

#include "bms_mcuWrapper.h"
#include "bms_gpio.h"
#include "main.h"

static TIM_HandleTypeDef *const htim = &htim2;


void bms_csLow(void)
{
    BMS_GPIO_IsoSpiCs(true);
}


void bms_csHigh(void)
{
    BMS_GPIO_IsoSpiCs(false);
}


// ADBMS6830B datasheet: one chip select pulse per device, 500 us <= wait < tIDLE between pulses
bool bms_spiContextAllowed(void)
{
    return __get_IPSR() == 0 && __get_PRIMASK() == 0;
}

bool bms_wakeupChain(void)
{
    if (!bms_spiContextAllowed()) return false;
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        bms_csLow();
        if (!bms_delayUs(CFG_ISOSPI_WAKE_PULSE_US)) { bms_csHigh(); return false; }
        bms_csHigh();
        if (!bms_delayUs(CFG_ISOSPI_WAKE_DELAY_US)) return false;
    }
    return true;
}


bool bms_delayUs(uint32_t us)
{
    if (!bms_spiContextAllowed() || HAL_TIM_Base_Start(htim) != HAL_OK) return false;
    uint32_t start = HAL_GetTick();
    while (__HAL_TIM_GetCounter(htim) < us)
    {
        if ((uint32_t)(HAL_GetTick()-start) >= us/1000U + CFG_SPI_TIMEOUT_MS) {
            HAL_TIM_Base_Stop(htim);
            __HAL_TIM_SetCounter(htim, 0);
            return false;
        }
    }
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SetCounter(htim, 0);
    return true;
}


bool bms_delayMsActive(uint32_t ms)
{
    if (!bms_spiContextAllowed()) return false;
    for (uint32_t i = 0; i < ms; i++)
    {
        bms_csLow();
        if (!bms_delayUs(500)) { bms_csHigh(); return false; }
        bms_csHigh();
        if (!bms_delayUs(500)) return false;
    }
    return true;
}
