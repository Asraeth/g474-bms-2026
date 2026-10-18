/*
 * bms_mcuWrapper.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#include "bms_mcuWrapper.h"
#include "main.h"
#include "bms_safety_config.h"

#define WAKEUP_DELAY BMS_CHAIN_WAKE_PULSE_MS

static TIM_HandleTypeDef *htim         = &htim2;       /* MCU TIM handler */

#define GPIO_PORT   BMS_CS_GPIO_Port
#define CS_PIN      BMS_CS_Pin

uint32_t bms_enterCritical(void)
{
    uint32_t previousMask = __get_PRIMASK();
    __disable_irq();
    return previousMask;
}

void bms_exitCritical(uint32_t previousMask)
{
    __set_PRIMASK(previousMask);
}

bool bms_spiContextAllowed(void)
{
    // Blocking HAL deadlines require a running SysTick. No SPI work from an
    // ISR or an interrupt-masked publication section.
    return __get_IPSR() == 0 && __get_PRIMASK() == 0;
}


void bms_csLow(void)
{
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
}


void bms_csHigh(void)
{
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}


void bms_startTimer(void)
{
    HAL_TIM_Base_Start(htim);
}


void bms_stopTimer(void)
{
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SetCounter(htim, 0);
}


// Get time elapsed in us
uint32_t bms_getTimCount(void)
{
    return((uint32_t)__HAL_TIM_GetCounter(htim));
}


// Wake up all the IC in the daisy chain
BMS_StatusTypeDef bms_wakeupChain(void)
{
    if (!bms_spiContextAllowed()) return BMS_ERR_COMMS;
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        bms_csLow();
        HAL_Delay(WAKEUP_DELAY);
        bms_csHigh();
        HAL_Delay(WAKEUP_DELAY);
    }
    return BMS_OK;
}


BMS_StatusTypeDef bms_delayUs(uint32_t us)
{
    if (!bms_spiContextAllowed() || HAL_TIM_Base_Start(htim) != HAL_OK) return BMS_ERR_COMMS;
    uint32_t started = HAL_GetTick();
    // Bound even a failed/stopped microsecond timer using the HAL timebase.
    const uint32_t deadlineMs = us / 1000U + BMS_ADBMS_SPI_TIMEOUT_MS;
    while (bms_getTimCount() < us) {
        if ((uint32_t)(HAL_GetTick() - started) >= deadlineMs) {
            bms_stopTimer();
            return BMS_ERR_COMMS;
        }
    }
    bms_stopTimer();
    return BMS_OK;
}


BMS_StatusTypeDef bms_delayMsActive(uint32_t ms)
{
    if (!bms_spiContextAllowed()) return BMS_ERR_COMMS;
    for (uint32_t i = 0; i < ms; i++)
    {
        bms_csLow();
        if (bms_delayUs(500) != BMS_OK) { bms_csHigh(); return BMS_ERR_COMMS; }
        bms_csHigh();
        if (bms_delayUs(500) != BMS_OK) return BMS_ERR_COMMS;
    }
    return BMS_OK;
}




