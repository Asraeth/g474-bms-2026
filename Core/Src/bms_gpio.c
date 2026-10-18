/*
 * bms_gpio.c
 *
 * Board-level digital I/O, configured from bms_config.h
 *
 * Runs after MX_GPIO_Init() and takes over the pins listed in bms_config.h, so a new PCB only
 * needs the config file changed (plus the .ioc for alternate-function pins).
 */

#include "bms_gpio.h"
#include "main.h"


static void enablePortClock(GPIO_TypeDef* port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
}


static void configOutput(GPIO_TypeDef* port, uint16_t pin, bool openDrain, GPIO_PinState initial)
{
    GPIO_InitTypeDef init = {0};

    enablePortClock(port);
    HAL_GPIO_WritePin(port, pin, initial);      // Set the level before the pin starts driving

    init.Pin   = pin;
    init.Mode  = openDrain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &init);
}


static void configInput(GPIO_TypeDef* port, uint16_t pin, uint32_t pull)
{
    GPIO_InitTypeDef init = {0};

    enablePortClock(port);
    HAL_GPIO_DeInit(port, pin);     // Clears any EXTI CubeMX set up on this pin

    init.Pin  = pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    HAL_GPIO_Init(port, &init);
}


static GPIO_PinState levelFor(bool active, bool activeHigh)
{
    return (active == activeHigh) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}


void BMS_GPIO_EmergencySafe(void)
{
    configOutput(CFG_PIN_BYPASS_PORT,CFG_PIN_BYPASS_PIN,false,levelFor(false,CFG_BYPASS_ACTIVE_HIGH));
    configOutput(CFG_PIN_PRECHARGE_PORT,CFG_PIN_PRECHARGE_PIN,false,levelFor(false,CFG_PRECHARGE_ACTIVE_HIGH));
    configOutput(CFG_PIN_TSAC_READY_PORT,CFG_PIN_TSAC_READY_PIN,false,levelFor(false,CFG_TSAC_READY_ACTIVE_HIGH));
    configOutput(CFG_PIN_FAULT_PORT,CFG_PIN_FAULT_PIN,CFG_FAULT_OPEN_DRAIN,levelFor(true,CFG_FAULT_ACTIVE_HIGH));
}

void BMS_GPIO_Init(void)
{
    // Safety outputs first, both in their safe state
    configOutput(CFG_PIN_FAULT_PORT, CFG_PIN_FAULT_PIN, CFG_FAULT_OPEN_DRAIN,
                 levelFor(true, CFG_FAULT_ACTIVE_HIGH));
    configOutput(CFG_PIN_TSAC_READY_PORT, CFG_PIN_TSAC_READY_PIN, false,
                 levelFor(false, CFG_TSAC_READY_ACTIVE_HIGH));
    configOutput(CFG_PIN_PRECHARGE_PORT, CFG_PIN_PRECHARGE_PIN, false, levelFor(false, CFG_PRECHARGE_ACTIVE_HIGH));
    configOutput(CFG_PIN_BYPASS_PORT, CFG_PIN_BYPASS_PIN, false, levelFor(false, CFG_BYPASS_ACTIVE_HIGH));

    // isoSPI transceiver: CS idle high, second port disabled, master mode
    configOutput(CFG_PIN_ISOSPI_CS_PORT,    CFG_PIN_ISOSPI_CS_PIN,    false, GPIO_PIN_SET);
    configOutput(CFG_PIN_ISOSPI_CS2_PORT,   CFG_PIN_ISOSPI_CS2_PIN,   false, GPIO_PIN_SET);
    configOutput(CFG_PIN_ISOSPI_MSTR_PORT,  CFG_PIN_ISOSPI_MSTR_PIN,  false, GPIO_PIN_SET);
    configOutput(CFG_PIN_ISOSPI_MSTR2_PORT, CFG_PIN_ISOSPI_MSTR2_PIN, false, GPIO_PIN_SET);

    // Inputs. The charger button is polled, so this also disables the old EXTI on that pin.
    configInput(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN, GPIO_NOPULL);
    configInput(CFG_PIN_CHARGER_BTN_PORT, CFG_PIN_CHARGER_BTN_PIN,
                CFG_CHARGER_BTN_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN);
}


void BMS_GPIO_WriteFault(bool fault)
{
    HAL_GPIO_WritePin(CFG_PIN_FAULT_PORT, CFG_PIN_FAULT_PIN, levelFor(fault, CFG_FAULT_ACTIVE_HIGH));
}


void BMS_GPIO_WriteTsacReady(bool ready)
{
    HAL_GPIO_WritePin(CFG_PIN_TSAC_READY_PORT, CFG_PIN_TSAC_READY_PIN, levelFor(ready && BMS_CAN_ACTUATE_CONTACTORS, CFG_TSAC_READY_ACTIVE_HIGH));
}


void BMS_GPIO_WritePrecharge(bool closed)
{
    HAL_GPIO_WritePin(CFG_PIN_PRECHARGE_PORT, CFG_PIN_PRECHARGE_PIN, levelFor(closed && BMS_CAN_ACTUATE_CONTACTORS, CFG_PRECHARGE_ACTIVE_HIGH));
}


void BMS_GPIO_WriteBypass(bool closed)
{
    HAL_GPIO_WritePin(CFG_PIN_BYPASS_PORT, CFG_PIN_BYPASS_PIN, levelFor(closed && BMS_CAN_ACTUATE_CONTACTORS, CFG_BYPASS_ACTIVE_HIGH));
}


bool BMS_GPIO_IsSdcClosed(void)
{
    bool high = (HAL_GPIO_ReadPin(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN) == GPIO_PIN_SET);
    return high == CFG_SDC_IN_ACTIVE_HIGH;
}


bool BMS_GPIO_IsChargerButtonPressed(void)
{
    bool high = (HAL_GPIO_ReadPin(CFG_PIN_CHARGER_BTN_PORT, CFG_PIN_CHARGER_BTN_PIN) == GPIO_PIN_SET);
    return CFG_CHARGER_BTN_ACTIVE_LOW ? !high : high;
}


void BMS_GPIO_IsoSpiCs(bool low)
{
    HAL_GPIO_WritePin(CFG_PIN_ISOSPI_CS_PORT, CFG_PIN_ISOSPI_CS_PIN, low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
