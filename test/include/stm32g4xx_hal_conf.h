/*
 * Simulation wrapper for the project's HAL configuration.
 *
 * Found before Core/Inc on the include path. Includes the real configuration, then points every
 * peripheral the firmware touches directly at simulated register blocks instead of fixed addresses.
 */

#ifndef SIM_STM32G4XX_HAL_CONF_H
#define SIM_STM32G4XX_HAL_CONF_H

#include "../../Core/Inc/stm32g4xx_hal_conf.h"

extern TIM_TypeDef         sim_TIM2, sim_TIM3, sim_TIM15, sim_TIM16;
extern SPI_TypeDef         sim_SPI1;
extern FDCAN_GlobalTypeDef sim_FDCAN1, sim_FDCAN2, sim_FDCAN3;
extern USART_TypeDef       sim_LPUART1;
extern IWDG_TypeDef        sim_IWDG;
extern DBGMCU_TypeDef      sim_DBGMCU;
extern RCC_TypeDef         sim_RCC;
extern FLASH_TypeDef       sim_FLASH;

#undef  TIM2
#define TIM2    (&sim_TIM2)
#undef  TIM3
#define TIM3    (&sim_TIM3)
#undef  TIM15
#define TIM15   (&sim_TIM15)
#undef  TIM16
#define TIM16   (&sim_TIM16)
#undef  SPI1
#define SPI1    (&sim_SPI1)
#undef  FDCAN1
#define FDCAN1  (&sim_FDCAN1)
#undef  FDCAN2
#define FDCAN2  (&sim_FDCAN2)
#undef  FDCAN3
#define FDCAN3  (&sim_FDCAN3)
#undef  LPUART1
#define LPUART1 (&sim_LPUART1)
#undef  IWDG
#define IWDG    (&sim_IWDG)
#undef  DBGMCU
#define DBGMCU  (&sim_DBGMCU)
#undef  RCC
#define RCC     (&sim_RCC)
#undef  FLASH
#define FLASH   (&sim_FLASH)

// Flash reads (the settings page) come from the simulated flash
const void* sim_flash_ptr(uint32_t address);
#define BMS_FLASH_PTR(addr)     sim_flash_ptr(addr)

// The timer counter is a free-running register on the chip; here it follows simulated time
uint32_t sim_tim_get_counter(const TIM_HandleTypeDef* htim);
void     sim_tim_set_counter(TIM_HandleTypeDef* htim, uint32_t value);
#undef  __HAL_TIM_GET_COUNTER
#define __HAL_TIM_GET_COUNTER(h)        sim_tim_get_counter(h)
#undef  __HAL_TIM_SET_COUNTER
#define __HAL_TIM_SET_COUNTER(h, v)     sim_tim_set_counter((h), (v))

#endif /* SIM_STM32G4XX_HAL_CONF_H */
