#pragma once
/* Candidate Nucleo/controller routing. Selected board definitions own these aliases. */
/*  MCU pins                                                             */

/*
 * Plain digital I/O is configured from here by BMS_GPIO_Init(). If you move a pin, also remove or
 * relabel the old pin in g474-bms.ioc so CubeMX doesn't keep driving it.
 *
 * Peripheral pins are tied to alternate functions and are set in g474-bms.ioc:
 *   SPI1 (isoSPI)  PA5 SCK, PA6 MISO, PA7 MOSI
 *   FDCAN1 (car)   PA11 RX, PA12 TX
 *   FDCAN2 (TSAC)  PB12 RX, PB13 TX
 *   FDCAN3 (charger) PA8 RX, PA15 TX
 *   LPUART1 debug  PA2 TX, PA3 RX
 *   TIM3_CH2 fan   PA4
 *   TIM15_CH1 IMD  PB14
 */

// AMS fault output to the hardware latch. Fail-safe: open-drain, released (high) = FAULT, so a
// reset or unpowered MCU also signals a fault through the external pull-up.
#define CFG_PIN_FAULT_PORT          GPIOC
#define CFG_PIN_FAULT_PIN           GPIO_PIN_1
#define CFG_FAULT_ACTIVE_HIGH       1
#define CFG_FAULT_OPEN_DRAIN        1

// Relay that puts the TSAC into the ready state. Needs a pull-down on the PCB so it is off in reset.
#define CFG_PIN_TSAC_READY_PORT     GPIOC
#define CFG_PIN_TSAC_READY_PIN      GPIO_PIN_8
#define CFG_TSAC_READY_ACTIVE_HIGH  1

// Precharge relay and bypass (AIR) drivers. Need pull-downs on the PCB so they are off in reset.
#define CFG_PIN_PRECHARGE_PORT      GPIOC
#define CFG_PIN_PRECHARGE_PIN       GPIO_PIN_6
#define CFG_PIN_BYPASS_PORT         GPIOC
#define CFG_PIN_BYPASS_PIN          GPIO_PIN_9
#define CFG_PRECHARGE_ACTIVE_HIGH   1
#define CFG_BYPASS_ACTIVE_HIGH      1

// Shutdown circuit sense input
#define CFG_PIN_SDC_IN_PORT         GPIOC
#define CFG_PIN_SDC_IN_PIN          GPIO_PIN_2
#define CFG_SDC_IN_ACTIVE_HIGH      1       // Pin high = SDC closed

// Charger start/stop button
#define CFG_PIN_CHARGER_BTN_PORT    GPIOC
#define CFG_PIN_CHARGER_BTN_PIN     GPIO_PIN_0
#define CFG_CHARGER_BTN_ACTIVE_LOW  1

// ADBMS6822 isoSPI transceiver
#define CFG_PIN_ISOSPI_CS_PORT      GPIOB
#define CFG_PIN_ISOSPI_CS_PIN       GPIO_PIN_6
#define CFG_PIN_ISOSPI_CS2_PORT     GPIOC   // Second port, unused: held high
#define CFG_PIN_ISOSPI_CS2_PIN      GPIO_PIN_7
#define CFG_PIN_ISOSPI_MSTR_PORT    GPIOA   // Held high (master mode)
#define CFG_PIN_ISOSPI_MSTR_PIN     GPIO_PIN_10
#define CFG_PIN_ISOSPI_MSTR2_PORT   GPIOB
#define CFG_PIN_ISOSPI_MSTR2_PIN    GPIO_PIN_4
