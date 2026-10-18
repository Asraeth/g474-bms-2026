/*
 * bms_gpio.h
 *
 * Board-level digital I/O, configured from bms_config.h
 */

#ifndef INC_BMS_GPIO_H_
#define INC_BMS_GPIO_H_

#include <stdbool.h>

void BMS_GPIO_Init(void);

void BMS_GPIO_WriteFault(bool fault);
void BMS_GPIO_WriteTsacReady(bool ready);
void BMS_GPIO_WritePrecharge(bool closed);
void BMS_GPIO_WriteBypass(bool closed);

bool BMS_GPIO_IsSdcClosed(void);
bool BMS_GPIO_IsChargerButtonPressed(void);

void BMS_GPIO_IsoSpiCs(bool low);

#endif /* INC_BMS_GPIO_H_ */

void BMS_GPIO_EmergencySafe(void);
