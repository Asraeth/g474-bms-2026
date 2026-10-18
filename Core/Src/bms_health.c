#include "bms_health.h"
#include "main.h"
static volatile uint32_t latched;
void BMS_Health_Latch(uint32_t flags) {
 uint32_t mask=__get_PRIMASK(); __disable_irq(); latched|=flags; __set_PRIMASK(mask);
}
uint32_t BMS_Health_Get(void) { return latched; }
void BMS_Health_ClearRecovered(uint32_t flags) {
 uint32_t mask=__get_PRIMASK(); __disable_irq();
 latched&=~(flags&(BMS_ERR_BALANCE_STOP|BMS_ERR_CAN|BMS_ERR_CHARGE_TEMP)); __set_PRIMASK(mask);
}
