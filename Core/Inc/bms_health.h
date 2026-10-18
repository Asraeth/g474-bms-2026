#pragma once
#include <stdint.h>
void BMS_Health_Latch(uint32_t flags);
uint32_t BMS_Health_Get(void);
void BMS_Health_ClearRecovered(uint32_t flags);
