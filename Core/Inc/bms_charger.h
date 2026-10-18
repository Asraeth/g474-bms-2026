#pragma once
#include "bms_can.h"
bool BMS_Charger_Healthy(void);
void BMS_Charger_HandleFrame(uint32_t id,const uint8_t *data,uint32_t length);
void BMS_Charger_GetInhibit(CanTxMsg *msg);
void BMS_Charger_Tick(void);
