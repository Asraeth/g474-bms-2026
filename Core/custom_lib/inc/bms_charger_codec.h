#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Current gateway README: 6 decimal digits, most-significant BCD byte first. */
bool BMS_Charger_Encode(float volts,float amps,uint8_t data[8]);
bool BMS_Charger_DecodeSetpoint(const uint8_t data[8],float *volts,float *amps);
