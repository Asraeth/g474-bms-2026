#pragma once
#include <stdint.h>
#include <limits.h>
#include <math.h>
/* ADBMS6830B signed little-endian ADC, 150 uV/bit and +1.5 V offset.
 * Integer microvolts avoid losing an inclusive boundary in float arithmetic. */
static inline float BMS_ADC_Voltage(const uint8_t p[2]) {
 int16_t raw=(int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8));
 if(raw==INT16_MIN)return NAN;
 int32_t microvolts=1500000+150*(int32_t)raw;
 return (float)((double)microvolts/1000000.0);
}
