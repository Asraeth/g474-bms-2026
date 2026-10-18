#include "bms_charger_codec.h"
#include <math.h>
#include <string.h>
static void bcd(uint32_t v,uint8_t *out) {
 for(int i=2;i>=0;i--) { uint8_t low=v%10;v/=10;out[i]=(uint8_t)(((v%10)<<4)|low);v/=10; }
}
static bool decimal(const uint8_t *in,uint32_t *out) {
 *out=0;for(int i=0;i<3;i++) {unsigned hi=in[i]>>4,lo=in[i]&15;if(hi>9||lo>9)return false;*out=*out*100+hi*10+lo;}return true;
}
bool BMS_Charger_Encode(float volts,float amps,uint8_t data[8]) {
 memset(data,0,8);
 if(!isfinite(volts)||!isfinite(amps)||volts<0||amps<0||volts>999.999f||amps>9.99999f)return false;
 bcd((uint32_t)floor((double)volts*1000+0.5),data);
 bcd((uint32_t)floor((double)amps*100000+0.5),data+3);return true;
}
bool BMS_Charger_DecodeSetpoint(const uint8_t data[8],float *volts,float *amps) {
 uint32_t v,i;if(data[6]||data[7]||!decimal(data,&v)||!decimal(data+3,&i))return false;
 *volts=v/1000.0f;*amps=i/100000.0f;return true;
}
