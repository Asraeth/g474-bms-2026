#include "bms_charger.h"
#include "bms_safety.h"
#include "bms_libWrapper.h"
#include "bms_health.h"
#include "bms_ivt.h"
#include "bms_gpio.h"
#include <math.h>
#include <string.h>
static bool haveStatus,haveMeasurements;
static uint32_t statusTick,measurementTick;
static int16_t le16(const uint8_t *d) {return (int16_t)((uint16_t)d[0]|((uint16_t)d[1]<<8));}
bool BMS_Charger_Healthy(void) {
 uint32_t now=HAL_GetTick();
 return haveStatus && haveMeasurements && now-statusTick<CFG_CHARGER_STATUS_TIMEOUT_MS && now-measurementTick<CFG_CHARGER_STATUS_TIMEOUT_MS && !chargerStatus.fault_bits && chargerStatus.state<=3;
}
void BMS_Charger_HandleFrame(uint32_t id,const uint8_t *data,uint32_t length) {
 if(length!=8)return;
 if(id==CHARGER_STATUS_CAN_ID) {
  if(data[0]>5 || data[3]||data[4]||data[5]||data[6]||data[7])return;
  chargerStatus.state=data[0];chargerStatus.fault_bits=(uint16_t)data[1]|((uint16_t)data[2]<<8);
  chargerStatus.hardware_fault=chargerStatus.fault_bits!=0;
  chargerStatus.over_temp_fault=(chargerStatus.fault_bits&0x80)!=0;
  chargerStatus.input_voltage_fault=(chargerStatus.fault_bits&0x0C)!=0;
  statusTick=HAL_GetTick();haveStatus=true;
 } else if(id==CHARGER_MEAS_CAN_ID) {
  chargerStatus.output_voltage=le16(data+2)*0.1f;chargerStatus.output_current=le16(data+4)*0.1f;
  measurementTick=HAL_GetTick();haveMeasurements=true;
 }
}
void BMS_Charger_Tick(void) {
 IvtReading ivt=BMS_IVT_GetReading();
 float cold=BMS_GetMinCellTempC(),hot=BMS_GetMaxCellTempC();
 bool tempOK=BMS_ControlDataValid() && isfinite(cold) && isfinite(hot) && cold>=CFG_CELL_CHARGE_TEMP_MIN_C && hot<=CFG_CELL_CHARGE_TEMP_MAX_C;
 if(tempOK)BMS_Health_ClearRecovered(BMS_ERR_CHARGE_TEMP);
 if(BMS_IsCharging()) {
  if(!tempOK)BMS_Health_Latch(BMS_ERR_CHARGE_TEMP);
  if(!BMS_Charger_Healthy())BMS_Health_Latch(BMS_ERR_CHARGER);
  if(!tempOK || !ivt.currentValid || !ivt.voltageValid || !BMS_GPIO_IsSdcClosed() || !BMS_Charger_Healthy() || BMS_Safety_IsFaultOutputActive())BMS_RequestSafeOutputs();
 }
}
void BMS_Charger_GetInhibit(CanTxMsg *msg) {
 memset(msg,0,sizeof(*msg));msg->header.Identifier=CHARGER_FAULT_CAN_ID;
 msg->header.IdType=FDCAN_EXTENDED_ID;msg->header.DataLength=FDCAN_DLC_BYTES_8;
 if(!BMS_CAN_ENABLE_CHARGER || !BMS_IsCharging() || !BMS_Charger_Healthy() || !BMS_ControlDataValid() || BMS_Safety_IsFaultOutputActive())msg->data[0]=0xAA;
}
