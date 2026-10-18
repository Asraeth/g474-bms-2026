/* Integration regressions: real production APIs; independent expected values. */
#include "bms_utility.h"
#include "bms_health.h"
#include "bms_charger.h"
#include "bms_charger_codec.h"
#include "bms_adc.h"
void BMS_WriteFanDuty(float duty);
static void test_numeric_and_imd_contract(void) {
 for(int code=INT16_MIN;code<=INT16_MAX;code++) {
  uint8_t bytes[2]={(uint8_t)code,(uint8_t)((uint16_t)code>>8)};
  float value=BMS_ADC_Voltage(bytes);
  if(code==INT16_MIN)SIM_CHECK(isnan(value),"reset code accepted");
  else {double golden=(1500000LL+150LL*code)/1000000.0;SIM_CHECK(fabs((double)value-golden)<0.000001,"ADC raw %d endian/offset/scaling",code);}
 }
 MX_TIM15_Init();
 const uint32_t vectors[][2]={{1280000,640000},{1280000,0},{1280000,1280000},{1,UINT32_MAX},{UINT32_MAX,UINT32_MAX},{UINT32_MAX,1},{3,2},{7,1},{100,33}};
 for(unsigned i=0;i<sizeof(vectors)/sizeof(*vectors);i++) {
  BmsImdStatus before=BMS_GetImdStatus();
  htim15.Instance->CCR1=vectors[i][0];htim15.Instance->CCR2=vectors[i][1];HAL_TIM_IC_CaptureCallback(&htim15);
  BmsImdStatus pending=BMS_GetImdStatus();SIM_CHECK(pending.period==before.period,"IMD ISR performed publication/division");
  BMS_ProcessImdCapture();BmsImdStatus after=BMS_GetImdStatus();
  SIM_CHECK(after.freq==CFG_IMD_TIMER_CLOCK_HZ/vectors[i][0] && after.period==vectors[i][0],"IMD period/frequency");
  float expected=(float)((double)vectors[i][1]*100.0/vectors[i][0]);SIM_CHECK(after.duty==expected,"IMD full-width numeric result");
 }
 BmsImdStatus before=BMS_GetImdStatus();htim15.Instance->CCR1=0;HAL_TIM_IC_CaptureCallback(&htim15);BMS_ProcessImdCapture();BmsImdStatus after=BMS_GetImdStatus();
 SIM_CHECK(after.period==0 && after.freq==before.freq && after.duty==before.duty,"zero-period guard");
 const float rejected[]={NAN,INFINITY,-INFINITY,-1,0,1.1f};
 for(unsigned i=0;i<sizeof(rejected)/sizeof(*rejected);i++){TIM3->CCR2=12345;BMS_WriteFanDuty(rejected[i]);SIM_CHECK(TIM3->CCR2==12345,"fan nonfinite/out-of-range register write");}
}
static void test_limiter_boundaries(void) {
 BmsSettings s={.powerLimit_W=80000,.dischargeCurrent_A=200,.tempLimit_C=60,.regenEnabled=true,.regenCurrent_A=35};
 const float voltage[]={399,400,401,450,470.4f};
 for(unsigned i=0;i<sizeof(voltage)/sizeof(*voltage);i++) {
  InverterLimits l=BMS_Inverter_ComputeLimits(&s,25,voltage[i],200,false,80000);
  float terminal=voltage[i]+CFG_PACK_RESISTANCE_MAX_OHM*fmaxf(200-l.discharge_A,0);
  SIM_CHECK(l.discharge_A>0 && l.discharge_A<=200 && l.discharge_A*terminal<=80001,"80kW/current intersection at %.1fV: %.3fA %.1fW",voltage[i],l.discharge_A,l.discharge_A*terminal);
  SIM_CHECK(l.dischargePower_W<=80000.1f,"published power exceeds ceiling");
 }
 const float temp[]={-0.1f,0,20,45,45.1f};
 for(unsigned i=0;i<5;i++) {
  InverterLimits l=BMS_Inverter_ComputeLimits(&s,temp[i],400,200,false,80000);
  SIM_CHECK(l.discharge_A>0,"charge temperature suppressed independent drive");
  SIM_CHECK((l.charge_A>0)==(temp[i]>=0 && temp[i]<=45),"regen charge window at %.1fC",temp[i]);
 }
 InverterLimits l=BMS_Inverter_ComputeLimits(&s,25,NAN,200,false,80000);
 SIM_CHECK(l.faulted && l.discharge_A==0 && l.charge_A==0,"NaN voltage not fail-safe");
 l=BMS_Inverter_ComputeLimits(&s,25,400,200,true,80000);
 SIM_CHECK(l.discharge_A==0 && l.charge_A==0,"fault did not zero both limits");
}
static void test_charger_documented_codec(void) {
 const uint8_t golden[8]={0x48,0x27,0x35,0x68,0x12,0x34,0,0};
 uint8_t actual[8];float v=0,a=0;
 SIM_CHECK(BMS_Charger_DecodeSetpoint(golden,&v,&a),"documented charger frame rejected");
 SIM_CHECK(fabsf(v-482.735f)<0.0001f && fabsf(a-6.81234f)<0.00001f,"documented BCD scaling/endian wrong");
 SIM_CHECK(BMS_Charger_Encode(482.735f,6.81234f,actual) && memcmp(golden,actual,8)==0,"BCD encoder differs from documentation");
 SIM_CHECK(!BMS_Charger_Encode(450,35,actual) && !BMS_Charger_Encode(NAN,1,actual) && !BMS_Charger_Encode(450,-1,actual),"unrepresentable charger setpoint accepted");
 uint8_t bad[8]={0xFA,0,0,0,0,0,0,0};SIM_CHECK(!BMS_Charger_DecodeSetpoint(bad,&v,&a),"invalid BCD accepted");
}
static void test_versioned_config(void) {
 BMS_Settings_Init();BmsSettings before=BMS_Settings_Get();CanTxMsg response;
 uint8_t req[8]={SETTINGS_CMD_WRITE,SETTING_POWER_LIMIT,1,19,0,0,0,0};
 BMS_Settings_HandleRequest(req,8);BMS_Settings_Service();
 SIM_CHECK(BMS_Settings_GetResponse(&response) && response.data[2]==SETTINGS_ERR_VERSION && response.data[3]>>3==19,"version/token rejection missing");
 SIM_CHECK(BMS_Settings_Get().powerLimit_W==before.powerLimit_W,"old protocol changed a setting");
 req[2]=CFG_CONFIG_VERSION;BMS_Settings_HandleRequest(req,7);BMS_Settings_Service();SIM_CHECK(!BMS_Settings_GetResponse(&response),"partial request accepted");
 req[0]=SETTINGS_CMD_READ;req[3]=3;BMS_Settings_HandleRequest(req,8);BMS_Settings_Service();
 SIM_CHECK(BMS_Settings_GetResponse(&response) && response.data[2]==SETTINGS_OK && response.data[3]>>3==3,"valid version/token exchange failed");
}
static BMS_AcquisitionStatus transactionBefore;
static uint64_t transactionInjected;
static void ev_transaction_observe(void *unused) {
 (void)unused;BMS_AcquisitionStatus after;BMS_GetAcquisitionStatus(&after);
 if(after.cellsValid && sim_now_us()-transactionInjected<SIM_MS(90)) {
  sim_schedule(sim_now_us()+SIM_MS(1),ev_transaction_observe,NULL);return;
 }
 SIM_CHECK(after.generation==transactionBefore.generation && !after.cellsValid && !after.auxValid && !BMS_ControlDataValid(),"failed cycle published partial/fresh data");
 SIM_CHECK(after.pecFault,"primary PEC diagnosis lost through discharge stop");
 BMS_Inverter_Update();InverterLimits l=BMS_Inverter_GetLimits();
 SIM_CHECK(l.discharge_A==0 && l.charge_A==0 && !BMS_IsCharging(),"failed acquisition did not inhibit energetic consumers");
}
static void ev_transaction_failure(void *unused) {
 (void)unused;BMS_GetAcquisitionStatus(&transactionBefore);transactionInjected=sim_now_us();
 sim_adbms_corrupt_reads(0,1);sim_schedule(sim_now_us()+SIM_MS(1),ev_transaction_observe,NULL);
}
static void test_transaction_contract(void) {
 board_init(true);sim_schedule(SIM_S(3),ev_transaction_failure,NULL);sim_run_firmware(SIM_S(5));common_checks(true);
 for(int ic=0;ic<TOTAL_IC;ic++)for(int ch=8;ch<16;ch++)SIM_CHECK(sim_adbms_pwm(ic,ch)==0,"unused physical cell discharge enabled");
}
#if BMS_BOARD_PROFILE == BMS_PROFILE_BENCH
static void test_bench_contract(void) {
 SIM_CHECK(BMS_PROFILE_IS_BENCH && TOTAL_IC==1,"bench needs exactly one device");
 board_init(true);sim_schedule(SIM_S(2),ev_ts_on,NULL);sim_run_firmware(SIM_S(3));
 BMS_EnableBalancing(true);BMS_EnableCharging(true);BMS_Inverter_Update();
 BMS_AcquisitionStatus a;BMS_GetAcquisitionStatus(&a);
 SIM_CHECK(a.cellsValid && a.auxValid && !a.celsiusValid,"raw acquisition and unconfirmed calibration not separated");
 SIM_CHECK(!BMS_IsCharging() && !precharge_on() && !bypass_on() && !sim_tsac_ready(),"bench energetic output enabled");
 InverterLimits l=BMS_Inverter_GetLimits();SIM_CHECK(l.discharge_A==0 && l.charge_A==0,"bench inverter enabled");
 for(int ch=0;ch<16;ch++)SIM_CHECK(sim_adbms_pwm(0,ch)==0,"bench discharge active");
 const sim_can_frame *id=last_sent(CFG_CAN_ID_IDENTITY),*raw=last_sent(CFG_CAN_ID_RAW_AUX_BASE);
 SIM_CHECK(id && id->data[3]==1 && id->data[7]==1 && raw && le16(raw->data)!=INT16_MIN,"bench identity/raw AUX unavailable");
}

#endif
