#pragma once

/* ===================================================================== */
/*  IVT-S current sensor (CAN2): IVT-S-500-U3-I-CAN2-12/24                */
/* ===================================================================== */

// IVT-S-500-U3-I-CAN2: +-500 A nominal range, three voltage inputs, galvanically isolated, and
// NO CAN termination on board: CAN2 needs a 120 R terminator at the IVT end too.
// U1: battery side of the AIRs (pack voltage). U2: vehicle side of the AIRs (precharge). U3: unused.
// Factory default IDs (IVT-S datasheet, Messages Overview)
#define CFG_IVT_ID_COMMAND          0x411
#define CFG_IVT_ID_RESPONSE         0x511
#define CFG_IVT_ID_RESULT_I         0x521
#define CFG_IVT_ID_RESULT_U1        0x522
#define CFG_IVT_ID_RESULT_U2        0x523
#define CFG_IVT_ID_RESULT_WH        0x528   // Energy counter, from I and U1

// Set to 1 if positive IVT current means discharge. Depends on mounting direction. VERIFY.
// Applied to power and energy as well.
#define CFG_IVT_CURRENT_DISCHARGE_POSITIVE 1

// The IVT-S datasheet contradicts itself on which results are enabled by default, and energy is off
// by default, so after start-up the BMS sets I, U1, U2 and Wh to cyclic and turns U3, T, W and As
// off. Nothing is stored in the IVT's EEPROM.
#define CFG_IVT_CONFIGURE_ON_BOOT   1

// Output periods. IVT-S limits (datasheet 4.1, 4.6 and 8.3), checked at compile time below:
//  - at most 1000 result messages per second in total, or its As/Wh counters can go wrong
//  - current: 1 ms minimum
//  - U1..U3 share one ADC: 2 ms minimum with two voltage channels on (3 ms with three); use a
//    multiple of 2 so each result averages the same number of samples
// Power isn't requested from the IVT, leaving the message budget to I, U1 and U2: the BMS
// multiplies I and U1 instead. The resulting message rate is listed in README.md.
#define CFG_IVT_CURRENT_PERIOD_MS   4
#define CFG_IVT_U1_PERIOD_MS        4
#define CFG_IVT_U2_PERIOD_MS        4
#define CFG_IVT_ENERGY_PERIOD_MS    100
#define CFG_IVT_MAX_MESSAGES_PER_S  1000
#define CFG_IVT_STARTUP_MS          400     // IVT re-/start-up time (datasheet max; ~190 ms measured)
// Once its current stops, wait this long for an alive message (a restart: ~190 ms measured) before configuring it
// anyway (stopped, or a slower restart; commands it gets while it boots are dropped, harmlessly). Restarting
// a stopped IVT this soon gets its data back before CFG_IVT_FAULT_TIMEOUT_MS.
#define CFG_IVT_RESTART_WAIT_MS     300
#define CFG_IVT_RETRY_INTERVAL_MS   500     // Reconfigure this often while no current arrives
// An IVT system error that is only CAN errors (a bus disturbance) is cleared this many times per power-up at
// most; any other system error, or more, needs the IVT power cycled (bms_ivt.c, "System error check")
#define CFG_IVT_AUTO_CLEAR_MAX      3
#define CFG_IVT_RECONFIG_INTERVAL_MS 2000   // ... and this often while it runs another configuration
// After a restart (its alive message on CFG_IVT_ID_RESPONSE) the IVT is configured straight away

// Readings older than these aren't used to control anything (inverter limits, SoC, precharge, CAN)
#define CFG_IVT_CURRENT_TIMEOUT_MS  100
#define CFG_IVT_VOLTAGE_TIMEOUT_MS  250     // U1 and U2
// Current or U1 missing this long is an AMS fault. Long enough to ride through an IVT restart.
#define CFG_IVT_FAULT_TIMEOUT_MS    450

// Result status nibble (DB1 high nibble). Bit 1: this result has an error / out of range,
// bit 3: system error. Bit 2 is an unspecified-channel error and inhibits all
// cached channels until independently refreshed with clean status.
#define CFG_IVT_STATUS_FAULT_MASK   0x0E


