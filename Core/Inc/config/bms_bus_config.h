#pragma once

/* ===================================================================== */
/*  CAN IDs (CAN1, 29-bit extended)                                      */
/* ===================================================================== */

/*
 * CAN buses. The bit timing itself is set in g474-bms.ioc (fdcan.c); tools/generate.py and the
 * simulation tests check it gives these rates.
 *   CAN1  car / datalogger bus, shared with the inverter (DTI HV500): BMS data out, TS request in
 *   CAN2  inside the TSAC: the IVT-S
 *   CAN3  charger
 */
#define CFG_CAN1_BITRATE            1000000 // Every node on the car bus has to match, the HV500 included
#define CFG_CAN2_BITRATE            500000  // IVT-S factory default
#define CFG_CAN3_BITRATE            500000  // Current charger gateway documentation; verify installed gateway revision

/*
 * CAN1 traffic, 29-bit IDs, little-endian:
 *  - Measurement burst every CFG_CAN_TX_PERIOD_MS, matching the measurement cycle. Pack, status,
 *    summary and energy frames, then per slave: voltage and temperature frames (4 values each) and
 *    a slave frame.
 *  - Fast frames every CFG_CAN_FAST_PERIOD_MS: IVT current, U1, U2, power and remaining energy.
 *    The burst never holds more than one of the three TX buffers, so these always get out.
 * The compile-time check below keeps the BMS's worst-case share of CAN1 under
 * CFG_CAN1_MAX_LOAD_PERCENT. Frame counts and bus load for the current settings are in README.md.
 */
#define CFG_CAN_TX_PERIOD_MS        100
// Not faster than the IVT sends I, U1 and U2 (checked below): faster would only repeat values
#define CFG_CAN_FAST_PERIOD_MS      4
#define CFG_CAN_POWER_W_PER_BIT     2.5f    // int16 power signals: +-81.9 kW at 2.5 W
#define CFG_CAN1_MAX_LOAD_PERCENT   50

#define CFG_CAN_V_FRAMES_PER_SLAVE  ((CFG_CELLS_PER_SLAVE + 3) / 4)     // 4 cell voltages per frame
#define CFG_CAN_T_FRAMES_PER_SLAVE  ((CFG_TEMPS_PER_SLAVE + 3) / 4)     // 4 temperatures per frame
#define CFG_CAN_MEAS_FRAMES         (CFG_NUM_SLAVES * (CFG_CAN_V_FRAMES_PER_SLAVE + 2 * CFG_CAN_T_FRAMES_PER_SLAVE + 2))
#define CFG_CAN_BURST_FRAMES        (CFG_CAN_MEAS_FRAMES + 8)   // + pack, status, summary, energy
#define CFG_CAN_FAST_FRAMES         2
#define CFG_CAN_FRAME_BITS_MAX      160     // Extended 8-byte frame with worst-case bit stuffing

// The inverter (DTI HV500, its CAN2 port) shares CAN1. DTI Inverter CAN Manual v2.4, CAN2 map "V24":
// 29-bit ID = (packet ID << 8) | node ID, commands 0x01-0x0C, status 0x20-0x24, so its IDs are below
// 0x2500 (0x3FFF reserved here). Checked below.
// Use the HV500's extended-ID mode: every BMS ID starts with eleven 0 bits, so in standard-ID mode the
// inverter's frames would lose arbitration to every BMS frame.
#define CFG_CAN1_INVERTER_ID_MAX    0x3FFF

// Inverter limit commands sent by the BMS (DTI extended ID: packet << 8 | node ID)
#define CFG_CAN_ID_INVERTER_DC_LIMIT        ((0x0A << 8) | CFG_INVERTER_NODE_ID)
#define CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT  ((0x0B << 8) | CFG_INVERTER_NODE_ID)

#define CFG_CAN_ID_CELL_V_BASE      0xB000  // + slave * CFG_CAN_V_FRAMES_PER_SLAVE + frame
#define CFG_CAN_ID_CELL_T_BASE      0xB080  // + slave * CFG_CAN_T_FRAMES_PER_SLAVE + frame
#define CFG_CAN_ID_SLAVE_BASE       0xB100  // + slave
#define CFG_CAN_ID_PACK             0xB200
#define CFG_CAN_ID_STATUS           0xB201
#define CFG_CAN_ID_SUMMARY          0xB202
#define CFG_CAN_ID_ENERGY           0xB203
#define CFG_CAN_ID_FAST_1           0xB204  // IVT current and U1 (pack voltage), every CFG_CAN_FAST_PERIOD_MS
#define CFG_CAN_ID_FAST_2           0xB205  // IVT U2 (vehicle side), power, remaining energy
#define CFG_CAN_ID_LIMITS           0xB206  // Live power and current limits, every CFG_INVERTER_LIMIT_PERIOD_MS
#define CFG_CAN_ID_CONFIG_RESPONSE  0xB207  // Answers to CFG_CAN_ID_CONFIG_REQUEST, at most one per fast period


