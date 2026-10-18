/*
 * bms_cmdlist.h
 *
 * The ADBMS6830B commands the firmware sends (datasheet Rev. 0, Table 50 "Command Codes"). Each is the 11-bit
 * command code in two bytes, MSB first; bms_utility.c adds the PEC15. A read returns the register group from
 * every device in the chain (6 bytes + command counter + PEC10 each), a write sends one group per device.
 *
 * The three conversion commands carry option bits, so they are bitfield structs filled from their _default
 * and changed per call. Bitfield layout is up to the compiler: test/check_layout.sh checks that both the ARM and
 * the PC compiler produce the datasheet's bytes (Tables 18 and 23).
 */

#pragma once

#include <stdint.h>


/* ---- Register groups ------------------------------------------------------------------------- */

// Configuration register groups A and B (Tables 55, 56)
extern const uint8_t WRCFGA[2];
extern const uint8_t WRCFGB[2];
extern const uint8_t RDCFGA[2];
extern const uint8_t RDCFGB[2];

// PWM register groups A (PWM1-12) and B (PWM13-16): balancing duty, 4 bits per cell input
extern const uint8_t WRPWMA[2];
extern const uint8_t WRPWMB[2];
extern const uint8_t RDPWMA[2];
extern const uint8_t RDPWMB[2];

// S-ADC cell voltage results, 3 channels per group (group F holds C16 only)
extern const uint8_t RDSVA[2];
extern const uint8_t RDSVB[2];
extern const uint8_t RDSVC[2];
extern const uint8_t RDSVD[2];
extern const uint8_t RDSVE[2];
extern const uint8_t RDSVF[2];

// AUX results: A-C GPIO1-9, D GPIO10 / VMV / VPV; status group A: VREF2, ITMP (die temperature)
extern const uint8_t RDAUXA[2];
extern const uint8_t RDAUXB[2];
extern const uint8_t RDAUXC[2];
extern const uint8_t RDAUXD[2];
extern const uint8_t RDSTATA[2];


/* ---- Polls and control ----------------------------------------------------------------------- */

// Poll S-ADC / AUX conversion: SDO reads 0x00 while converting, 0xFF once done
extern const uint8_t PLSADC[2];
extern const uint8_t PLAUX1[2];

extern const uint8_t RDSID[2];      // Serial ID
extern const uint8_t MUTE[2];
extern const uint8_t UNMUTE[2];
extern const uint8_t RSTCC[2];      // Command counters to 0
extern const uint8_t SRST[2];       // Soft reset: every device to sleep, registers to their reset values


/* ---- Conversion commands (Table 18: ADCV, ADSV; Table 23: ADAX) --------------------------------- */

// Start C-ADC cell conversion
typedef struct
{
    uint8_t       RD  : 1;      // Redundant measurement by the S-ADC
    uint8_t       RES2: 2;
    uint8_t       X   : 5;

    uint8_t       OW  : 2;      // Open wire switches: 00 off, 01 even, 10 odd, 11 all
    uint8_t       RSTF: 1;      // Reset the IIR filter
    uint8_t       RES0: 1;
    uint8_t       DCP : 1;      // 1: PWM discharge carries on during the conversion
    uint8_t       RES1: 2;
    uint8_t       CONT: 1;      // Continuous
} ADCV_t;

extern const ADCV_t ADCV_default;

// Start S-ADC cell conversion
typedef struct
{
    uint8_t       RES2: 3;
    uint8_t       X   : 5;

    uint8_t       OW  : 2;      // Open wire switches: 00 off, 01 even, 10 odd, 11 all
    uint8_t       RES0: 2;
    uint8_t       DCP : 1;      // 1: PWM discharge carries on during the conversion
    uint8_t       RES1: 2;
    uint8_t       CONT: 1;      // Continuous
} ADSV_t;

extern const ADSV_t ADSV_default;

// Start AUX conversion
typedef struct
{
    uint8_t       OW  : 1;      // Open wire current sources on the GPIOs
    uint8_t       RES0: 2;
    uint8_t       X   : 5;

    uint8_t       CH  : 4;      // Channel select (CH4..CH0 = 0: all channels)
    uint8_t       RES1: 2;
    uint8_t       CH4 : 1;
    uint8_t       PUP : 1;      // OW: 0 pull-down, 1 pull-up
} ADAX_t;

extern const ADAX_t ADAX_default;
