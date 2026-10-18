/*
 * layout_probe.c
 *
 * The ADBMS6830B command and register structs are C bitfields, whose layout is up to the compiler.
 * This file fills each one with known field values. test/check_layout.sh compiles it with the ARM
 * compiler (the real firmware) and with the PC compiler (the simulation), extracts the bytes and
 * compares them with the bit positions in the ADBMS6830B datasheet.
 */

#include <stdint.h>
#include "bms_datatypes.h"
#include "bms_cmdlist.h"

// Configuration Register Group A (datasheet Table 55):
// CFGAR0 REFON=1 CTH=5, CFGAR1 FLAG_D=0xA5, CFGAR2 SOAKON=1 OWRNG=0 OWA=3,
// CFGAR3 GPO8..1=0x5A, CFGAR4 GPO10..9=0b10, CFGAR5 SNAP_ST=0 MUTE_ST=1 COMM_BK=0 FC=6
const ad68_cfa_t probe_cfa = {
    .cth = 5, .refon = 1, .flag_d = 0xA5, .owa = 3, .owrng = 0, .soakon = 1,
    .gpo1to8 = 0x5A, .gpo9to10 = 0x2, .fc = 6, .comm_bk = 0, .mute_st = 1, .snap_st = 0,
};

// Configuration Register Group B (Table 56): VUV=0xABC, VOV=0x123, DTMEN=1 DTRNG=0 DCTO=0x2A, DCC=0x8001
const ad68_cfb_t probe_cfb = {
    .vuv0to7 = 0xBC, .vuv8to11 = 0xA, .vov0to3 = 0x3, .vov4to11 = 0x12,
    .dcto = 0x2A, .dtrng = 0, .dtmen = 1, .dcc = 0x8001,
};

// PWM Register Group A/B: PWMn = n & 0xF
const ad68_pwma_t probe_pwma = {
    .pwm1 = 1, .pwm2 = 2, .pwm3 = 3, .pwm4 = 4, .pwm5 = 5, .pwm6 = 6,
    .pwm7 = 7, .pwm8 = 8, .pwm9 = 9, .pwm10 = 10, .pwm11 = 11, .pwm12 = 12,
};
const ad68_pwmb_t probe_pwmb = { .pwm13 = 13, .pwm14 = 14, .pwm15 = 15, .pwm16 = 1, .reserved = {0} };

// Commands (Tables 18 and 23), built from the firmware's defaults with the fields set
const ADCV_t probe_adcv = { .RD = 1, .RES2 = 0b01, .X = 0, .OW = 0b10, .RSTF = 1, .RES0 = 0, .DCP = 1, .RES1 = 0b11, .CONT = 1 };
const ADSV_t probe_adsv = { .RES2 = 0b001, .X = 0, .OW = 0b01, .RES0 = 0b10, .DCP = 0, .RES1 = 0b11, .CONT = 1 };
const ADAX_t probe_adax = { .OW = 1, .RES0 = 0b10, .X = 0, .CH = 0b1010, .RES1 = 0b01, .CH4 = 1, .PUP = 1 };
