/*
 * bms_datatypes.h
 *
 * ADBMS6830B register groups the firmware writes, as bitfield structs (6 bytes each, byte 0 first on the wire).
 * Bitfield layout is up to the compiler: test/check_layout.sh checks the ARM and the PC compiler against the
 * datasheet (Rev. 0) tables named below.
 */

#pragma once

#include <stdint.h>


// Configuration Register Group A (Table 55)
typedef struct
{
    uint8_t     cth     :3;     // C-ADC vs S-ADC comparison threshold
    uint8_t     unused1 :4;
    uint8_t     refon   :1;     // Reference stays on between conversions (no 5 ms REF-UP before each one)

    uint8_t     flag_d  :8;     // Diagnostic flags to set (latent fault tests)

    uint8_t     unused2 :3;
    uint8_t     owa     :3;     // Open wire soak time
    uint8_t     owrng   :1;
    uint8_t     soakon  :1;

    uint8_t     gpo1to8 :8;     // GPIO pull-downs: 0 = on (output low), 1 = off (input / high)

    uint8_t     gpo9to10:2;
    uint8_t     unused3 :6;

    uint8_t     fc      :3;     // IIR filter corner
    uint8_t     comm_bk :1;
    uint8_t     mute_st :1;     // Read only: discharge muted
    uint8_t     snap_st :1;     // Read only: snapshot taken
    uint8_t     unused4 :2;
} ad68_cfa_t;


// Configuration Register Group B (Table 56)
typedef struct
{
    uint8_t     vuv0to7 :8;     // Undervoltage comparison threshold
    uint8_t     vuv8to11:4;
    uint8_t     vov0to3 :4;     // Overvoltage comparison threshold

    uint8_t     vov4to11:8;

    uint8_t     dcto    :6;     // Discharge timeout, counted from the last WRCFGB: PWM discharge only while it runs (0 = off)
    uint8_t     dtrng   :1;     // DCTO range: 0 = minutes, 1 = hours
    uint8_t     dtmen   :1;     // Discharge timer monitor

    uint16_t    dcc     :16;    // Discharge on regardless of PWM (bit per cell)
} ad68_cfb_t;


// PWM Register Group A (Table 95): 4-bit discharge duty per cell input, n / 15 of the 937 ms period
typedef struct
{
    uint8_t     pwm1    :4;
    uint8_t     pwm2    :4;

    uint8_t     pwm3    :4;
    uint8_t     pwm4    :4;

    uint8_t     pwm5    :4;
    uint8_t     pwm6    :4;

    uint8_t     pwm7    :4;
    uint8_t     pwm8    :4;

    uint8_t     pwm9    :4;
    uint8_t     pwm10   :4;

    uint8_t     pwm11   :4;
    uint8_t     pwm12   :4;
} ad68_pwma_t;


// PWM Register Group B (Table 96)
typedef struct
{
    uint8_t     pwm13   :4;
    uint8_t     pwm14   :4;

    uint8_t     pwm15   :4;
    uint8_t     pwm16   :4;

    uint8_t     reserved[4];
} ad68_pwmb_t;
