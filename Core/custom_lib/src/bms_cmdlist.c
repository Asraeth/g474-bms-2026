/*
 * bms_cmdlist.c
 *
 * ADBMS6830B command codes (datasheet Rev. 0, Table 50), see bms_cmdlist.h.
 */

#include "bms_cmdlist.h"


/* ---- Register groups ------------------------------------------------------------------------- */

const uint8_t WRCFGA[2]  = { 0x00, 0x01 };
const uint8_t WRCFGB[2]  = { 0x00, 0x24 };
const uint8_t RDCFGA[2]  = { 0x00, 0x02 };
const uint8_t RDCFGB[2]  = { 0x00, 0x26 };

const uint8_t WRPWMA[2]  = { 0x00, 0x20 };
const uint8_t WRPWMB[2]  = { 0x00, 0x21 };
const uint8_t RDPWMA[2]  = { 0x00, 0x22 };
const uint8_t RDPWMB[2]  = { 0x00, 0x23 };

const uint8_t RDSVA[2]   = { 0x00, 0x03 };
const uint8_t RDSVB[2]   = { 0x00, 0x05 };
const uint8_t RDSVC[2]   = { 0x00, 0x07 };
const uint8_t RDSVD[2]   = { 0x00, 0x0D };
const uint8_t RDSVE[2]   = { 0x00, 0x0E };
const uint8_t RDSVF[2]   = { 0x00, 0x0F };

const uint8_t RDAUXA[2]  = { 0x00, 0x19 };
const uint8_t RDAUXB[2]  = { 0x00, 0x1A };
const uint8_t RDAUXC[2]  = { 0x00, 0x1B };
const uint8_t RDAUXD[2]  = { 0x00, 0x1F };
const uint8_t RDSTATA[2] = { 0x00, 0x30 };


/* ---- Polls and control ----------------------------------------------------------------------- */

const uint8_t PLSADC[2]  = { 0x07, 0x1D };
const uint8_t PLAUX1[2]  = { 0x07, 0x1E };

const uint8_t RDSID[2]   = { 0x00, 0x2C };
const uint8_t RSTCC[2]   = { 0x00, 0x2E };
const uint8_t SRST[2]    = { 0x00, 0x27 };


/* ---- Conversion commands --------------------------------------------------------------------- */

// The RES fields are the fixed bits of each command code

const ADCV_t ADCV_default =
{
    .RD   = 0,
    .RES2 = 0b01,
    .X    = 0b00000,

    .OW   = 0b00,
    .RSTF = 0,
    .RES0 = 0b0,
    .DCP  = 0,
    .RES1 = 0b11,
    .CONT = 0,
};

const ADSV_t ADSV_default =
{
    .RES2 = 0b001,
    .X    = 0b00000,

    .OW   = 0b00,
    .RES0 = 0b10,
    .DCP  = 0,
    .RES1 = 0b11,
    .CONT = 0,
};

const ADAX_t ADAX_default =
{
    .OW   = 0b0,
    .RES0 = 0b10,
    .X    = 0b00000,

    .CH   = 0b0000,
    .RES1 = 0b01,
    .CH4  = 0b0,
    .PUP  = 0b0,
};

const uint8_t MUTE[2]={0,0x28};
const uint8_t UNMUTE[2]={0,0x29};
