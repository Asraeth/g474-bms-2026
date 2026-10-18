/*
 * bms_utility.c
 *
 * isoSPI transfers to the ADBMS6830B chain, see bms_utility.h. The PEC tables and both PEC functions are
 * Analog Devices' reference code (datasheet "Packet Error Code").
 */

#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "main.h"
#include <string.h>


static SPI_HandleTypeDef *const hspi = &hspi1;

// CRC15 table for the command PEC: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
static const uint16_t Crc15Table[256] =
{
    0x0000,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
    0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

// CRC10 table for the data PEC: x^10 + x^7 + x^3 + x^2 + x + 1
static const uint16_t crc10Table[256] =
{
    0x000, 0x08f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0x0f7, 0x078, 0x1e9, 0x166, 0x2cb, 0x244, 0x3d5, 0x35a,
    0x1ee, 0x161, 0x0f0, 0x07f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119, 0x196, 0x007, 0x088, 0x325, 0x3aa, 0x23b, 0x2b4,
    0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f, 0x0fe, 0x071, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x009, 0x086,
    0x232, 0x2bd, 0x32c, 0x3a3, 0x00e, 0x081, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0x0f9, 0x076, 0x1e7, 0x168,
    0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x015, 0x09a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc, 0x173, 0x0e2, 0x06d,
    0x2d9, 0x256, 0x3c7, 0x348, 0x0e5, 0x06a, 0x1fb, 0x174, 0x22e, 0x2a1, 0x330, 0x3bf, 0x012, 0x09d, 0x10c, 0x183,
    0x0eb, 0x064, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9, 0x346, 0x01c, 0x093, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1,
    0x105, 0x18a, 0x01b, 0x094, 0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0x0ec, 0x063, 0x3ce, 0x341, 0x2d0, 0x25f,
    0x2e1, 0x26e, 0x3ff, 0x370, 0x0dd, 0x052, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x02a, 0x0a5, 0x134, 0x1bb,
    0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x02d, 0x0a2, 0x3f8, 0x377, 0x2e6, 0x269, 0x1c4, 0x14b, 0x0da, 0x055,
    0x13d, 0x1b2, 0x023, 0x0ac, 0x301, 0x38e, 0x21f, 0x290, 0x1ca, 0x145, 0x0d4, 0x05b, 0x3f6, 0x379, 0x2e8, 0x267,
    0x0d3, 0x05c, 0x1cd, 0x142, 0x2ef, 0x260, 0x3f1, 0x37e, 0x024, 0x0ab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389,
    0x1d6, 0x159, 0x0c8, 0x047, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x03f, 0x0b0, 0x31d, 0x392, 0x203, 0x28c,
    0x038, 0x0b7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0x0cf, 0x040, 0x1d1, 0x15e, 0x2f3, 0x27c, 0x3ed, 0x362,
    0x20a, 0x285, 0x314, 0x39b, 0x036, 0x0b9, 0x128, 0x1a7, 0x2fd, 0x272, 0x3e3, 0x36c, 0x0c1, 0x04e, 0x1df, 0x150,
    0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157, 0x0c6, 0x049, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x031, 0x0be
};


static uint16_t bms_calcPec15(const uint8_t *data, uint8_t len)
{
    uint16_t remainder, addr;
    remainder = 16; /* initialize the PEC */
    for (uint8_t i = 0; i<len; i++) /* loops for each byte in data array */
    {
        addr = (((remainder>>7)^data[i])&0xff);/* calculate PEC table address */
        remainder = ((remainder<<8)^Crc15Table[addr]);
    }
    return(remainder*2);/* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}


// commandCounter: NULL for data sent to the devices; for data read back, the counter the device sent with it
static uint16_t bms_calcPec10(const uint8_t *pDataBuf, int nLength, const uint8_t *commandCounter)
{
    uint16_t nRemainder = 16u; /* PEC_SEED */
    /* x10 + x7 + x3 + x2 + x + 1 <- the CRC10 polynomial 100 1000 1111 */
    uint16_t nPolynomial = 0x8Fu;
    uint8_t nByteIndex, nBitIndex;
    uint16_t nTableAddr;

    for (nByteIndex = 0u; nByteIndex < nLength; ++nByteIndex)
    {
        /* calculate PEC table address */
        nTableAddr = (uint16_t)(((uint16_t)(nRemainder >> 2) ^ (uint8_t)pDataBuf[nByteIndex]) & (uint8_t)0xff);
        nRemainder = (uint16_t)(((uint16_t)(nRemainder << 8)) ^ crc10Table[nTableAddr]);
    }
    /* If array is from received buffer add command counter to crc calculation */
    if (commandCounter != NULL)
    {
        nRemainder ^= (uint16_t)(*commandCounter << 4u);
    }
    /* Perform modulo-2 division, a bit at a time */
    for (nBitIndex = 6u; nBitIndex > 0u; --nBitIndex)
    {
        /* Try to divide the current data bit */
        if ((nRemainder & 0x200u) > 0u)
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
            nRemainder = (uint16_t)(nRemainder ^ nPolynomial);
        }
        else
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
        }
    }
    return ((uint16_t)(nRemainder & 0x3FFu));
}


// Command code + PEC15
static bool bms_spiTransmitCmd(const uint8_t cmd[CMD_LEN])
{
    uint16_t pec = bms_calcPec15(cmd, CMD_LEN);
    uint8_t packet[CMDPKT_LEN] = { cmd[0], cmd[1], (uint8_t)(pec >> 8), (uint8_t)pec };

    return HAL_SPI_Transmit(hspi, packet, CMDPKT_LEN, CFG_SPI_TIMEOUT_MS) == HAL_OK;
}


// One register group + PEC10 per device. Each device passes on what it receives and keeps the last group to
// arrive, so the group for the last device in the chain goes first.
static bool bms_spiTransmitData(uint8_t data[TOTAL_IC][DATA_LEN])
{
    uint8_t packets[TOTAL_IC][DATAPKT_LEN];

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        memcpy(packets[ic], data[(TOTAL_IC - 1) - ic], DATA_LEN);

        uint16_t pec = bms_calcPec10(packets[ic], DATA_LEN, NULL);
        packets[ic][DATA_LEN + 0] = (uint8_t)(pec >> 8);
        packets[ic][DATA_LEN + 1] = (uint8_t)pec;
    }

    return HAL_SPI_Transmit(hspi, (uint8_t *)packets, DATAPKT_LEN * TOTAL_IC, CFG_SPI_TIMEOUT_MS) == HAL_OK;
}


// One register group per device, first device first. After its data each device sends its command counter
// (6 bits) and the PEC10 (10 bits).
static bool bms_spiReceiveData(uint8_t rxData[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC])
{
    uint8_t packets[TOTAL_IC][DATAPKT_LEN];

    if (HAL_SPI_Receive(hspi, (uint8_t *)packets, DATAPKT_LEN * TOTAL_IC, CFG_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        memcpy(rxData[ic], packets[ic], DATA_LEN);
        rxCc[ic]  = packets[ic][DATA_LEN] >> 2;
        rxPec[ic] = (uint16_t)(((packets[ic][DATA_LEN] & 0x03) << 8) | packets[ic][DATA_LEN + 1]);
    }
    return true;
}


bool bms_checkRxPec(uint8_t rxData[TOTAL_IC][DATA_LEN], const uint16_t rxPec[TOTAL_IC], const uint8_t rxCc[TOTAL_IC],
                    bool errorIndex[TOTAL_IC])
{
    bool error = false;

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        errorIndex[ic] = bms_calcPec10(rxData[ic], DATA_LEN, &rxCc[ic]) != rxPec[ic];
        error |= errorIndex[ic];
    }
    return error;
}


/* Per-device expected sequence, inherited from the qualified migration. */
static BMS_CommandCounterChain commandCounters;
static BMS_TransportFault transportFault;
BMS_TransportFault bms_getTransportFault(void) { return transportFault; }
void bms_getCounterState(BMS_CommandCounterChain *state) { *state = commandCounters; }
bool bms_checkRxCounter(const uint8_t cc[TOTAL_IC], const bool pecErrors[TOTAL_IC], bool errors[TOTAL_IC])
{
    return BMS_CC_Check(&commandCounters, cc, pecErrors, errors);
}
uint8_t bms_expectedCc(void) { return commandCounters.device[0].expected; }
static bool context(void)
{
    transportFault = bms_spiContextAllowed() ? BMS_TRANSPORT_OK : BMS_TRANSPORT_CONTEXT;
    return transportFault == BMS_TRANSPORT_OK;
}
static bool commandAllowed(const uint8_t *cmd, BMS_CounterCommand required)
{
    if (BMS_CC_Classify(cmd) == required) return true;
    transportFault = BMS_TRANSPORT_COMMAND;
    return false;
}
static void complete(BMS_CounterCommand kind, bool ok)
{
    if (!ok) BMS_CC_Invalidate(&commandCounters);
    else if (kind == BMS_CC_RESET) BMS_CC_Reset(&commandCounters);
    else if (kind == BMS_CC_INCREMENT || kind == BMS_CC_WRITE) BMS_CC_Advance(&commandCounters);
}
bool bms_transmitCmd(const uint8_t cmd[CMD_LEN])
{
    if (!context()) return false;
    BMS_CounterCommand kind = BMS_CC_Classify(cmd);
    if (kind != BMS_CC_INCREMENT && kind != BMS_CC_RESET) { transportFault=BMS_TRANSPORT_COMMAND; return false; }
    bms_csLow();
    bool ok = bms_spiTransmitCmd(cmd);
    bms_csHigh();
    if (!ok) transportFault=BMS_TRANSPORT_TX;
    complete(kind,ok);
    return ok;
}
bool bms_transmitPoll(const uint8_t cmd[CMD_LEN], uint32_t timeoutMs)
{
    if (!context() || !commandAllowed(cmd,BMS_CC_INCREMENT) || timeoutMs==0) return false;
    uint32_t start=HAL_GetTick();
    bms_csLow();
    bool ok=bms_spiTransmitCmd(cmd);
    if (!ok) transportFault=BMS_TRANSPORT_TX;
    complete(BMS_CC_INCREMENT,ok);
    /* Rev.0 p55: status valid after 2*N clocks. Discard ceil(2*N/8)
       bytes; only subsequent bytes can establish complete-stack readiness. */
    uint32_t primeBytes=(2U*TOTAL_IC+7U)/8U;
    while (ok) {
        uint32_t age=HAL_GetTick()-start;
        if (age>=timeoutMs) { transportFault=BMS_TRANSPORT_POLL_TIMEOUT; ok=false; break; }
        uint32_t remaining=timeoutMs-age;
        uint8_t value=0;
        uint32_t budget=remaining<CFG_SPI_TIMEOUT_MS ? remaining : CFG_SPI_TIMEOUT_MS;
        if (HAL_SPI_Receive(hspi,&value,1,budget)!=HAL_OK) { transportFault=BMS_TRANSPORT_RX;ok=false;break; }
        if (primeBytes) { primeBytes--; continue; }
        if (value==0xFF) break;
    }
    bms_csHigh();
    if (!ok) BMS_CC_Invalidate(&commandCounters);
    return ok;
}
bool bms_transmitData(const uint8_t cmd[CMD_LEN], uint8_t txBuffer[TOTAL_IC][DATA_LEN])
{
    if (!context() || !commandAllowed(cmd,BMS_CC_WRITE)) return false;
    bms_csLow();
    bool ok=bms_spiTransmitCmd(cmd) && bms_spiTransmitData(txBuffer);
    bms_csHigh();
    if (!ok) transportFault=BMS_TRANSPORT_TX;
    complete(BMS_CC_WRITE,ok);
    return ok;
}
bool bms_receiveData(const uint8_t cmd[CMD_LEN], uint8_t rxBuffer[TOTAL_IC][DATA_LEN], uint16_t rxPec[TOTAL_IC],uint8_t rxCc[TOTAL_IC])
{
    for(int ic=0;ic<TOTAL_IC;ic++) {
        for(int b=0;b<DATA_LEN;b+=2) {rxBuffer[ic][b]=0;rxBuffer[ic][b+1]=0x80;}
        rxPec[ic]=0;rxCc[ic]=0;
    }
    if (!context() || !commandAllowed(cmd,BMS_CC_READ)) return false;
    bms_csLow();
    bool ok=bms_spiTransmitCmd(cmd);
    if (!ok) transportFault=BMS_TRANSPORT_TX;
    else if (!(ok=bms_spiReceiveData(rxBuffer,rxPec,rxCc))) transportFault=BMS_TRANSPORT_RX;
    bms_csHigh();
    if (!ok) BMS_CC_Invalidate(&commandCounters);
    return ok;
}
