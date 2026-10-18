/*
 * sim_adbms.c
 *
 * ADBMS6830B daisy chain behind the ADBMS6822, modelled from the ADBMS6830B datasheet (Rev. 0):
 *  - command decoding with PEC15, register reads with PEC10 (incl. command counter), writes in
 *    daisy-chain order (first data block goes to the last device)
 *  - isoSPI wake-up: one CS pulse per device, tREADY 10 us when the core is awake, tWAKE 500 us
 *    from sleep, port idles after tIDLE 4.3 ms without activity, core sleeps after 1.8 s without
 *    a valid command (registers reset)
 *  - S-ADC single shots (8 ms) incl. open wire switches, AUX conversions, polling
 *  - fault injection: broken chain, corrupted reads, SPI errors, open sense wires
 *
 * The CRCs are written bit by bit from the datasheet definition, independently of the firmware's
 * table-driven versions, so a mismatch shows up as PEC errors.
 */

#include "sim.h"
#include "spi.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_DEV         32
#define T_READY_US      10
#define T_WAKE_US       500
#define T_IDLE_US       4300
#define T_SLEEP_US      1800000
// Conversion times: typical by default, datasheet slowest with SIM_PESSIMISTIC=1 (S-ADC 110 Hz,
// AUX 0.9 kHz per channel) plus extra SPI driver overhead per call and per byte
static uint64_t T_SADC_US = 8000;
static uint64_t T_AUX_US  = 18000;
static uint64_t spi_call_overhead_us = 0;
static uint64_t spi_byte_gap_us = 0;
#define NEVER           UINT64_MAX

#define SPI_APB_CLOCK_HZ 128000000ULL

float sim_adbms_ow_open_ratio = 0.3f;

typedef struct
{
    bool     core_asleep;
    uint64_t port_ready_at;         // NEVER if the port is idle
    uint64_t last_activity;
    uint64_t last_valid_cmd;

    uint8_t  cfga[6], cfgb[6], pwma[6], pwmb[6];
    uint8_t  sid[6];
    uint8_t  cc;
    bool     cadc_cont;             // C-ADC converting continuously (ADCV with CONT = 1, until reset or sleep)

    float    cell_v[16];
    bool     open_wire[16];
    float    gpio_v[10];
    float    die_temp;

    int16_t  sv[16];
    int16_t  aux[12];               // G1..G10, VMV, VPV
    int16_t  vref2, itmp;
    uint64_t sadc_done_at, aux_done_at;
    uint64_t last_sadc_start[4];    // By OW mode

    int      corrupt_reads;
} sim_dev;

static sim_dev dev[MAX_DEV];
static int     lose_conversion[MAX_DEV];  // Normal S-ADC conversion commands the device will miss
static int     num_dev = 0;
static int     broken_after = -1;
static int     unpowered = -1;      // Device without supply: it and everything after it unreachable
static bool    spi_error = false;
static int     unknown_commands = 0;
static int     lost_transactions = 0;
static uint32_t noise_state = 12345;

typedef enum { T_IDLE, T_CMD, T_READ, T_WRITE, T_POLL_S, T_POLL_AUX, T_IGNORE } txn_state;

static txn_state state = T_IDLE;
static bool      cs_low = false;
static int       reachable = 0;     // Devices awake at the start of the transaction
static uint8_t   cmd[4];
static int       cmd_len = 0;
static uint16_t  cmd_code = 0;
static uint8_t   rx_buf[MAX_DEV * 8];
static int       rx_len = 0, rx_pos = 0;
static uint8_t   wr_buf[MAX_DEV * 8];
static int       wr_len = 0;
static bool      slept_this_txn = false;   // SRST: the trailing CS edge doesn't wake the chain again


/* ---- CRCs (ADBMS6830B datasheet, bitwise) ---------------------------------------------------- */

static uint16_t pec15(const uint8_t* data, int len)
{
    uint16_t rem = 16;
    for (int i = 0; i < len; i++)
    {
        for (int bit = 7; bit >= 0; bit--)
        {
            int in = ((data[i] >> bit) & 1) ^ ((rem >> 14) & 1);
            rem = (uint16_t)((rem << 1) & 0x7FFF);
            if (in) rem ^= 0x4599;
        }
    }
    return (uint16_t)(rem << 1);
}

static uint16_t pec10(const uint8_t* data, int len, int cc)
{
    uint16_t rem = 16;
    for (int i = 0; i < len; i++)
    {
        for (int bit = 7; bit >= 0; bit--)
        {
            int in = ((data[i] >> bit) & 1) ^ ((rem >> 9) & 1);
            rem = (uint16_t)((rem << 1) & 0x3FF);
            if (in) rem ^= 0x08F;
        }
    }
    if (cc >= 0)
    {
        for (int bit = 5; bit >= 0; bit--)
        {
            int in = ((cc >> bit) & 1) ^ ((rem >> 9) & 1);
            rem = (uint16_t)((rem << 1) & 0x3FF);
            if (in) rem ^= 0x08F;
        }
    }
    return rem;
}


/* ---- Device state ---------------------------------------------------------------------------- */

static int16_t encode(float volts)
{
    float raw = (volts - 1.5f) / 0.00015f;
    if (raw > 32767.0f) raw = 32767.0f;
    if (raw < -32768.0f) raw = -32768.0f;
    return (int16_t)lroundf(raw);
}

static float noise(void)
{
    noise_state = noise_state * 1103515245U + 12345U;
    return ((float)((noise_state >> 16) & 0x7FFF) / 32767.0f - 0.5f) * 0.0006f;   // +-0.3 mV
}

static void reset_registers(sim_dev* d)
{
    static const uint8_t cfga_default[6] = { 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00 };
    static const uint8_t cfgb_default[6] = { 0x00, 0xF8, 0x7F, 0x00, 0x00, 0x00 };
    memcpy(d->cfga, cfga_default, 6);
    memcpy(d->cfgb, cfgb_default, 6);
    memset(d->pwma, 0, 6);
    memset(d->pwmb, 0, 6);
    for (int i = 0; i < 16; i++) d->sv[i] = (int16_t)0x8000;
    for (int i = 0; i < 12; i++) d->aux[i] = (int16_t)0x8000;
    d->vref2 = (int16_t)0x8000;
    d->itmp = 0x7FFF;
    d->cc = 0;
    d->cadc_cont = false;
    d->sadc_done_at = 0;
    d->aux_done_at = 0;
}

static void go_to_sleep(sim_dev* d)
{
    d->core_asleep = true;
    d->port_ready_at = NEVER;
    reset_registers(d);
}

static bool port_awake(const sim_dev* d, uint64_t now)
{
    if (d->port_ready_at == NEVER || d->port_ready_at > now) return false;
    uint64_t since = d->last_activity > d->port_ready_at ? d->last_activity : d->port_ready_at;
    return now - since <= T_IDLE_US;
}

// Devices the isoSPI traffic reaches: all of them, up to the break, or none (SIM_CHAIN_BROKEN_AT_MASTER)
static int chain_limit(void)
{
    if (broken_after == SIM_CHAIN_BROKEN_AT_MASTER) return 0;
    int limit = (broken_after >= 0 && broken_after + 1 < num_dev) ? broken_after + 1 : num_dev;
    return (unpowered >= 0 && unpowered < limit) ? unpowered : limit;
}

static void update_sleep(uint64_t now)
{
    for (int i = 0; i < num_dev; i++)
    {
        if (!dev[i].core_asleep && now - dev[i].last_valid_cmd > T_SLEEP_US)
        {
            go_to_sleep(&dev[i]);
        }
    }
}

// isoSPI traffic keeps every awake device (up to the first idle one) awake
static void activity(uint64_t now)
{
    for (int i = 0; i < chain_limit(); i++)
    {
        if (!port_awake(&dev[i], now)) break;
        dev[i].last_activity = now;
    }
}

static int count_reachable(uint64_t now)
{
    int n = 0;
    for (int i = 0; i < chain_limit(); i++)
    {
        if (!port_awake(&dev[i], now)) break;
        n++;
    }
    return n;
}

// A CS low/high pair is a long isoSPI pulse pair: it wakes the first device that isn't awake
static void wake_pulse(uint64_t now)
{
    for (int i = 0; i < chain_limit(); i++)
    {
        sim_dev* d = &dev[i];
        if (port_awake(d, now)) continue;
        if (d->port_ready_at != NEVER && d->port_ready_at > now) return;    // Still waking, pulse lost

        d->port_ready_at = now + (d->core_asleep ? T_WAKE_US : T_READY_US);
        d->last_activity = d->port_ready_at;
        if (d->core_asleep)
        {
            d->core_asleep = false;
            d->last_valid_cmd = now;
        }
        return;
    }
}


/* ---- Commands --------------------------------------------------------------------------------- */

typedef enum
{
    C_UNKNOWN, C_MUTE, C_UNMUTE, C_WRCFGA, C_WRCFGB, C_WRPWMA, C_WRPWMB, C_RDCFGA, C_RDCFGB, C_RDPWMA, C_RDPWMB, C_RDSID,
    C_RDSV, C_RDAUX, C_RDSTATA, C_SRST, C_PLSADC, C_PLAUX1, C_PLCADC, C_ADCV, C_ADSV, C_ADAX, C_RSTCC
} cmd_kind;

static int read_group = 0;

static cmd_kind decode(uint16_t cc)
{
    switch (cc)
    {
    case 0x001: return C_WRCFGA;
    case 0x024: return C_WRCFGB;
    case 0x020: return C_WRPWMA;
    case 0x021: return C_WRPWMB;
    case 0x002: return C_RDCFGA;
    case 0x026: return C_RDCFGB;
    case 0x022: return C_RDPWMA;
    case 0x023: return C_RDPWMB;
    case 0x02C: return C_RDSID;
    case 0x003: read_group = 0; return C_RDSV;
    case 0x005: read_group = 1; return C_RDSV;
    case 0x007: read_group = 2; return C_RDSV;
    case 0x00D: read_group = 3; return C_RDSV;
    case 0x00E: read_group = 4; return C_RDSV;
    case 0x00F: read_group = 5; return C_RDSV;
    case 0x019: read_group = 0; return C_RDAUX;
    case 0x01A: read_group = 1; return C_RDAUX;
    case 0x01B: read_group = 2; return C_RDAUX;
    case 0x01F: read_group = 3; return C_RDAUX;
    case 0x030: return C_RDSTATA;
    case 0x027: return C_SRST;
    case 0x028: return C_MUTE;
    case 0x029: return C_UNMUTE;
    case 0x02E: return C_RSTCC;
    case 0x71D: return C_PLSADC;
    case 0x71E: return C_PLAUX1;
    case 0x71C: return C_PLCADC;
    default: break;
    }
    if ((cc & 0x668) == 0x260) return C_ADCV;
    if ((cc & 0x76C) == 0x168) return C_ADSV;
    if ((cc & 0x630) == 0x410) return C_ADAX;
    return C_UNKNOWN;
}

static void inc_cc(sim_dev* d)
{
    d->cc = (uint8_t)(d->cc >= 63 ? 1 : d->cc + 1);
}

static void group_data(sim_dev* d, cmd_kind kind, uint8_t out[6])
{
    memset(out, 0xFF, 6);
    switch (kind)
    {
    case C_RDCFGA: memcpy(out, d->cfga, 6); break;
    case C_RDCFGB: memcpy(out, d->cfgb, 6); break;
    case C_RDPWMA: memcpy(out, d->pwma, 6); break;
    case C_RDPWMB: memcpy(out, d->pwmb, 6); break;
    case C_RDSID:  memcpy(out, d->sid, 6);  break;
    case C_RDSV:
        for (int k = 0; k < 3; k++)
        {
            int ch = read_group * 3 + k;
            if (ch < 16)
            {
                out[k * 2]     = (uint8_t)(d->sv[ch] & 0xFF);
                out[k * 2 + 1] = (uint8_t)((d->sv[ch] >> 8) & 0xFF);
            }
        }
        break;
    case C_RDAUX:
        for (int k = 0; k < 3; k++)
        {
            int idx = read_group * 3 + k;
            int16_t v = d->aux[idx];
            out[k * 2]     = (uint8_t)(v & 0xFF);
            out[k * 2 + 1] = (uint8_t)((v >> 8) & 0xFF);
        }
        break;
    case C_RDSTATA:
        out[0] = (uint8_t)(d->vref2 & 0xFF);
        out[1] = (uint8_t)((d->vref2 >> 8) & 0xFF);
        out[2] = (uint8_t)(d->itmp & 0xFF);
        out[3] = (uint8_t)((d->itmp >> 8) & 0xFF);
        out[4] = 0;
        out[5] = 0;
        break;
    default:
        break;
    }
}

static void start_sadc(sim_dev* d, int ow, uint64_t now)
{
    for (int ch = 0; ch < 16; ch++)
    {
        bool even = ((ch + 1) % 2) == 0;
        bool switch_on = (ow == 3) || (ow == 1 && even) || (ow == 2 && !even);
        float v = d->cell_v[ch];
        float reading;
        if (d->open_wire[ch])
        {
            reading = switch_on ? v * sim_adbms_ow_open_ratio : v;     // Floating input looks plausible
        }
        else
        {
            reading = switch_on ? v * 0.90f : v;
        }
        d->sv[ch] = encode(reading + noise());
    }
    d->sadc_done_at = now + T_SADC_US;
    d->last_sadc_start[ow & 3] = now;
}

static void start_aux(sim_dev* d, uint64_t now)
{
    float vpv = 0;
    for (int ch = 0; ch < 16; ch++) vpv += d->cell_v[ch];
    for (int g = 0; g < 10; g++) d->aux[g] = encode(d->gpio_v[g] + noise());
    d->aux[10] = encode(0.0f);                  // VMV
    d->aux[11] = encode(vpv / 25.0f);           // VPV
    d->vref2 = encode(3.0f);
    d->itmp = encode((d->die_temp + 273.0f) * 0.0075f);
    d->aux_done_at = now + T_AUX_US;
}

static void execute(cmd_kind kind, uint64_t now)
{
    for (int i = 0; i < reachable; i++)
    {
        sim_dev* d = &dev[i];
        d->last_valid_cmd = now;
        switch (kind)
        {
        case C_ADCV: inc_cc(d); d->cadc_cont = (cmd_code & 0x080) != 0; break;     // CONT: bit 7
        case C_ADSV:
            if ((cmd_code & 0x3) == 0 && lose_conversion[i] > 0)
            {
                lose_conversion[i]--;           // Lost on the way (bad contact): not executed, not counted
                break;
            }
            inc_cc(d);
            start_sadc(d, cmd_code & 0x3, now);
            break;
        case C_ADAX: inc_cc(d); start_aux(d, now); break;
        default: break;
        }
    }
}

static void prepare_read(cmd_kind kind)
{
    rx_len = num_dev * 8;
    rx_pos = 0;
    memset(rx_buf, 0xFF, sizeof(rx_buf));       // Unreachable devices: MISO stays high

    for (int i = 0; i < reachable; i++)
    {
        sim_dev* d = &dev[i];
        uint8_t* out = &rx_buf[i * 8];
        group_data(d, kind, out);
        uint16_t pec = pec10(out, 6, d->cc);
        out[6] = (uint8_t)((d->cc << 2) | ((pec >> 8) & 0x3));
        out[7] = (uint8_t)(pec & 0xFF);
        if (d->corrupt_reads > 0)
        {
            d->corrupt_reads--;
            out[0] ^= 0x01;
        }
    }
}

static uint32_t pollClocks;
static void command_received(uint64_t now)
{
    pollClocks=0;
    uint16_t pec = (uint16_t)((cmd[2] << 8) | cmd[3]);
    if (pec15(cmd, 2) != pec)
    {
        sim_fail("isoSPI command 0x%02X%02X sent with a bad PEC", cmd[0], cmd[1]);
        state = T_IGNORE;
        return;
    }

    cmd_code = (uint16_t)(((cmd[0] & 0x07) << 8) | cmd[1]);
    cmd_kind kind = decode(cmd_code);

    for (int i = 0; i < reachable; i++) dev[i].last_valid_cmd = now;

    switch (kind)
    {
    case C_UNKNOWN:
        unknown_commands++;
        sim_fail("unknown ADBMS6830B command 0x%03X", cmd_code);
        state = T_IGNORE;
        break;
    case C_WRCFGA: case C_WRCFGB: case C_WRPWMA: case C_WRPWMB:
        wr_len = 0;
        state = T_WRITE;
        break;
    // Polls count as commands (datasheet Table 50, INC); RSTCC sets the counter to 0
    case C_PLSADC: for (int i = 0; i < reachable; i++) inc_cc(&dev[i]); state = T_POLL_S; break;
    case C_PLAUX1: for (int i = 0; i < reachable; i++) inc_cc(&dev[i]); state = T_POLL_AUX; break;
    case C_PLCADC: for (int i = 0; i < reachable; i++) inc_cc(&dev[i]); state = T_IGNORE; break;
    case C_MUTE: case C_UNMUTE: for(int i=0;i<reachable;i++) inc_cc(&dev[i]); state=T_IGNORE; break;
    case C_RSTCC:  for (int i = 0; i < reachable; i++) dev[i].cc = 0; state = T_IGNORE; break;
    case C_SRST:
    case C_ADCV: case C_ADSV: case C_ADAX:
        execute(kind, now);
        state = T_IGNORE;
        if (kind == C_SRST)
        {
            for (int i = 0; i < reachable; i++) go_to_sleep(&dev[i]);
            slept_this_txn = true;
        }
        break;
    default:
        prepare_read(kind);
        state = T_READ;
        break;
    }
}

static void apply_write(void)
{
    cmd_kind kind = decode(cmd_code);
    /* An injected HAL timeout after the write opcode can leave no payload.
     * Real devices discard incomplete data/PEC without changing registers/CC. */
    if(spi_error && wr_len==0) {lost_transactions++;return;}
    for (int i = 0; i < reachable; i++)
    {
        int offset = (num_dev - 1 - i) * 8;     // First block sent is for the last device
        if (offset + 8 > wr_len)
        {
            sim_fail("write 0x%03X too short for device %d (%d bytes)", cmd_code, i, wr_len);
            continue;
        }
        const uint8_t* blk = &wr_buf[offset];
        uint16_t pec = (uint16_t)(((blk[6] & 0x3) << 8) | blk[7]);
        if (pec10(blk, 6, 0) != pec)            // Datasheet Table 42: CC bits sent as 0 and included
        {
            sim_fail("write 0x%03X to device %d has a bad data PEC", cmd_code, i);
            continue;
        }
        sim_dev* d = &dev[i];
        uint8_t* reg = kind == C_WRCFGA ? d->cfga : kind == C_WRCFGB ? d->cfgb : kind == C_WRPWMA ? d->pwma : d->pwmb;
        memcpy(reg, blk, 6);
        inc_cc(d);
    }
}


/* ---- Chip select and SPI ----------------------------------------------------------------------- */

static void cs_hook(GPIO_TypeDef* port, uint16_t pin, bool level)
{
    if (port != CFG_PIN_ISOSPI_CS_PORT || pin != CFG_PIN_ISOSPI_CS_PIN) return;
    uint64_t now = sim_now_us();
    update_sleep(now);

    if (!level)
    {
        cs_low = true;
        reachable = count_reachable(now);
        activity(now);
        state = T_CMD;
        cmd_len = 0;
    }
    else
    {
        if (state == T_WRITE) apply_write();
        if (state == T_CMD && cmd_len > 0 && cmd_len < 4) lost_transactions++;
        cs_low = false;
        state = T_IDLE;
        activity(now);
        if (!slept_this_txn) wake_pulse(now);
        slept_this_txn = false;
    }
}

static uint64_t spi_byte_us(void)
{
    uint32_t br = (hspi1.Init.BaudRatePrescaler >> 3) & 0x7;
    uint64_t clock = SPI_APB_CLOCK_HZ / (2ULL << br);
    return 8ULL * 1000000ULL / clock + spi_byte_gap_us;
}

static void mosi(uint8_t byte)
{
    uint64_t now = sim_now_us();
    activity(now);
    switch (state)
    {
    case T_CMD:
        cmd[cmd_len++] = byte;
        if (cmd_len == 4) command_received(now);
        break;
    case T_WRITE:
        if (wr_len < (int)sizeof(wr_buf)) wr_buf[wr_len++] = byte;
        break;
    default:
        break;
    }
}

static uint8_t miso(void)
{
    uint64_t now = sim_now_us();
    activity(now);
    switch (state)
    {
    case T_READ:
        return rx_pos < rx_len ? rx_buf[rx_pos++] : 0xFF;
    case T_POLL_S:
    case T_POLL_AUX:
        /* Datasheet 2*N priming clocks may read high even while the ADC is busy. */
        if(pollClocks<(uint32_t)(2*reachable)) {pollClocks+=8;return 0xFF;}
        if (reachable == 0) return 0xFF;
        for (int i = 0; i < reachable; i++)
        {
            uint64_t done = state == T_POLL_S ? dev[i].sadc_done_at : dev[i].aux_done_at;
            if (now < done) return 0x00;
        }
        return 0xFF;
    default:
        return 0xFF;
    }
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* h)
{
    HAL_SPI_MspInit(h);
    h->State = HAL_SPI_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* h, const uint8_t* data, uint16_t size, uint32_t timeout)
{
    (void)h;
    if (spi_error)
    {
        sim_advance_us((uint64_t)timeout * 1000ULL);
        return HAL_TIMEOUT;
    }
    sim_advance_us(spi_call_overhead_us);
    for (int i = 0; i < size; i++)
    {
        if (cs_low) mosi(data[i]);
        sim_advance_us(spi_byte_us());
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* h, uint8_t* data, uint16_t size, uint32_t timeout)
{
    (void)h;
    if (spi_error)
    {
        sim_advance_us((uint64_t)timeout * 1000ULL);
        return HAL_TIMEOUT;
    }
    sim_advance_us(spi_call_overhead_us);
    for (int i = 0; i < size; i++)
    {
        data[i] = cs_low ? miso() : 0xFF;
        sim_advance_us(spi_byte_us());
    }
    return HAL_OK;
}


/* ---- Test controls ------------------------------------------------------------------------------ */

void sim_adbms_init(int devices)
{
    memset(lose_conversion, 0, sizeof(lose_conversion));
    const char* pessimistic = getenv("SIM_PESSIMISTIC");
    if (pessimistic && pessimistic[0] == '1')
    {
        T_SADC_US = 9100;
        T_AUX_US = 20000;
        spi_call_overhead_us = 15;
        spi_byte_gap_us = 1;
        sim_note("pessimistic timing: S-ADC 9.1 ms, AUX 20 ms, +15 us per SPI call, +1 us per byte");
    }
    num_dev = devices;
    for (int i = 0; i < num_dev; i++)
    {
        sim_dev* d = &dev[i];
        memset(d, 0, sizeof(*d));
        go_to_sleep(d);
        d->last_valid_cmd = 0;
        for (int k = 0; k < 6; k++) d->sid[k] = (uint8_t)(0xA0 + i + k);
        d->die_temp = 30.0f;
    }
    sim_gpio_add_write_hook(cs_hook);
}

void  sim_adbms_set_cell(int d, int ch, float v)       { dev[d].cell_v[ch] = v; }
float sim_adbms_get_cell(int d, int ch)                { return dev[d].cell_v[ch]; }
void  sim_adbms_set_gpio(int d, int gpio, float v)     { dev[d].gpio_v[gpio - 1] = v; }
void  sim_adbms_set_die_temp(int d, float c)           { dev[d].die_temp = c; }
void  sim_adbms_set_open_wire(int d, int ch, bool o)   { dev[d].open_wire[ch] = o; }
void  sim_adbms_set_broken_after(int d)                { broken_after = d; }
void  sim_adbms_lose_next_conversion(int d)            { lose_conversion[d]++; }

// Supply lost: the device forgets everything and breaks the chain; back on, it starts asleep in its reset state
void sim_adbms_set_powered(int d, bool powered)
{
    if (!powered)
    {
        unpowered = d;
    }
    else if (unpowered == d)
    {
        unpowered = -1;
    }
    go_to_sleep(&dev[d]);
}
void  sim_adbms_corrupt_reads(int d, int count)        { dev[d].corrupt_reads = count; }
void  sim_adbms_set_spi_error(bool e)                  { spi_error = e; }
int   sim_adbms_unknown_commands(void)                 { return unknown_commands; }
int   sim_adbms_lost_transactions(void)                { return lost_transactions; }
uint64_t sim_adbms_last_sadc_us(int d, int ow)         { return dev[d].last_sadc_start[ow & 3]; }
bool  sim_adbms_refon(int d)                           { return (dev[d].cfga[0] & 0x80) != 0; }
bool  sim_adbms_cadc_running(int d)                    { return dev[d].cadc_cont; }
uint8_t sim_adbms_dcto(int d)                          { return dev[d].cfgb[3] & 0x3F; }

uint8_t sim_adbms_pwm(int d, int ch)
{
    const uint8_t* reg = ch < 12 ? dev[d].pwma : dev[d].pwmb;
    int idx = ch < 12 ? ch : ch - 12;
    uint8_t byte = reg[idx / 2];
    return (idx % 2 == 0) ? (byte & 0x0F) : (byte >> 4);
}

uint16_t sim_adbms_gpo(int d)
{
    return (uint16_t)(dev[d].cfga[3] | ((dev[d].cfga[4] & 0x3) << 8));
}
