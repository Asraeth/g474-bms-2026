/*
 * sim_ivt.c
 *
 * Isabellenhuette IVT-S-500-U3-I-CAN2-12/24 on CAN2, from the IVT-S datasheet v1.01:
 *  - result frames 0x521..0x528 (I mA, U1-U3 mV, T, W, As, Wh): DB0 mux, DB1 counter | status << 4,
 *    DB2..5 signed 32-bit big-endian, DLC 6
 *  - default config: as the team's unit came (bench, 2026-10-12), all eight results cyclic: I 20 ms, U1..U3
 *    60 ms, T 100 ms, W/As/Wh 30 ms. (Chapter 8.3 says T/W/As/Wh are disabled and the "Default parameters"
 *    table that U1 is; U1 disabled is still modelled.)
 *  - Set Config Result with a time of 0x0000 is ignored altogether, mode included (seen on the unit)
 *  - commands sent during the 400 ms start-up are lost; then the alive message (0xBF on 0x511)
 *  - commands on 0x411, responses on 0x511; results can only be configured in stop mode
 *  - 400 ms start-up after power-up; the energy counter restarts from a value stored up to 15 min
 *    earlier (log data), modelled as a step back
 *  - status bits: bit1 when this result is out of its nominal range (500 A for the 500 variant),
 *    bit2 when any result has a measurement error (the unconnected U3 input), bit3 (system error)
 *    latched until the next power-up once one is logged
 *  - RESTART (0x3F): it restarts like a power-up but logs the restart as a watchdog reset, a system
 *    error (bench, 2026-10-12)
 *  - system error counters (lifetime, kept over power cycles); Get system errors (0x41: bitmask or one
 *    counter) and Get serial number (0x7B) are only answered in stop mode (bench); Reset Error- and Logdata
 *    (0x30 0x01 0x00 + serial, stop mode) clears them and the status bit, answered ~0.8 s later
 *  - responses on 0x511: Set commands answered with DB0 | 0x80 and the command echoed, Gets with DB0 + 0x40
 *  - commands from other nodes on the bus are obeyed like the MCU's (sim_ivt_command)
 *  - configuration limits (chapters 4.1 and 8.3), flagged when run mode is entered: at most 1000
 *    result messages per second, current 1 ms minimum, U1..U3 share one ADC so each needs at least
 *    (number of voltage channels on) ms, in a multiple of that
 *
 * Also models the precharge circuit on the vehicle side of the AIRs, driven by the MCU's precharge
 * and bypass outputs: U2 charges towards U1 through the precharge resistor, follows U1 with the
 * bypass closed, and discharges otherwise.
 */

#include "sim.h"

#include <math.h>
#include <string.h>

#define IVT_STARTUP_DEFAULT_US  400000ULL         // Datasheet maximum; ~190 ms measured on the team's unit
static uint64_t IVT_STARTUP_US = IVT_STARTUP_DEFAULT_US;
#define IVT_NOMINAL_RANGE_A 500.0f
#define IVT_CHANNELS        8           // I, U1, U2, U3, T, W, As, Wh
#define CH_I                0
#define CH_U1               1
#define CH_U2               2
#define CH_U3               3
#define CH_W                5
#define CH_WH               7

typedef struct
{
    int      mode;              // 0 disabled, 1 triggered, 2 cyclic
    uint16_t period_ms;
    uint64_t next_us;
    uint8_t  counter;
} ivt_channel;

static struct
{
    bool        connected;
    bool        running;
    bool        u1_default;
    bool        u3_open;
    uint64_t    started_at;
    bool        alive_sent;
    bool        system_error;       // Latched until power-up (or cleared): status bit 3
    uint8_t     sys_counters[16];   // System error counters (lifetime)
    int         error_resets;       // Reset Error- and Logdata commands carried out
    void      (*command_hook)(const uint8_t* data);
    ivt_channel ch[IVT_CHANNELS];
    float       current_a;
    float       u1_v;
    float       u2_v;
    double      energy_wh;
    uint8_t     status;
    int         config_commands;
    int         boot_commands;
    int         config_violations;
    float       precharge_tau_s;
    float       discharge_tau_s;
    float       bypass_closed_at_ratio;
    int         bypass_closures;
    bool        last_bypass;
    sim_timer   timer;
} ivt;

static void restart_channels(void)
{
    uint64_t now = sim_now_us();
    for (int i = 0; i < IVT_CHANNELS; i++) ivt.ch[i].next_us = now + ivt.ch[i].period_ms * 1000ULL;
}

static void load_defaults(void)
{
    static const uint16_t periods[IVT_CHANNELS] = { 20, 60, 60, 60, 100, 30, 30, 30 };
    for (int i = 0; i < IVT_CHANNELS; i++)
    {
        int mode = 2;
        if (i == CH_U1) mode = ivt.u1_default ? 2 : 0;
        ivt.ch[i] = (ivt_channel){ mode, periods[i], 0, 0 };
    }
    ivt.running = true;
    ivt.started_at = sim_now_us() + IVT_STARTUP_US;
    ivt.alive_sent = false;
    restart_channels();
    for (int i = 0; i < IVT_CHANNELS; i++) ivt.ch[i].next_us += IVT_STARTUP_US;
}

static uint8_t status_for(int index)
{
    uint8_t status = ivt.status;
    if (index == CH_I && fabsf(ivt.current_a) > IVT_NOMINAL_RANGE_A) status |= 0x2;
    if (ivt.u3_open) status |= 0x4;
    if (ivt.system_error) status |= 0x8;
    return status;
}

static int32_t value_for(int index)
{
    switch (index)
    {
    case CH_I:  return (int32_t)lroundf(ivt.current_a * 1000.0f);
    case CH_U1: return (int32_t)lroundf(ivt.u1_v * 1000.0f);
    case CH_U2: return (int32_t)lroundf(ivt.u2_v * 1000.0f);
    case CH_W:  return (int32_t)lroundf(ivt.current_a * ivt.u1_v);
    case CH_WH: return (int32_t)lround(ivt.energy_wh);
    default:    return 0;
    }
}

static void send_result(int index)
{
    ivt_channel* c = &ivt.ch[index];
    int32_t value = value_for(index);
    uint8_t data[6];
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)((status_for(index) << 4) | (c->counter & 0x0F));
    data[2] = (uint8_t)((uint32_t)value >> 24);
    data[3] = (uint8_t)((uint32_t)value >> 16);
    data[4] = (uint8_t)((uint32_t)value >> 8);
    data[5] = (uint8_t)value;
    c->counter = (uint8_t)((c->counter + 1) & 0x0F);
    sim_can_node_send(2, 0x521U + (uint32_t)index, false, data, 6);
}

static bool output_active(GPIO_TypeDef* port, uint16_t pin, bool active_high)
{
    if (!sim_gpio_is_output(port, pin)) return false;       // Floating: PCB pull-down holds it off
    return sim_gpio_output(port, pin) == active_high;
}

static void update_circuit(float dt_s)
{
    bool precharge = output_active(CFG_PIN_PRECHARGE_PORT, CFG_PIN_PRECHARGE_PIN, CFG_PRECHARGE_ACTIVE_HIGH);
    bool bypass    = output_active(CFG_PIN_BYPASS_PORT, CFG_PIN_BYPASS_PIN, CFG_BYPASS_ACTIVE_HIGH);

    if (bypass && !ivt.last_bypass)
    {
        ivt.bypass_closures++;
        float ratio = ivt.u1_v > 0 ? ivt.u2_v / ivt.u1_v : 0.0f;
        if (ivt.bypass_closures == 1 || ratio < ivt.bypass_closed_at_ratio) ivt.bypass_closed_at_ratio = ratio;
    }
    ivt.last_bypass = bypass;

    if (bypass)
    {
        ivt.u2_v = ivt.u1_v;
    }
    else if (precharge && ivt.precharge_tau_s > 0)
    {
        ivt.u2_v += (ivt.u1_v - ivt.u2_v) * (1.0f - expf(-dt_s / ivt.precharge_tau_s));
    }
    else
    {
        ivt.u2_v -= ivt.u2_v * (1.0f - expf(-dt_s / ivt.discharge_tau_s));
    }

    // Energy counter integrates I x U1 while the IVT is running
    if (ivt.running && sim_now_us() >= ivt.started_at)
    {
        ivt.energy_wh += (double)ivt.current_a * ivt.u1_v * dt_s / 3600.0;
    }
}

static void tick(sim_timer* t)
{
    uint64_t now = sim_now_us();
    update_circuit(0.001f);

    if (ivt.connected && now >= ivt.started_at && !ivt.alive_sent)
    {
        // Alive message once start-up is over: 0xBF, command ID 0x411, serial number (as the team's unit sends it)
        static const uint8_t alive[8] = { 0xBF, 0x04, 0x11, 0x00, 0x99, 0x47, 0x0E, 0x00 };
        sim_can_node_send(2, 0x511, false, alive, 8);
        ivt.alive_sent = true;
    }
    if (ivt.connected && ivt.running && now >= ivt.started_at)
    {
        for (int i = 0; i < IVT_CHANNELS; i++)
        {
            ivt_channel* c = &ivt.ch[i];
            if (c->mode != 2 || now < c->next_us) continue;
            c->next_us += c->period_ms * 1000ULL;
            send_result(i);
        }
    }
    sim_timer_start(t, now + 1000);
}

static float messages_per_s(void)
{
    float total = 0;
    for (int i = 0; i < IVT_CHANNELS; i++)
        if (ivt.ch[i].mode == 2 && ivt.ch[i].period_ms > 0) total += 1000.0f / ivt.ch[i].period_ms;
    return total;
}

static void check_config(void)
{
    int voltages = 0;
    for (int i = CH_U1; i <= CH_U3; i++) if (ivt.ch[i].mode == 2) voltages++;

    for (int i = 0; i < IVT_CHANNELS; i++)
    {
        ivt_channel* c = &ivt.ch[i];
        if (c->mode != 2) continue;
        bool voltage = i >= CH_U1 && i <= CH_U3;
        int minimum = voltage && voltages > 1 ? voltages : 1;
        if (c->period_ms < minimum || (voltage && c->period_ms % minimum != 0))
        {
            ivt.config_violations++;
            sim_note("IVT result %d at %d ms: below %d ms or not a multiple of it", i, c->period_ms, minimum);
        }
    }
    if (messages_per_s() > 1000.0f)
    {
        ivt.config_violations++;
        sim_note("IVT configured for %.0f result messages per second (datasheet maximum 1000)", messages_per_s());
    }
}

static void respond(const uint8_t* cmd)
{
    uint8_t data[8];
    memcpy(data, cmd, 8);
    data[0] = (uint8_t)(cmd[0] | 0x80);             // Set commands: echoed, DB0 | 0x80
    sim_can_node_send(2, 0x511, false, data, 8);
}

static const uint8_t serial_number[4] = { 0x00, 0x99, 0x47, 0x0E };

static void log_system_error(int bit)
{
    if (ivt.sys_counters[bit] < 255) ivt.sys_counters[bit]++;
    ivt.system_error = true;
}

static void ev_errors_reset(void* arg)
{
    (void)arg;
    memset(ivt.sys_counters, 0, sizeof(ivt.sys_counters));
    ivt.system_error = false;
    ivt.error_resets++;
    uint8_t data[8] = { 0xB0, 0x01, 0x00, serial_number[0], serial_number[1], serial_number[2], serial_number[3], 0 };
    if (ivt.connected) sim_can_node_send(2, 0x511, false, data, 8);
}

// Gets and the error reset; true if the command was one of them
static bool handle_get_or_reset(const uint8_t* d)
{
    if (d[0] == 0x41 || d[0] == 0x7B || d[0] == 0x30)
    {
        if (ivt.running) return true;               // Not answered in run mode
        uint8_t data[8] = {0};
        if (d[0] == 0x41)
        {
            data[0] = 0x81;
            data[1] = d[1];
            if (d[1] == 0)
            {
                uint16_t mask = 0;
                for (int b = 0; b < 16; b++) if (ivt.sys_counters[b]) mask |= (uint16_t)(1U << b);
                data[2] = (uint8_t)mask;
                data[3] = (uint8_t)(mask >> 8);
            }
            else if (d[1] <= 16)
            {
                data[2] = ivt.sys_counters[d[1] - 1];
            }
            sim_can_node_send(2, 0x511, false, data, 8);
        }
        else if (d[0] == 0x7B)
        {
            data[0] = 0xBB;
            memcpy(&data[1], serial_number, 4);
            sim_can_node_send(2, 0x511, false, data, 8);
        }
        else if (d[1] == 0x01 && d[2] == 0x00 && memcmp(&d[3], serial_number, 4) == 0)
        {
            sim_schedule(sim_now_us() + 785000, ev_errors_reset, NULL);
        }
        return true;
    }
    return false;
}

static void handle_command(const uint8_t* d)
{
    if (sim_now_us() < ivt.started_at)              // Still starting up: commands are lost
    {
        ivt.boot_commands++;
        return;
    }
    if (ivt.command_hook) ivt.command_hook(d);

    if (d[0] == 0x3F)                               // RESTART: no response, it starts over
    {
        load_defaults();
        log_system_error(14);                       // Logged as a watchdog reset
        return;
    }
    if (handle_get_or_reset(d)) return;
    if (d[0] == 0x34)                               // SET_MODE
    {
        bool was_running = ivt.running;
        ivt.running = d[1] == 0x01;
        if (ivt.running) restart_channels();
        if (ivt.running && !was_running) check_config();
    }
    else if ((d[0] & 0xF0) == 0x20 && (d[0] & 0x0F) < IVT_CHANNELS)    // Set config result
    {
        if (!ivt.running)
        {
            ivt_channel* c = &ivt.ch[d[0] & 0x0F];
            uint16_t period = (uint16_t)((d[2] << 8) | d[3]);
            if (period == 0)
            {
                sim_note("IVT ignored a config command with time 0 (result %d)", d[0] & 0x0F);
            }
            else
            {
                c->mode = d[1] & 0x0F;
                c->period_ms = period;
            }
            ivt.config_commands++;
        }
        else
        {
            sim_note("IVT got a config command while running (ignored)");
        }
    }
    respond(d);
}

static void on_mcu_frame(const sim_can_frame* f)
{
    if (!ivt.connected || f->ext || f->rtr || f->id != 0x411 || f->dlc < 1) return;
    handle_command(f->data);
}

void sim_ivt_command(const uint8_t* data)
{
    if (ivt.connected) handle_command(data);
}

void sim_ivt_set_command_hook(void (*hook)(const uint8_t* data)) { ivt.command_hook = hook; }
void sim_ivt_log_system_error(int bit)       { log_system_error(bit); }
int  sim_ivt_system_error_count(int bit)     { return ivt.sys_counters[bit]; }
int  sim_ivt_error_resets(void)              { return ivt.error_resets; }
bool sim_ivt_running(void)                   { return ivt.connected && ivt.running && sim_now_us() >= ivt.started_at; }

void sim_ivt_init(bool u1_enabled_by_default)
{
    memset(&ivt, 0, sizeof(ivt));
    IVT_STARTUP_US = IVT_STARTUP_DEFAULT_US;
    ivt.u1_default = u1_enabled_by_default;
    ivt.connected = true;
    ivt.u3_open = true;                             // U3 isn't wired on the car
    ivt.energy_wh = 12345.0;                        // Counter carries history from earlier runs
    ivt.precharge_tau_s = 0.4f;
    ivt.discharge_tau_s = 1.5f;
    load_defaults();
    sim_can_set_listener(2, on_mcu_frame);
    ivt.timer.fire = tick;
    sim_timer_start(&ivt.timer, sim_now_us() + 1000);
}

void  sim_ivt_set_current(float amps)        { ivt.current_a = amps; }
void  sim_ivt_set_u1(float volts)            { ivt.u1_v = volts; }
void  sim_ivt_set_status(uint8_t s)          { ivt.status = s & 0x0F; }
void  sim_ivt_set_u23_open(bool open)        { ivt.u3_open = open; }
void  sim_ivt_set_precharge_tau(float s)     { ivt.precharge_tau_s = s; }
int   sim_ivt_config_commands(void)          { return ivt.config_commands; }
int   sim_ivt_boot_commands(void)            { return ivt.boot_commands; }
int   sim_ivt_config_violations(void)        { return ivt.config_violations; }
float sim_ivt_messages_per_s(void)           { return messages_per_s(); }
bool  sim_ivt_u1_cyclic(void)                { return ivt.ch[CH_U1].mode == 2; }
bool  sim_ivt_channel_cyclic(int index)      { return ivt.ch[index].mode == 2; }
float sim_ivt_u2(void)                       { return ivt.u2_v; }
float sim_ivt_u1(void)                       { return ivt.u1_v; }
double sim_ivt_energy_wh(void)               { return ivt.energy_wh; }
float sim_ivt_bypass_closed_ratio(void)      { return ivt.bypass_closed_at_ratio; }
int   sim_ivt_bypass_closures(void)          { return ivt.bypass_closures; }

void sim_ivt_set_connected(bool connected)
{
    ivt.connected = connected;
    sim_can_set_ack(2, connected);
}

void sim_ivt_set_startup_ms(int ms) { IVT_STARTUP_US = (uint64_t)ms * 1000ULL; }

void sim_ivt_power_cycle(void)
{
    ivt.energy_wh -= 37.0;                          // Restarts from the last stored log value
    load_defaults();
    ivt.system_error = false;
}
