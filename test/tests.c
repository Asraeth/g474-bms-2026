/*
 * tests.c
 *
 * Scenario tests: the unmodified firmware (from main()) runs against the simulated board.
 * Each test runs in its own process:  bms_tests <test name>  (see run_tests.sh).
 */

#include "sim.h"
#include "bms_libWrapper.h"
#include "bms_safety.h"
#include "bms_soc.h"
#include "bms_precharge.h"
#include "bms_inverter.h"
#include "bms_settings.h"
#include "bms_charger_codec.h"
#include "uartDMA.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sim_uart_set_file(FILE* f);
extern volatile uint32_t droppedMessages;
extern volatile uint32_t canTxErrorCount;
extern volatile uint32_t canFastTxDropped;

static const char* out_dir = "test/build";

// Slave numbers in the tests are for the car's chain; with fewer slaves (BENCH_CONFIG) they wrap
#define SLAVE(n) ((n) % CFG_NUM_SLAVES)


/* ===================================================================================== */
/*  Pack helpers                                                                          */
/* ===================================================================================== */

static const uint8_t cell_channel[CFG_CELLS_PER_SLAVE] = CFG_SLAVE_CELL_CHANNELS;
static const uint8_t temp_gpio[CFG_TEMPS_PER_SLAVE]    = CFG_SLAVE_TEMP_GPIOS;
static const float   table_c[] = CFG_TEMP_TABLE_C;
static const float   table_v[] = CFG_TEMP_TABLE_V;

static float temp_to_voltage(float celsius)
{
    int n = (int)(sizeof(table_c) / sizeof(table_c[0]));
    if (celsius <= table_c[0]) return table_v[0];
    if (celsius >= table_c[n - 1]) return table_v[n - 1];
    for (int i = 0; i < n - 1; i++)
    {
        if (celsius <= table_c[i + 1])
        {
            float f = (celsius - table_c[i]) / (table_c[i + 1] - table_c[i]);
            return table_v[i] + f * (table_v[i + 1] - table_v[i]);
        }
    }
    return table_v[n - 1];
}

static float pack_sum(void)
{
    float sum = 0;
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
            sum += sim_adbms_get_cell(s, cell_channel[c] - 1);
    return sum;
}

static bool keep_u1_matched = true;

static void set_cell(int slave, int cell, float volts)
{
    sim_adbms_set_cell(slave, cell_channel[cell] - 1, volts);
    if (keep_u1_matched) sim_ivt_set_u1(pack_sum());
}

static void set_temp(int slave, int sensor, float celsius)
{
    sim_adbms_set_gpio(slave, temp_gpio[sensor], temp_to_voltage(celsius));
}

static float ivt_reported(float discharge_amps)
{
    return CFG_IVT_CURRENT_DISCHARGE_POSITIVE ? discharge_amps : -discharge_amps;
}

static void healthy_pack(void)
{
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        for (int ch = 0; ch < 16; ch++) sim_adbms_set_cell(s, ch, 0.0f);    // Unused inputs shorted
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) sim_adbms_set_cell(s, cell_channel[c] - 1, 3.80f);
        for (int g = 1; g <= 10; g++) sim_adbms_set_gpio(s, g, 3.0f);
        for (int t = 0; t < CFG_TEMPS_PER_SLAVE; t++) set_temp(s, t, 25.0f);
        sim_adbms_set_die_temp(s, 30.0f);
    }
    /* A healthy fixture has no global IVT result error; open U3 is tested explicitly. */
    sim_ivt_set_u23_open(false);
    sim_ivt_set_current(ivt_reported(20.0f));
    sim_ivt_set_u1(pack_sum());
    sim_gpio_set_input(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN, CFG_SDC_IN_ACTIVE_HIGH);
    sim_gpio_set_input(CFG_PIN_CHARGER_BTN_PORT, CFG_PIN_CHARGER_BTN_PIN, CFG_CHARGER_BTN_ACTIVE_LOW);
}

// Relay invariants, checked on every write to the fault, precharge or bypass pin:
//  - no relay closed while the fault output is active
//  - the bypass only closes with the vehicle side at the precharge target
static int      relay_violations = 0;
static bool     bypass_was_on = false;
static int      bypass_closures = 0;
static uint64_t bypass_on_time = 0;
static uint64_t precharge_off_time = 0;
static uint64_t precharge_on_time = 0;
static bool     precharge_was_on = false;

static bool output_on(GPIO_TypeDef* port, uint16_t pin, bool active_high)
{
    return sim_gpio_is_output(port, pin) && sim_gpio_output(port, pin) == active_high;
}

static bool precharge_on(void) { return output_on(CFG_PIN_PRECHARGE_PORT, CFG_PIN_PRECHARGE_PIN, CFG_PRECHARGE_ACTIVE_HIGH); }
static bool bypass_on(void)    { return output_on(CFG_PIN_BYPASS_PORT, CFG_PIN_BYPASS_PIN, CFG_BYPASS_ACTIVE_HIGH); }

static void relay_hook(GPIO_TypeDef* port, uint16_t pin, bool level)
{
    (void)level;
    bool relevant = (port == CFG_PIN_FAULT_PORT && pin == CFG_PIN_FAULT_PIN) ||
                    (port == CFG_PIN_PRECHARGE_PORT && pin == CFG_PIN_PRECHARGE_PIN) ||
                    (port == CFG_PIN_BYPASS_PORT && pin == CFG_PIN_BYPASS_PIN);
    if (!relevant) return;

    bool pre = precharge_on();
    bool byp = bypass_on();
    if (sim_fault_active() && (pre || byp))
    {
        if (relay_violations++ == 0) sim_fail("relay closed while the fault output is active (precharge %d, bypass %d)", pre, byp);
    }
    if (byp && !bypass_was_on)
    {
        bypass_closures++;
        bypass_on_time = sim_now_us();
        float ratio = sim_ivt_u1() > 0 ? sim_ivt_u2() / sim_ivt_u1() : 0.0f;
        if (ratio * 100.0f < CFG_PRECHARGE_TARGET_PERCENT)
        {
            relay_violations++;
            sim_fail("bypass closed with the vehicle side at %.1f %% of the battery", ratio * 100.0f);
        }
    }
    if (pre && !precharge_was_on) precharge_on_time = sim_now_us();
    if (!pre && precharge_was_on)
    {
        precharge_off_time = sim_now_us();
        uint64_t limit = (uint64_t)(CFG_PRECHARGE_TIMEOUT_MS + CFG_PRECHARGE_OVERLAP_MS + 2 * CFG_SAFETY_TICK_MS) * 1000ULL;
        if (precharge_off_time - precharge_on_time > limit)
        {
            relay_violations++;
            sim_fail("precharge relay closed for %.0f ms", (precharge_off_time - precharge_on_time) / 1000.0);
        }
    }
    bypass_was_on = byp;
    precharge_was_on = pre;
}

// Fast frames, timed as they leave the MCU (after start-up)
#define FAST_WATCH_FROM_US  SIM_S(3)
static uint64_t fast_last[2];
static uint64_t fast_max_gap = 0;
static uint64_t fast_min_gap = UINT64_MAX;

static int fast_index(uint32_t id)
{
    return id == CFG_CAN_ID_FAST_1 ? 0 : (id == CFG_CAN_ID_FAST_2 ? 1 : -1);
}

static bool inverter_limit_id(uint32_t id)
{
    return id == CFG_CAN_ID_INVERTER_DC_LIMIT || id == CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT || id == CFG_CAN_ID_LIMITS;
}

// Extra handler for frames the MCU sends on CAN1 (e.g. the inverter model)
static void (*can1_hook)(const sim_can_frame* f) = NULL;

static void can1_listener(const sim_can_frame* f)
{
    if (can1_hook) can1_hook(f);
    int k = fast_index(f->id);
    if (k < 0 || !f->ext) return;
    if (f->t_us >= FAST_WATCH_FROM_US && fast_last[k] >= FAST_WATCH_FROM_US)
    {
        uint64_t gap = f->t_us - fast_last[k];
        if (gap > fast_max_gap) fast_max_gap = gap;
        if (gap < fast_min_gap) fast_min_gap = gap;
    }
    fast_last[k] = f->t_us;
}

// Fast frames every CFG_CAN_FAST_PERIOD_MS, never late by more than a couple of frames on the bus
static void check_fast_timing(void)
{
    uint64_t period = CFG_CAN_FAST_PERIOD_MS * 1000ULL;
    sim_note("fast frames every %.2f .. %.2f ms", fast_min_gap / 1000.0, fast_max_gap / 1000.0);
    SIM_CHECK(fast_max_gap <= period + 1000 && fast_min_gap + 1000 >= period,
              "fast frame gaps %.2f .. %.2f ms, period %d ms", fast_min_gap / 1000.0, fast_max_gap / 1000.0, CFG_CAN_FAST_PERIOD_MS);
    SIM_CHECK(canFastTxDropped == 0, "%lu fast frames dropped", (unsigned long)canFastTxDropped);
}

static void board_init(bool u1_default)
{
    sim_adbms_init(CFG_NUM_SLAVES);
    sim_ivt_init(u1_default);
    healthy_pack();
    sim_gpio_add_write_hook(relay_hook);
    sim_can_set_listener(1, can1_listener);
}


/* ===================================================================================== */
/*  Output analysis                                                                       */
/* ===================================================================================== */

static uint64_t first_fault_after(uint64_t t)
{
    const sim_transition* list;
    int n = sim_fault_transitions(&list);
    for (int i = 0; i < n; i++)
    {
        if (list[i].state && list[i].t_us >= t) return list[i].t_us;
    }
    return UINT64_MAX;
}

static uint64_t first_release(void)
{
    const sim_transition* list;
    int n = sim_fault_transitions(&list);
    for (int i = 0; i < n; i++)
    {
        if (!list[i].state) return list[i].t_us;
    }
    return UINT64_MAX;
}

static int fault_asserts_between(uint64_t from, uint64_t to)
{
    const sim_transition* list;
    int n = sim_fault_transitions(&list);
    int count = 0;
    for (int i = 0; i < n; i++)
    {
        if (list[i].state && list[i].t_us > from && list[i].t_us < to) count++;
    }
    return count;
}

static int16_t le16(const uint8_t* p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

static int32_t le32(const uint8_t* p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// Longest main loop after the first one (which also initialises the chain and configures the IVT)
static int max_loop_time_ms(void)
{
    int worst = 0;
    bool first = true;
    const char* p = sim_uart_text();
    while ((p = strstr(p, "LoopTime: ")) != NULL)
    {
        int v = atoi(p + 10);
        if (!first && v > worst) worst = v;
        first = false;
        p += 10;
    }
    return worst;
}

// Frames of the last complete CAN1 burst (starting at the pack frame), without the fast and inverter frames
static int last_burst(const sim_can_frame** out, int max)
{
    int n = sim_can_log_count();
    int start = -1, prev_start = -1;
    for (int i = 0; i < n; i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == 1 && f->from_mcu && f->id == CFG_CAN_ID_PACK)
        {
            prev_start = start;
            start = i;
        }
    }
    if (prev_start < 0) return 0;
    int count = 0;
    for (int i = prev_start; i < start && count < max; i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == 1 && f->from_mcu && fast_index(f->id) < 0 && !inverter_limit_id(f->id)) out[count++] = f;
    }
    return count;
}

static const sim_can_frame* find_frame(const sim_can_frame** burst, int n, uint32_t id)
{
    for (int i = 0; i < n; i++)
    {
        if (burst[i]->id == id) return burst[i];
    }
    return NULL;
}

// Checks every test makes
static void common_checks(bool expect_release)
{
    SIM_CHECK(sim_error_handler_calls == 0, "Error_Handler was reached");
    SIM_CHECK(sim_adbms_unknown_commands() == 0, "unknown isoSPI commands were sent");
    SIM_CHECK(!sim_ready_was_on_while_faulted(), "TSAC ready relay was on while the fault output was active");
    SIM_CHECK(sim_wdg_started(), "watchdog was never started");
    SIM_CHECK(sim_wdg_max_gap_us() < sim_wdg_timeout_us(),
              "longest gap between watchdog kicks %.1f ms >= timeout %.1f ms",
              sim_wdg_max_gap_us() / 1000.0, sim_wdg_timeout_us() / 1000.0);
    if (expect_release)
    {
        SIM_CHECK(first_release() != UINT64_MAX, "fault output was never released");
    }
    SIM_CHECK(relay_violations == 0, "%d relay invariant violations", relay_violations);
    SIM_CHECK(sim_ivt_config_violations() == 0, "IVT configured outside its datasheet limits");
    sim_note("watchdog: longest kick gap %.1f ms (timeout %.1f ms), longest loop %d ms",
             sim_wdg_max_gap_us() / 1000.0, sim_wdg_timeout_us() / 1000.0, max_loop_time_ms());
}


/* ===================================================================================== */
/*  Fault reaction sweeps                                                                 */
/* ===================================================================================== */

typedef struct
{
    const char* name;
    void      (*inject)(void);
    void      (*restore)(void);
    uint32_t    flag;               // Expected BMS_StatusTypeDef bit while faulted
    uint64_t    limit_us;           // Maximum allowed reaction time
} fault_case;

static const fault_case* current_case;
static int      iteration = 0;
static int      iterations = 12;
static uint64_t inject_time;
static uint64_t worst_latency = 0;
static uint64_t best_latency = UINT64_MAX;

static void ev_inject(void* arg);

static void ev_verify_clear(void* arg)
{
    (void)arg;
    SIM_CHECK(!sim_fault_active(), "%s: fault output still active 2.5 s after the fault was removed (flags 0x%02lX)",
              current_case->name, (unsigned long)BMS_Safety_GetActiveFlags());
    SIM_CHECK(sim_tsac_ready(), "%s: TSAC ready relay not back on after the fault cleared", current_case->name);

    if (++iteration < iterations)
    {
        // Shift the injection phase against the 100 ms measurement cycle each time
        sim_schedule(sim_now_us() + 300000 + (uint64_t)iteration * 9000, ev_inject, NULL);
    }
}

static void ev_check(void* arg)
{
    (void)arg;
    uint64_t asserted = first_fault_after(inject_time);
    if (asserted == UINT64_MAX)
    {
        sim_fail("%s: no fault within %.0f ms of the injection", current_case->name, (sim_now_us() - inject_time) / 1000.0);
    }
    else
    {
        uint64_t latency = asserted - inject_time;
        if (latency > worst_latency) worst_latency = latency;
        if (latency < best_latency) best_latency = latency;
        SIM_CHECK(latency <= current_case->limit_us, "%s: fault output after %.1f ms (limit %.0f ms)",
                  current_case->name, latency / 1000.0, current_case->limit_us / 1000.0);
    }
    SIM_CHECK(BMS_Safety_GetActiveFlags() & current_case->flag, "%s: expected flag 0x%02lX, active flags 0x%02lX",
              current_case->name, (unsigned long)current_case->flag, (unsigned long)BMS_Safety_GetActiveFlags());
    SIM_CHECK(!sim_tsac_ready(), "%s: TSAC ready relay still on during the fault", current_case->name);

    current_case->restore();
    sim_schedule(sim_now_us() + 2500000, ev_verify_clear, NULL);
}

static void ev_inject(void* arg)
{
    (void)arg;
    SIM_CHECK(!sim_fault_active(), "%s: fault output already active before injection %d (flags 0x%02lX)",
              current_case->name, iteration, (unsigned long)BMS_Safety_GetActiveFlags());
    inject_time = sim_now_us();
    current_case->inject();
    sim_schedule(inject_time + 700000, ev_check, NULL);
}

static void run_fault_case(const fault_case* fc)
{
    current_case = fc;
    board_init(true);
    sim_schedule(SIM_MS(4000), ev_inject, NULL);
    uint64_t end = SIM_MS(4000) + (uint64_t)iterations * 3600000ULL;
    sim_run_firmware(end);

    common_checks(true);
    SIM_CHECK(iteration == iterations, "%s: only %d of %d injections completed", fc->name, iteration, iterations);
    sim_note("%s: reaction %.1f .. %.1f ms over %d injections", fc->name,
             best_latency / 1000.0, worst_latency / 1000.0, iteration);
}

#define FS_LIMIT_US     ((uint64_t)CFG_FS_MAX_REACTION_MS * 1000ULL)

static void inj_cell_ov(void)   { set_cell(SLAVE(7), 3, CFG_CELL_OV_V + 0.05f); }
static void inj_cell_uv(void)   { set_cell(SLAVE(9), 0, CFG_CELL_UV_V - 0.05f); }
static void res_cell(void)      { set_cell(SLAVE(7), 3, 3.80f); set_cell(SLAVE(9), 0, 3.80f); }
static void inj_cell_ot(void)   { set_temp(SLAVE(3), 5, CFG_CELL_TEMP_MAX_C + 1.0f); }
static void inj_cell_ut(void)   { set_temp(SLAVE(3), 5, CFG_CELL_TEMP_MIN_C - 1.0f); }
static void inj_sensor_open(void) { sim_adbms_set_gpio(SLAVE(3), temp_gpio[5], 3.0f); }
static void res_temp(void)      { set_temp(SLAVE(3), 5, 25.0f); }
static void inj_die_ot(void)    { sim_adbms_set_die_temp(SLAVE(12), CFG_SLAVE_DIE_TEMP_MAX_C + 5.0f); }
static void res_die(void)       { sim_adbms_set_die_temp(SLAVE(12), 30.0f); }
static void inj_open_wire(void) { sim_adbms_set_open_wire(SLAVE(5), cell_channel[2] - 1, true); }
static void res_open_wire(void) { sim_adbms_set_open_wire(SLAVE(5), cell_channel[2] - 1, false); }
static void inj_oc_dis(void)    { sim_ivt_set_current(ivt_reported(CFG_MAX_DISCHARGE_CURRENT_A + 10.0f)); }
static void inj_oc_chg(void)    { sim_ivt_set_current(ivt_reported(-(CFG_MAX_CHARGE_CURRENT_A + 10.0f))); }
static void inj_oc_range(void)  { sim_ivt_set_current(ivt_reported(600.0f)); }     // Beyond the IVT-S-500's nominal range
static void res_current(void)   { sim_ivt_set_current(ivt_reported(20.0f)); }
static void inj_pack_ov(void)   { keep_u1_matched = false; sim_ivt_set_u1(CFG_PACK_V_MAX + 5.0f); }
static void inj_pack_mismatch(void) { keep_u1_matched = false; sim_ivt_set_u1(pack_sum() + CFG_PACK_V_PLAUSIBILITY_V + 5.0f); }
static void res_pack(void)      { keep_u1_matched = true; sim_ivt_set_u1(pack_sum()); }
static void inj_ivt_lost(void)  { sim_ivt_set_connected(false); }
static void res_ivt_lost(void)  { sim_ivt_set_connected(true); }
static void inj_ivt_error(void) { sim_ivt_set_status(0x8); }
static void res_ivt_error(void) { sim_ivt_set_status(0x0); }
static void inj_chain(void)     { sim_adbms_set_broken_after(CFG_NUM_SLAVES > 6 ? 5 : SIM_CHAIN_BROKEN_AT_MASTER); }
static void res_chain(void)     { sim_adbms_set_broken_after(-1); }
static void inj_spi(void)       { sim_adbms_set_spi_error(true); }
static void res_spi(void)       { sim_adbms_set_spi_error(false); }

static const fault_case fault_cases[] = {
    { "cell_overvoltage",      inj_cell_ov,        res_cell,      BMS_ERR_VOLTAGE,   FS_LIMIT_US },
    { "cell_undervoltage",     inj_cell_uv,        res_cell,      BMS_ERR_VOLTAGE,   FS_LIMIT_US },
    { "cell_overtemp",         inj_cell_ot,        res_temp,      BMS_ERR_TEMP,      FS_LIMIT_US },
    { "cell_undertemp",        inj_cell_ut,        res_temp,      BMS_ERR_TEMP,      FS_LIMIT_US },
    { "temp_sensor_open",      inj_sensor_open,    res_temp,      BMS_ERR_TEMP,      FS_LIMIT_US },
    { "slave_die_overtemp",    inj_die_ot,         res_die,       BMS_ERR_TEMP,      FS_LIMIT_US },
    { "open_sense_wire",       inj_open_wire,      res_open_wire, BMS_ERR_OPEN_WIRE, FS_LIMIT_US },
    { "overcurrent_discharge", inj_oc_dis,         res_current,   BMS_ERR_CURRENT,   FS_LIMIT_US },
    { "overcurrent_charge",    inj_oc_chg,         res_current,   BMS_ERR_CURRENT,   FS_LIMIT_US },
    { "overcurrent_ivt_range", inj_oc_range,       res_current,   BMS_ERR_CURRENT,   FS_LIMIT_US },
    { "pack_overvoltage",      inj_pack_ov,        res_pack,      BMS_ERR_VOLTAGE,   FS_LIMIT_US },
    { "pack_implausible",      inj_pack_mismatch,  res_pack,      BMS_ERR_VOLTAGE,   FS_LIMIT_US },
    { "ivt_lost",              inj_ivt_lost,       res_ivt_lost,  BMS_ERR_IVT,       FS_LIMIT_US },
    { "ivt_sensor_error",      inj_ivt_error,      res_ivt_error, BMS_ERR_IVT,       FS_LIMIT_US },
    { "isospi_chain_broken",   inj_chain,          res_chain,     BMS_ERR_COMMS,     FS_LIMIT_US },
    { "spi_timeout",           inj_spi,            res_spi,       BMS_ERR_COMMS,     FS_LIMIT_US },
};


/* ===================================================================================== */
/*  Scenario tests                                                                        */
/* ===================================================================================== */

static void test_boot(void)
{
    board_init(true);
    sim_run_firmware(SIM_S(8));

    common_checks(true);
    uint64_t release = first_release();
    sim_note("fault output released at %.3f s", release / 1e6);
    SIM_CHECK(release < SIM_S(4), "fault output released only at %.2f s", release / 1e6);
    SIM_CHECK(fault_asserts_between(release, SIM_S(8)) == 0, "fault output asserted again after start-up");
    SIM_CHECK(sim_ivt_config_commands() == 8, "IVT got %d result config commands, expected 8", sim_ivt_config_commands());
    SIM_CHECK(sim_ivt_channel_cyclic(0) && sim_ivt_channel_cyclic(1) && sim_ivt_channel_cyclic(2) && sim_ivt_channel_cyclic(7) &&
              !sim_ivt_channel_cyclic(3) && !sim_ivt_channel_cyclic(4) && !sim_ivt_channel_cyclic(5) && !sim_ivt_channel_cyclic(6),
              "IVT results not configured as I, U1, U2, Wh cyclic and U3, T, W, As off");
    sim_note("IVT sends %.0f result messages per second", sim_ivt_messages_per_s());
    SIM_CHECK(sim_ivt_messages_per_s() <= 1000.0f, "IVT at %.0f messages per second", sim_ivt_messages_per_s());
    SIM_CHECK(!precharge_on() && !bypass_on(), "precharge or bypass closed without a TS request");
    SIM_CHECK(sim_can_bitrate(1) == CFG_CAN1_BITRATE, "CAN1 at %lu bit/s", (unsigned long)sim_can_bitrate(1));
    SIM_CHECK(sim_can_bitrate(2) == CFG_CAN2_BITRATE, "CAN2 at %lu bit/s", (unsigned long)sim_can_bitrate(2));
    SIM_CHECK(sim_can_bitrate(3) == CFG_CAN3_BITRATE, "CAN3 at %lu bit/s", (unsigned long)sim_can_bitrate(3));
    uint64_t tick_us = (htim16.Init.Prescaler + 1ULL) * (htim16.Init.Period + 1ULL) * 1000000ULL / 128000000ULL;
    SIM_CHECK(tick_us == CFG_TIMER_TICK_MS * 1000ULL, "TIM16 period %.3f ms, CFG_TIMER_TICK_MS %d", tick_us / 1000.0, CFG_TIMER_TICK_MS);
    SIM_CHECK(sim_can_irq_enabled(1) && sim_can_irq_enabled(2) && sim_can_irq_enabled(3), "FDCAN interrupt not enabled in the NVIC");
    SIM_CHECK(sim_can_rx_lost(1) == 0 && sim_can_rx_lost(2) == 0, "CAN RX FIFO overflowed (%d / %d frames lost)",
              sim_can_rx_lost(1), sim_can_rx_lost(2));
    SIM_CHECK(max_loop_time_ms() < CFG_MEAS_PERIOD_MS, "main loop took %d ms (period %d ms)", max_loop_time_ms(), CFG_MEAS_PERIOD_MS);
    SIM_CHECK(sim_uart_count("Measurement cycle") == 0, "measurement cycle longer than CFG_MEAS_CYCLE_MAX_MS (%d warnings)",
              sim_uart_count("Measurement cycle"));
    SIM_CHECK(droppedMessages == 0, "%lu UART messages dropped", (unsigned long)droppedMessages);
    SIM_CHECK(canTxErrorCount == 0, "%lu CAN frames could not be queued", (unsigned long)canTxErrorCount);
    SIM_CHECK((sim_uart_count("BENCH BUILD") > 0) == (CFG_LOCAL_OVERRIDES != 0), "bench banner %d times, overrides %d",
              sim_uart_count("BENCH BUILD"), CFG_LOCAL_OVERRIDES);
    check_fast_timing();
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        SIM_CHECK(sim_adbms_refon(s), "slave %d: REFON not set", s);
        SIM_CHECK(sim_adbms_gpo(s) == 0x3FF, "slave %d: GPO 0x%03X, expected all pull-downs off", s, sim_adbms_gpo(s));
        for (int ch = 0; ch < 16; ch++)
            SIM_CHECK(sim_adbms_pwm(s, ch) == 0, "slave %d: balancing channel %d on a balanced pack", s, ch);
    }
}

static void test_boot_u1_disabled(void)
{
    board_init(false);
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    SIM_CHECK(sim_ivt_u1_cyclic(), "IVT U1 not switched to cyclic");
    SIM_CHECK(first_release() < SIM_S(4), "fault output released only at %.2f s", first_release() / 1e6);
}

// Settings traffic for test/check_dbc.py: read all, then a refused write (above the built-in power limit)
static void ev_layout_read_settings(void* arg)
{
    (void)arg;
    uint8_t d[8] = { 5, 0, 0, 0, 0, 0, 0, 0 };
    sim_can_node_send(1, CFG_CAN_ID_CONFIG_REQUEST, true, d, 8);
}

static void ev_layout_bad_setting(void* arg)
{
    (void)arg;
    int32_t v = (int32_t)lroundf(CFG_INVERTER_POWER_LIMIT_W) + 1;
    uint8_t d[8] = { 2, 1, 0, 0, (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    sim_can_node_send(1, CFG_CAN_ID_CONFIG_REQUEST, true, d, 8);
}

static void ev_layout_load(void *unused) {(void)unused;sim_ivt_set_current(ivt_reported(123.4f));}
static void test_can_layout(void)
{
    board_init(true);
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) set_cell(s, c, 3.70f + s * 0.004f + c * 0.001f);
        for (int t = 0; t < CFG_TEMPS_PER_SLAVE; t++) set_temp(s, t, 20.0f + s * 1.0f + t * 0.5f);
        sim_adbms_set_die_temp(s, 35.0f + s);
    }
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(1.5),ev_layout_load,NULL);
    sim_schedule(SIM_S(4.5), ev_layout_read_settings, NULL);
    sim_schedule(SIM_S(4.6), ev_layout_bad_setting, NULL);
    sim_run_firmware(SIM_S(6));
    common_checks(true);

    char path[256];
    snprintf(path, sizeof(path), "%s/can_layout.csv", out_dir);
    sim_can_log_write_csv(path);

    const sim_can_frame* burst[256];
    int n = last_burst(burst, 256);
    SIM_CHECK(n == CAN_BUFFER_LEN, "burst has %d frames, expected %d", n, CAN_BUFFER_LEN);
    if (n < 3) return;

    SIM_CHECK(burst[0]->id == CFG_CAN_ID_PACK && burst[1]->id == CFG_CAN_ID_STATUS && burst[2]->id == CFG_CAN_ID_SUMMARY,
              "burst does not start with pack, status, summary");
    for (int i = 0; i < n; i++)
    {
        SIM_CHECK(burst[i]->ext && burst[i]->dlc == 8, "frame 0x%X: ext %d dlc %d", burst[i]->id, burst[i]->ext, burst[i]->dlc);
    }

    // Burst period and duration
    uint64_t t_first = 0, t_prev = 0, max_period = 0, min_period = UINT64_MAX;
    int bursts = 0;
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (!(f->bus == 1 && f->from_mcu && f->id == CFG_CAN_ID_PACK) || f->t_us < SIM_S(3)) continue;
        if (bursts++ == 0) t_first = f->t_us;
        else
        {
            uint64_t p = f->t_us - t_prev;
            if (p > max_period) max_period = p;
            if (p < min_period) min_period = p;
        }
        t_prev = f->t_us;
    }
    (void)t_first;
    sim_note("CAN1 bursts every %.1f .. %.1f ms, burst duration %.1f ms", min_period / 1000.0, max_period / 1000.0,
             (burst[n - 1]->t_us - burst[0]->t_us) / 1000.0);
    SIM_CHECK(max_period <= CFG_CAN_TX_PERIOD_MS * 1000ULL + 15000, "CAN burst gap up to %.1f ms", max_period / 1000.0);
    check_fast_timing();

    // Charger command: on CAN3 once per burst, never on the car bus
    int on_can3 = 0, on_can1 = 0;
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (!f->from_mcu || f->id != CHARGER_CONFIG_CAN_ID) continue;
        if (f->bus == 3 && f->t_us >= SIM_S(3) && f->t_us < SIM_S(6)) on_can3++;
        if (f->bus == 1) on_can1++;
    }
    int expect = 3000 / CFG_CAN_TX_PERIOD_MS;
    SIM_CHECK(abs(on_can3 - expect) <= 1 && on_can1 == 0, "charger command: %d on CAN3 in 3 s (expected %d), %d on CAN1",
              on_can3, expect, on_can1);
    SIM_CHECK(sim_uart_count("Overwritten") == 0, "CAN bursts overlapped (%d times)", sim_uart_count("Overwritten"));

    // Cell voltages and temperatures
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        for (int k = 0; k < CFG_CAN_V_FRAMES_PER_SLAVE; k++)
        {
            const sim_can_frame* f = find_frame(burst, n, CFG_CAN_ID_CELL_V_BASE + s * CFG_CAN_V_FRAMES_PER_SLAVE + k);
            SIM_CHECK(f != NULL, "missing cell voltage frame slave %d frame %d", s, k);
            if (!f) continue;
            for (int j = 0; j < 4; j++)
            {
                int c = k * 4 + j;
                if (c >= CFG_CELLS_PER_SLAVE) continue;
                float expect = 3.70f + s * 0.004f + c * 0.001f;
                float got = le16(&f->data[j * 2]) / 1000.0f;
                SIM_CHECK(fabsf(got - expect) < 0.002f, "slave %d cell %d: CAN %.3f V, actual %.3f V", s, c, got, expect);
            }
        }
        for (int k = 0; k < CFG_CAN_T_FRAMES_PER_SLAVE; k++)
        {
            const sim_can_frame* f = find_frame(burst, n, CFG_CAN_ID_CELL_T_BASE + s * CFG_CAN_T_FRAMES_PER_SLAVE + k);
            SIM_CHECK(f != NULL, "missing temperature frame slave %d frame %d", s, k);
            if (!f) continue;
            for (int j = 0; j < 4; j++)
            {
                int t = k * 4 + j;
                if (t >= CFG_TEMPS_PER_SLAVE) continue;
                float expect = 20.0f + s * 1.0f + t * 0.5f;
                float got = le16(&f->data[j * 2]) / 10.0f;
                SIM_CHECK(fabsf(got - expect) < 0.3f, "slave %d sensor %d: CAN %.1f C, actual %.1f C", s, t, got, expect);
            }
        }
        const sim_can_frame* f = find_frame(burst, n, CFG_CAN_ID_SLAVE_BASE + s);
        SIM_CHECK(f != NULL, "missing slave frame %d", s);
        if (f)
        {
            float vseg = 0;
            for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) vseg += 3.70f + s * 0.004f + c * 0.001f;
            SIM_CHECK(fabsf(le16(&f->data[0]) / 100.0f - vseg) < 0.05f, "slave %d voltage: CAN %.2f V, actual %.2f V",
                      s, le16(&f->data[0]) / 100.0f, vseg);
            SIM_CHECK(fabsf(le16(&f->data[2]) / 10.0f - (35.0f + s)) < 0.5f, "slave %d die temp: CAN %.1f C", s, le16(&f->data[2]) / 10.0f);
            SIM_CHECK(f->data[7] == 0, "slave %d flags 0x%02X on a healthy pack", s, f->data[7]);
        }
    }

    const sim_can_frame* pack = find_frame(burst, n, CFG_CAN_ID_PACK);
    if (pack)
    {
        SIM_CHECK(fabsf(le16(&pack->data[2]) / 10.0f - 123.4f) < 0.15f, "pack current: CAN %.1f A", le16(&pack->data[2]) / 10.0f);
        SIM_CHECK(fabsf(le16(&pack->data[0]) / 10.0f - pack_sum()) < 0.2f, "pack U1: CAN %.1f V, actual %.1f V",
                  le16(&pack->data[0]) / 10.0f, pack_sum());
        SIM_CHECK(fabsf(le16(&pack->data[4]) / 10.0f - pack_sum()) < 0.2f, "sum of cells: CAN %.1f V", le16(&pack->data[4]) / 10.0f);
        SIM_CHECK(pack->data[6] == 0, "pack flags 0x%02X", pack->data[6]);
    }
    const sim_can_frame* status = find_frame(burst, n, CFG_CAN_ID_STATUS);
    if (status)
    {
        SIM_CHECK(status->data[0] == 0, "status: fault flags 0x%02X", status->data[0]);
        SIM_CHECK((status->data[1] & 0x03) == 0x02, "status: fault output / ready bits 0x%02X", status->data[1]);
        SIM_CHECK(((status->data[1] >> 5) & 1) == CFG_LOCAL_OVERRIDES, "status: bench build bit %d, overrides %d",
                  (status->data[1] >> 5) & 1, CFG_LOCAL_OVERRIDES);
        SIM_CHECK(status->data[6] == PRECHARGE_IDLE, "status: precharge state %d", status->data[6]);
        SIM_CHECK(le16(&status->data[2]) < 200, "status: measurement age %d ms", le16(&status->data[2]));
    }
    const sim_can_frame* summary = find_frame(burst, n, CFG_CAN_ID_SUMMARY);
    if (summary)
    {
        SIM_CHECK(abs(le16(&summary->data[0]) - 3700) <= 2, "summary min cell %d mV", le16(&summary->data[0]));
        float vmax = 3.70f + (CFG_NUM_SLAVES - 1) * 0.004f + (CFG_CELLS_PER_SLAVE - 1) * 0.001f;
        SIM_CHECK(abs(le16(&summary->data[2]) - (int)lroundf(vmax * 1000)) <= 2, "summary max cell %d mV", le16(&summary->data[2]));
    }
    const sim_can_frame* energy = find_frame(burst, n, CFG_CAN_ID_ENERGY);
    SIM_CHECK(energy != NULL, "no energy frame");
    if (energy)
    {
        float meanCell = 0;
        for (int s = 0; s < CFG_NUM_SLAVES; s++)
            for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) meanCell += 3.70f + s * 0.004f + c * 0.001f;
        meanCell /= CFG_TOTAL_CELLS;
        float soc = BMS_SoC_FromCellVoltage(meanCell);
        float power = le16(&energy->data[0]) * CFG_CAN_POWER_W_PER_BIT;
        float expectPower = 123.4f * pack_sum();
        SIM_CHECK(fabsf(power - expectPower) < 0.002f * expectPower + 20.0f, "energy frame power %.0f W, expected %.0f W", power, expectPower);
        float remaining = (uint16_t)le16(&energy->data[2]);
        float used = le16(&energy->data[6]);
        float socLive = (soc / 100.0f * CFG_PACK_ENERGY_WH - used) / CFG_PACK_ENERGY_WH * 100.0f;
        SIM_CHECK(abs((uint16_t)le16(&energy->data[4]) - (int)lroundf(socLive * 10)) <= 3, "energy frame SoC %d (0.1 %%), expected %.1f %%",
                  (uint16_t)le16(&energy->data[4]), socLive);
        SIM_CHECK(used > 0, "no energy counted at 123 A discharge");
        SIM_CHECK(fabsf(remaining + used - soc / 100.0f * CFG_PACK_ENERGY_WH) < 10.0f,
                  "energy frame remaining %.0f Wh + used %.0f Wh vs initial %.0f Wh", remaining, used, soc / 100.0f * CFG_PACK_ENERGY_WH);
    }
}

// A limit violation seen in a single measurement must not trip the AMS
static int  glitch_count = 0;
static bool glitch_armed = false;
static uint64_t glitch_sadc = 0;

static void ev_glitch_poll(void* arg)
{
    (void)arg;
    if (glitch_armed && sim_adbms_last_sadc_us(SLAVE(7), 0) != glitch_sadc)
    {
        set_cell(SLAVE(7), 3, 3.80f);              // Gone again after one conversion
        glitch_armed = false;
    }
    if (sim_now_us() < SIM_S(40)) sim_schedule(sim_now_us() + 1000, ev_glitch_poll, NULL);
}

static void ev_glitch(void* arg)
{
    (void)arg;
    glitch_sadc = sim_adbms_last_sadc_us(SLAVE(7), 0);
    set_cell(SLAVE(7), 3, CFG_CELL_OV_V + 0.05f);
    glitch_armed = true;
    glitch_count++;
}

static void test_glitch_immunity(void)
{
    board_init(true);
    for (int i = 0; i < 12; i++) sim_schedule(SIM_MS(4000) + i * SIM_MS(2900) + i * 9000, ev_glitch, NULL);
    sim_schedule(SIM_MS(3000), ev_glitch_poll, NULL);
    sim_run_firmware(SIM_S(40));
    common_checks(true);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(40)) == 0,
              "%d single-sample glitches caused %d faults", glitch_count, fault_asserts_between(first_release(), SIM_S(40)));
}

static void ev_current_normal(void* arg)
{
    (void)arg;
    res_current();
}

static void ev_current_spike(void* arg)
{
    (void)arg;
    sim_ivt_set_current(ivt_reported(CFG_MAX_DISCHARGE_CURRENT_A + 50.0f));
    sim_schedule(sim_now_us() + 30000, ev_current_normal, NULL);
}

static void test_current_spike_immunity(void)
{
    board_init(true);
    for (int i = 0; i < 10; i++) sim_schedule(SIM_MS(4000) + i * SIM_MS(1500) + i * 3000, ev_current_spike, NULL);
    sim_run_firmware(SIM_S(20));
    common_checks(true);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(20)) == 0, "30 ms current spikes caused a fault");
}

// Occasional corrupted isoSPI reads (noise) must not trip the AMS
static void ev_corrupt(void* arg)
{
    (void)arg;
    sim_adbms_corrupt_reads((int)(sim_now_us() / 1000) % CFG_NUM_SLAVES, 1);
}

static void test_pec_noise(void)
{
    board_init(true);
    for (int i = 0; i < 20; i++) sim_schedule(SIM_MS(4000) + i * SIM_MS(1500) + i * 7000, ev_corrupt, NULL);
    sim_run_firmware(SIM_S(36));
    common_checks(true);
    int pec = sim_uart_count("PEC ERROR");
    sim_note("%d PEC errors logged", pec);
    SIM_CHECK(pec >= 15, "only %d PEC errors seen, corruption not reaching the firmware", pec);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(36)) == 0,
              "single corrupted reads caused %d faults", fault_asserts_between(first_release(), SIM_S(36)));
}

static void ev_unbalance(void* arg)
{
    (void)arg;
    set_cell(SLAVE(2), 4, 3.85f);                                  // One cell high
    for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) set_cell(SLAVE(4), c, 3.85f);   // Whole slave high
}

static void ev_check_balancing(void* arg)
{
    (void)arg;
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++)
        {
            int ch = cell_channel[c] - 1;
            uint8_t pwm = sim_adbms_pwm(s, ch);
            if (s == 2 && c == 4)
            {
                SIM_CHECK(pwm == 15, "slave 2 cell 4: PWM %d, expected 15 (one cell, under the heat budget)", pwm);
            }
            else if (s == 4)
            {
                float p_full = CFG_CELLS_PER_SLAVE * 3.85f * 3.85f / CFG_BALANCE_RESISTOR_OHM;
                int expect = (int)(15 * CFG_BALANCE_MAX_POWER_PER_SLAVE_W / p_full);
                SIM_CHECK(pwm == expect, "slave 4 cell %d: PWM %d, expected %d (heat budget)", c, pwm, expect);
                float power = 0;
                for (int k = 0; k < CFG_CELLS_PER_SLAVE; k++)
                    power += sim_adbms_pwm(4, cell_channel[k] - 1) / 15.0f * 3.85f * 3.85f / CFG_BALANCE_RESISTOR_OHM;
                SIM_CHECK(power <= CFG_BALANCE_MAX_POWER_PER_SLAVE_W + 0.01f, "slave 4 balances at %.2f W", power);
            }
            else
            {
                SIM_CHECK(pwm == 0, "slave %d cell %d: PWM %d, expected off", s, c, pwm);
            }
        }
        for (int ch = 0; ch < 16; ch++)
        {
            bool used = false;
            for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) used |= (cell_channel[c] - 1 == ch);
            if (!used) SIM_CHECK(sim_adbms_pwm(s, ch) == 0, "slave %d: unused channel %d balancing", s, ch);
        }
    }
    SIM_CHECK(sim_adbms_dcto(2) == CFG_BALANCE_DCTO_MIN && sim_adbms_dcto(4) == CFG_BALANCE_DCTO_MIN, "DCTO not set while balancing");
    set_cell(SLAVE(10), 1, CFG_CELL_OV_V + 0.05f);                // Fault while balancing
}

static void ev_check_balancing_stopped(void* arg)
{
    (void)arg;
    SIM_CHECK(sim_fault_active(), "no fault for the over-voltage during balancing");
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        for (int ch = 0; ch < 16; ch++)
            SIM_CHECK(sim_adbms_pwm(s, ch) == 0, "slave %d channel %d still balancing during a fault", s, ch);
        SIM_CHECK(sim_adbms_dcto(s) == 0, "slave %d DCTO still set during a fault", s);
    }
}

static void test_balancing(void)
{
    board_init(true);
    sim_schedule(SIM_S(4), ev_unbalance, NULL);
    sim_schedule(SIM_S(5), ev_check_balancing, NULL);
    sim_schedule(SIM_S(5.5), ev_check_balancing_stopped, NULL);
    sim_run_firmware(SIM_S(6));
    common_checks(true);
}

// A cell at the balancing threshold must not switch on and off every cycle: once balancing it carries on until
// it's CFG_BALANCE_HYSTERESIS_V below the threshold
static int hyst_pwm_on = -1, hyst_pwm_mid = -1, hyst_pwm_off = -1;
static void ev_hyst_set(void* arg) { set_cell(SLAVE(2), 4, 3.80f + *(const float*)arg); }
static void ev_hyst_read(void* arg) { *(int*)arg = sim_adbms_pwm(SLAVE(2), cell_channel[4] - 1); }

static void test_balancing_hysteresis(void)
{
    static const float above = CFG_BALANCE_THRESHOLD_V + 0.005f;
    static const float within = CFG_BALANCE_THRESHOLD_V - CFG_BALANCE_HYSTERESIS_V / 2;
    static const float below = CFG_BALANCE_THRESHOLD_V - CFG_BALANCE_HYSTERESIS_V - 0.005f;
    board_init(true);
    sim_schedule(SIM_S(4), ev_hyst_set, (void*)&above);
    sim_schedule(SIM_S(5), ev_hyst_read, &hyst_pwm_on);
    sim_schedule(SIM_S(5.1), ev_hyst_set, (void*)&within);
    sim_schedule(SIM_S(6), ev_hyst_read, &hyst_pwm_mid);
    sim_schedule(SIM_S(6.1), ev_hyst_set, (void*)&below);
    sim_schedule(SIM_S(7), ev_hyst_read, &hyst_pwm_off);
    sim_run_firmware(SIM_S(7.2));
    common_checks(true);
    SIM_CHECK(hyst_pwm_on > 0, "cell %.0f mV above the others not balancing", above * 1000);
    SIM_CHECK(hyst_pwm_mid > 0, "balancing stopped %.0f mV above the others (hysteresis down to %.0f mV)",
              within * 1000, (CFG_BALANCE_THRESHOLD_V - CFG_BALANCE_HYSTERESIS_V) * 1000);
    SIM_CHECK(hyst_pwm_off == 0, "still balancing %.0f mV above the others", below * 1000);
}

static void ev_led_1(void* arg) { (void)arg; BMS_SetSlaveLeds(3, true, false); }
static void ev_led_2(void* arg)
{
    (void)arg;
    uint16_t red = (uint16_t)(1U << (CFG_SLAVE_LED_RED_GPIO - 1));
    uint16_t green = (uint16_t)(1U << (CFG_SLAVE_LED_GREEN_GPIO - 1));
    SIM_CHECK(sim_adbms_gpo(3) == (0x3FF & ~red), "slave 3 GPO 0x%03X with the red LED on", sim_adbms_gpo(3));
    SIM_CHECK(sim_adbms_gpo(2) == 0x3FF, "slave 2 GPO 0x%03X changed", sim_adbms_gpo(2));
    BMS_SetSlaveLeds(3, false, true);
    (void)green;
}
static void ev_led_3(void* arg)
{
    (void)arg;
    uint16_t green = (uint16_t)(1U << (CFG_SLAVE_LED_GREEN_GPIO - 1));
    SIM_CHECK(sim_adbms_gpo(3) == (0x3FF & ~green), "slave 3 GPO 0x%03X with the green LED on", sim_adbms_gpo(3));
}

static void test_leds(void)
{
    board_init(true);
    sim_schedule(SIM_S(3), ev_led_1, NULL);
    sim_schedule(SIM_S(3.5), ev_led_2, NULL);
    sim_schedule(SIM_S(4), ev_led_3, NULL);
    sim_run_firmware(SIM_S(5));
    common_checks(true);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(5)) == 0, "LED writes disturbed the measurements");
}

// Charging: charge temperature window, SDC and button
static void ev_charger_status(void* arg)
{
    (void)arg;
    uint8_t data[8]={3,0,0,0,0,0,0,0}; /* Current gateway ACTIVE, no faults. */
    uint8_t measurements[8]={0,0,0xA0,0x0F,50,0,0xA0,0x0F};
    sim_can_node_send(3,CHARGER_MEAS_CAN_ID,true,measurements,8);
    sim_can_node_send(3, CHARGER_STATUS_CAN_ID, true, data, 8);
    sim_schedule(sim_now_us() + 100000, ev_charger_status, NULL);
}
static void ev_press(void* arg)   { (void)arg; sim_gpio_set_input(CFG_PIN_CHARGER_BTN_PORT, CFG_PIN_CHARGER_BTN_PIN, !CFG_CHARGER_BTN_ACTIVE_LOW); }
static void ev_release(void* arg) { (void)arg; sim_gpio_set_input(CFG_PIN_CHARGER_BTN_PORT, CFG_PIN_CHARGER_BTN_PIN, CFG_CHARGER_BTN_ACTIVE_LOW); }
static void ev_warm(void* arg)    { (void)arg; set_temp(SLAVE(6), 2, CFG_CELL_CHARGE_TEMP_MAX_C + 1.0f); }
static uint64_t charge_on_time = 0;
static void ev_check_not_faulted_warm(void* arg)
{
    (void)arg;
    SIM_CHECK(!sim_fault_active(), "fault at %.0f C while not charging (discharge window)", CFG_CELL_CHARGE_TEMP_MAX_C + 1.0f);
    set_temp(SLAVE(6), 2, 25.0f);
}
static void ev_check_charging(void* arg)
{
    (void)arg;
    SIM_CHECK(BMS_IsCharging(), "charging not enabled by the button");
    charge_on_time = sim_now_us();
    set_temp(SLAVE(6), 2, CFG_CELL_CHARGE_TEMP_MAX_C + 1.0f);
}
static void ev_check_charge_fault(void* arg)
{
    (void)arg;
    uint64_t t = first_fault_after(charge_on_time);
    SIM_CHECK(t != UINT64_MAX && t - charge_on_time <= FS_LIMIT_US, "no fault within 500 ms for %.0f C while charging",
              CFG_CELL_CHARGE_TEMP_MAX_C + 1.0f);
    SIM_CHECK(!BMS_IsCharging(), "charging still enabled during a fault");
    set_temp(SLAVE(6), 2, 25.0f);
}
static void ev_check_charging_again(void* arg)
{
    (void)arg;
    SIM_CHECK(BMS_IsCharging(), "charging not re-enabled after the fault cleared");
    sim_gpio_set_input(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN, !CFG_SDC_IN_ACTIVE_HIGH);   // SDC opens
}
static void ev_check_sdc_stop(void* arg)
{
    (void)arg;
    SIM_CHECK(!BMS_IsCharging(), "charging not stopped when the SDC opened");
}

static void test_charging(void)
{
    board_init(true);
    sim_schedule(SIM_S(1), ev_charger_status, NULL);
    sim_schedule(SIM_S(4.0), ev_warm, NULL);
    sim_schedule(SIM_S(5.0), ev_check_not_faulted_warm, NULL);
    sim_schedule(SIM_S(5.5), ev_press, NULL);
    sim_schedule(SIM_S(5.7), ev_release, NULL);
    sim_schedule(SIM_S(6.0), ev_check_charging, NULL);
    sim_schedule(SIM_S(6.7), ev_check_charge_fault, NULL);
    sim_schedule(SIM_S(9.5), ev_press, NULL);
    sim_schedule(SIM_S(9.7), ev_release, NULL);
    sim_schedule(SIM_S(10.0), ev_check_charging_again, NULL);
    sim_schedule(SIM_S(10.2), ev_check_sdc_stop, NULL);
    sim_run_firmware(SIM_S(11));
    common_checks(true);
}

static void test_can_no_ack(void)
{
    board_init(true);
    sim_can_set_ack(1, false);          // Nothing else on the car bus
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "AMS faulted because CAN1 has no listener");
    SIM_CHECK(sim_uart_count("Overwritten") == 1, "lost CAN1 bursts logged %d times (once expected, not every period)",
              sim_uart_count("Overwritten"));
}

static void ev_bus_off(void* arg) { (void)arg; sim_can_force_bus_off(1); }
static uint64_t frames_before_recovery = 0;
static void ev_check_bus_off(void* arg)
{
    (void)arg;
    SIM_CHECK(!(sim_FDCAN1.CCCR & FDCAN_CCCR_INIT), "CAN1 still in bus-off 50 ms later");
    frames_before_recovery = (uint64_t)sim_can_log_count();
}

static void test_can_bus_off(void)
{
    board_init(true);
    sim_schedule(SIM_S(4), ev_bus_off, NULL);
    sim_schedule(SIM_S(4.05), ev_check_bus_off, NULL);
    sim_run_firmware(SIM_S(6));
    common_checks(true);
    int after = 0;
    for (int i = (int)frames_before_recovery; i < sim_can_log_count(); i++)
        if (sim_can_log(i)->bus == 1 && sim_can_log(i)->from_mcu) after++;
    SIM_CHECK(after > 100, "only %d CAN1 frames sent after bus-off recovery", after);
}

static void ev_ivt_power_cycle(void* arg) { (void)arg; sim_ivt_power_cycle(); }

// The IVT back on the BMS's configuration: I, U1, U2 and Wh cyclic, the rest off
static void check_ivt_configured(const char* when)
{
    static const bool on[8] = { true, true, true, false, false, false, false, true };
    for (int i = 0; i < 8; i++)
    {
        SIM_CHECK(sim_ivt_channel_cyclic(i) == on[i], "%s: IVT result %d %s", when, i, on[i] ? "off" : "still on");
    }
    float expect = 1000.0f / CFG_IVT_CURRENT_PERIOD_MS + 1000.0f / CFG_IVT_U1_PERIOD_MS + 1000.0f / CFG_IVT_U2_PERIOD_MS +
                   1000.0f / CFG_IVT_ENERGY_PERIOD_MS;
    SIM_CHECK(fabsf(sim_ivt_messages_per_s() - expect) < 1.0f, "%s: IVT sends %.0f messages/s, expected %.0f", when,
              sim_ivt_messages_per_s(), expect);
}

// The IVT's supply off for 3 s while another node (the USB-CAN analyser, the inverter) keeps acknowledging the
// BMS's frames, so its configuration attempts look successful; then the IVT boots with its stored configuration
static void ev_ivt_off(void* arg)
{
    (void)arg;
    sim_ivt_set_connected(false);
    sim_can_set_ack(2, true);
}

static void ev_ivt_back(void* arg)
{
    (void)arg;
    sim_ivt_power_cycle();
    sim_ivt_set_connected(true);
}

static void run_ivt_power_cycle(bool u1_default)
{
    board_init(u1_default);
    sim_schedule(SIM_S(5), ev_ivt_off, NULL);
    sim_schedule(SIM_S(8), ev_ivt_back, NULL);
    sim_run_firmware(SIM_S(15));
    common_checks(true);
    uint64_t t = first_fault_after(SIM_S(5));
    SIM_CHECK(t != UINT64_MAX && t - SIM_S(5) <= FS_LIMIT_US, "no fault within 500 ms of the IVT restarting");
    check_ivt_configured("after the restart");
    SIM_CHECK(!sim_fault_active(), "still faulted 7 s after the IVT came back (flags 0x%02lX)",
              (unsigned long)BMS_Safety_GetActiveFlags());
}

// The team's IVT comes back with all eight results on, so no data goes missing: the BMS must notice the
// configuration and set its own again
static void test_ivt_power_cycle(void) { run_ivt_power_cycle(true); }

// The datasheet's other default: U1 off, so U1 goes missing after the restart
static void test_ivt_power_cycle_u1_off(void) { run_ivt_power_cycle(false); }


/* ===================================================================================== */
/*  Power-on order and brief power faults                                                 */
/* ===================================================================================== */
/*
 * Whatever powers up first, the BMS ends up healthy without a reset. A brief loss of the IVT or of a slave
 * is ridden through without tripping; a longer one trips within the deadline from the last good data
 * (FS 2027 EV 5.8.10, T 11.9) and recovers by itself once the part is back.
 */

#define DEADLINE_US SIM_MS(CFG_FAULT_DEADLINE_MS + CFG_SAFETY_TICK_MS)

static void ev_slaves_off(void* arg) { (void)arg; sim_adbms_set_powered(SLAVE(0), false); }
static void ev_slaves_on(void* arg)  { (void)arg; sim_adbms_set_powered(SLAVE(0), true); }
static void ev_slave_off(void* arg)  { (void)arg; sim_adbms_set_powered(SLAVE(5), false); }
static void ev_slave_on(void* arg)   { (void)arg; sim_adbms_set_powered(SLAVE(5), true); }

static void check_healthy(const char* when)
{
    SIM_CHECK(!sim_fault_active(), "%s: still faulted (flags 0x%02lX)", when, (unsigned long)BMS_Safety_GetActiveFlags());
    check_ivt_configured(when);
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        SIM_CHECK(sim_adbms_refon(s), "%s: slave %d not configured (REFON off)", when, s + 1);
    }
}

// Slaves and IVT powered after the BMS, at the given times (0 = with the BMS)
static void run_power_on(double slaves_s, double ivt_s, const char* what)
{
    board_init(true);
    if (slaves_s > 0)
    {
        ev_slaves_off(NULL);
        sim_schedule(SIM_S(slaves_s), ev_slaves_on, NULL);
    }
    if (ivt_s > 0)
    {
        ev_ivt_off(NULL);
        sim_schedule(SIM_S(ivt_s), ev_ivt_back, NULL);
    }
    double last = slaves_s > ivt_s ? slaves_s : ivt_s;
    sim_run_firmware(SIM_S(last + 3.0));
    common_checks(true);
    check_healthy(what);
    uint64_t release = first_release();
    SIM_CHECK(release >= SIM_S(last), "%s: fault output released at %.2f s, before everything was up", what, release / 1e6);
    sim_note("%s: fault output released %.2f s after the last part came up", what, (release - SIM_S(last)) / 1e6);
}

static void test_power_on_slaves_late(void)       { run_power_on(3.0, 0, "slaves powered 3 s after the BMS"); }
static void test_power_on_ivt_late(void)          { run_power_on(0, 3.0, "IVT powered 3 s after the BMS"); }
static void test_power_on_slaves_then_ivt(void)   { run_power_on(2.0, 4.0, "slaves at 2 s, IVT at 4 s"); }
static void test_power_on_ivt_then_slaves(void)   { run_power_on(4.0, 2.0, "IVT at 2 s, slaves at 4 s"); }

// A dropout of the given length at 5 s; brief ones must not trip, long ones must trip by the deadline
static void run_dropout(sim_event_fn off, sim_event_fn on, double length_s, bool expect_trip, const char* what)
{
    sim_schedule(SIM_S(5), off, NULL);
    sim_schedule(SIM_S(5 + length_s), on, NULL);
    sim_run_firmware(SIM_S(5 + length_s + 3.0));
    common_checks(true);
    SIM_CHECK(first_release() < SIM_S(5), "%s: not healthy before the dropout", what);
    uint64_t trip = first_fault_after(SIM_S(5));
    if (expect_trip)
    {
        SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= DEADLINE_US, "%s: no fault within %.0f ms", what,
                  DEADLINE_US / 1000.0);
        if (trip != UINT64_MAX) sim_note("%s: fault %.0f ms after the dropout started", what, (trip - SIM_S(5)) / 1000.0);
    }
    else
    {
        SIM_CHECK(trip == UINT64_MAX, "%s: AMS tripped %.0f ms after it started (flags 0x%02lX)", what,
                  (trip - SIM_S(5)) / 1000.0, (unsigned long)BMS_Safety_GetActiveFlags());
    }
    check_healthy(what);
}

static void test_dropout_ivt_brief(void)
{
    board_init(true);
    sim_ivt_set_startup_ms(200);            // ~190 ms measured on the team's IVT (restart command)
    run_dropout(ev_ivt_off, ev_ivt_back, 0.05, false, "IVT supply lost for 50 ms");
    // It drops them, so they'd only be noise: the BMS waits for its alive message
    SIM_CHECK(sim_ivt_boot_commands() == 0, "%d commands sent to the IVT while it was starting up",
              sim_ivt_boot_commands());
}

static void test_dropout_ivt_long(void)
{
    board_init(true);                       // Datasheet start-up time (400 ms)
    run_dropout(ev_ivt_off, ev_ivt_back, 1.0, true, "IVT supply lost for 1 s");
}

static void test_dropout_slave_brief(void)
{
    board_init(true);
    run_dropout(ev_slave_off, ev_slave_on, 0.15, false, "slave supply lost for 150 ms");
}

static void test_dropout_slave_long(void)
{
    board_init(true);
    run_dropout(ev_slave_off, ev_slave_on, 1.0, true, "slave supply lost for 1 s");
}

static void ev_chain_break(void* arg)  { (void)arg; sim_adbms_set_broken_after(SIM_CHAIN_BROKEN_AT_MASTER); }
static void ev_chain_mend(void* arg)   { (void)arg; sim_adbms_set_broken_after(-1); }

static void test_dropout_chain_brief(void)
{
    board_init(true);
    run_dropout(ev_chain_break, ev_chain_mend, 0.15, false, "isoSPI link from the master lost for 150 ms");
}

// isoSPI chaos: random disturbances of random kind and length, one after another. Brief ones (10..150 ms, one or
// two corrupted reads, a lost command) must never trip the AMS; long ones (0.7..1.5 s) must trip by the deadline;
// after each the chain must come back on its own.
#ifndef CHAOS_SEED
#define CHAOS_SEED 4242
#endif
#define CHAOS_EPISODES 40

enum { CHAOS_BREAK_MASTER, CHAOS_BREAK_AFTER, CHAOS_SLAVE_POWER, CHAOS_SPI_ERROR, CHAOS_CORRUPT, CHAOS_LOST_CMD, CHAOS_KINDS };
static const char* const chaos_names[CHAOS_KINDS] = {
    "link from the master broken", "chain broken after a slave", "slave unpowered", "SPI errors",
    "corrupted reads", "lost conversion command",
};

typedef struct { int kind, slave, reads; bool long_one, healthy_at_start; uint64_t start, end; } chaos_episode;
static chaos_episode chaos[CHAOS_EPISODES];
static uint32_t chaos_rng = CHAOS_SEED;      // Other seeds: EXTRA_CFLAGS=-DCHAOS_SEED=n
static uint32_t chaos_rand(uint32_t n) { chaos_rng = chaos_rng * 1103515245u + 12345u; return (chaos_rng >> 16) % n; }

static void ev_chaos_end(void* arg);

// Each episode schedules its end, and each end the next episode (the simulator holds only so many events)
static void ev_chaos_start(void* arg)
{
    chaos_episode* e = arg;
    e->healthy_at_start = !sim_fault_active();
    sim_schedule(e->end, ev_chaos_end, e);
    switch (e->kind)
    {
    case CHAOS_BREAK_MASTER: sim_adbms_set_broken_after(SIM_CHAIN_BROKEN_AT_MASTER); break;
    case CHAOS_BREAK_AFTER:  sim_adbms_set_broken_after(e->slave); break;
    case CHAOS_SLAVE_POWER:  sim_adbms_set_powered(e->slave, false); break;
    case CHAOS_SPI_ERROR:    sim_adbms_set_spi_error(true); break;
    case CHAOS_CORRUPT:      sim_adbms_corrupt_reads(e->slave, e->reads); break;
    default:                 sim_adbms_lose_next_conversion(e->slave); break;
    }
}

static void ev_chaos_end(void* arg)
{
    chaos_episode* e = arg;
    switch (e->kind)
    {
    case CHAOS_BREAK_MASTER:
    case CHAOS_BREAK_AFTER:  sim_adbms_set_broken_after(-1); break;
    case CHAOS_SLAVE_POWER:  sim_adbms_set_powered(e->slave, true); break;
    case CHAOS_SPI_ERROR:    sim_adbms_set_spi_error(false); break;
    case CHAOS_CORRUPT:      sim_adbms_corrupt_reads(e->slave, 0); break;
    default:                 break;
    }
    if (e + 1 < &chaos[CHAOS_EPISODES]) sim_schedule(e[1].start, ev_chaos_start, e + 1);
}

static void test_isospi_chaos(void)
{
    board_init(true);
    uint64_t t = SIM_S(4);
    int longs = 0;
    for (int i = 0; i < CHAOS_EPISODES; i++)
    {
        chaos_episode* e = &chaos[i];
        e->kind = (int)chaos_rand(CHAOS_KINDS);
        e->long_one = e->kind != CHAOS_LOST_CMD && chaos_rand(3) == 0;
        // A break after the last slave breaks nothing
        e->slave = (int)chaos_rand(e->kind == CHAOS_BREAK_AFTER && CFG_NUM_SLAVES > 1 ? CFG_NUM_SLAVES - 1 : CFG_NUM_SLAVES);
        if (e->kind == CHAOS_BREAK_AFTER && CFG_NUM_SLAVES == 1) e->kind = CHAOS_BREAK_MASTER;
        e->reads = e->long_one ? 1000000 : 1 + (int)chaos_rand(2);
        e->start = t;
        e->end = t + (e->long_one ? SIM_MS(700 + chaos_rand(800)) : SIM_MS(10 + chaos_rand(140)));
        t = e->end + (e->long_one ? SIM_S(4) : SIM_MS(1500));
        longs += e->long_one;
    }
    sim_schedule(chaos[0].start, ev_chaos_start, &chaos[0]);
    sim_run_firmware(t + SIM_S(1));
    common_checks(true);
    check_healthy("after the chaos");

    int short_trips = 0, long_misses = 0;
    for (int i = 0; i < CHAOS_EPISODES; i++)
    {
        const chaos_episode* e = &chaos[i];
        SIM_CHECK(e->healthy_at_start, "episode %d (%s) started with the AMS still faulted", i + 1, chaos_names[e->kind]);
        if (e->long_one)
        {
            uint64_t trip = first_fault_after(e->start);
            bool ok = trip != UINT64_MAX && trip - e->start <= DEADLINE_US;
            long_misses += !ok;
            SIM_CHECK(ok, "episode %d: %s for %.0f ms, no fault within %.0f ms", i + 1, chaos_names[e->kind],
                      (e->end - e->start) / 1000.0, DEADLINE_US / 1000.0);
        }
        else
        {
            int trips = fault_asserts_between(e->start, e->end + SIM_S(1));
            short_trips += trips != 0;
            SIM_CHECK(trips == 0, "episode %d: %s (%.0f ms, slave %d, %d reads) tripped the AMS", i + 1,
                      chaos_names[e->kind], (e->end - e->start) / 1000.0, e->slave + 1, e->reads);
        }
    }
    sim_note("%d episodes over %.0f s: %d long (all faulted: %s), %d brief (none faulted: %s)", CHAOS_EPISODES,
             t / 1e6, longs, long_misses ? "no" : "yes", CHAOS_EPISODES - longs, short_trips ? "no" : "yes");
}

// A slave resetting between two cycles (a brief dip of its supply, like an SRST): asleep and at its reset values,
// but it answers again at once. Found on the bench (2026-10-13): the BMS carried on without noticing, so the
// slave's continuous C-ADC, started only at initialisation, stayed off (its readings shifted slightly: AUX
// temperatures 0.4 C, VPV 30 mV). It must notice, say which slave, and restore it, without faulting.
static void ev_slave_blip(void* arg)
{
    (void)arg;
    sim_adbms_set_powered(SLAVE(3), false);
    sim_adbms_set_powered(SLAVE(3), true);
}

static void test_slave_reset_restored(void)
{
    board_init(true);
    sim_schedule(SIM_S(5.07), ev_slave_blip, NULL);
    sim_schedule(SIM_S(6.07), ev_slave_blip, NULL);         // Second one in the middle of a cycle, most likely
    sim_run_firmware(SIM_S(7.5));
    common_checks(true);
    char msg[48];
    snprintf(msg, sizeof(msg), "Slave %d was reset", SLAVE(3) + 1);
    sim_note("\"%s\" logged %d times", msg, sim_uart_count(msg));
    SIM_CHECK(sim_uart_count(msg) == 2, "slave reset logged %d times, expected 2", sim_uart_count(msg));
    SIM_CHECK(sim_uart_count("was reset") == 2, "other slaves reported as reset");
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
    {
        SIM_CHECK(sim_adbms_refon(s) && sim_adbms_cadc_running(s), "slave %d after the resets: REFON %d, C-ADC %s", s + 1,
                  sim_adbms_refon(s), sim_adbms_cadc_running(s) ? "running" : "stopped");
    }
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(7.5)) == 0, "a slave reset caused an AMS fault");
}

/* ===================================================================================== */
/*  Intermittent limit violations                                                         */
/* ===================================================================================== */
/*
 * A value that keeps crossing a limit (noise around it, pulses over it) must fault like a steady one, only a
 * little later, while isolated glitches still must not. Found on the bench (2026-10-12): one sample under the
 * limit used to restart the debounce, so 2/3 of the samples over the current limit, or 150 ms pulses with
 * 20 ms gaps, never tripped.
 */

static float    flap_over_a, flap_under_a;
static uint64_t flap_over_us, flap_under_us, flap_end_us;

static void ev_current_flap(void* arg)
{
    bool over = arg != NULL;
    if (sim_now_us() >= flap_end_us)
    {
        res_current();
        return;
    }
    sim_ivt_set_current(ivt_reported(over ? flap_over_a : flap_under_a));
    sim_schedule(sim_now_us() + (over ? flap_over_us : flap_under_us), ev_current_flap, over ? NULL : (void*)1);
}

// Current alternating between over_a for over_ms and under_a for under_ms, from 5 s for 3 s
static void run_current_flap(float over_a, int over_ms, float under_a, int under_ms, bool expect_trip, const char* what)
{
    board_init(true);
    flap_over_a = over_a;
    flap_under_a = under_a;
    flap_over_us = SIM_MS(over_ms);
    flap_under_us = SIM_MS(under_ms);
    flap_end_us = SIM_S(8);
    sim_schedule(SIM_S(5), ev_current_flap, (void*)1);
    sim_run_firmware(SIM_S(9));
    common_checks(true);
    uint64_t trip = first_fault_after(SIM_S(5));
    if (expect_trip)
    {
        SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= FS_LIMIT_US, "%s: no fault within %d ms", what,
                  CFG_FS_MAX_REACTION_MS);
        if (trip != UINT64_MAX) sim_note("%s: fault %.0f ms after it started", what, (trip - SIM_S(5)) / 1000.0);
    }
    else
    {
        SIM_CHECK(trip == UINT64_MAX || trip >= flap_end_us, "%s: fault %.0f ms after it started", what,
                  (trip - SIM_S(5)) / 1000.0);
    }
}

static void test_intermittent_overcurrent(void)
{
    run_current_flap(CFG_MAX_DISCHARGE_CURRENT_A + 5.0f, 8, CFG_MAX_DISCHARGE_CURRENT_A - 5.0f, 4, true,
                     "current 5 A over the limit for 8 ms, 5 A under for 4 ms");
}

static void test_overcurrent_pulses(void)
{
    run_current_flap(CFG_MAX_DISCHARGE_CURRENT_A * 1.5f, 150, CFG_MAX_DISCHARGE_CURRENT_A * 0.5f, 20, true,
                     "150 ms pulses at 1.5 x the limit, 20 ms gaps");
}

static void test_current_glitches_periodic(void)
{
    run_current_flap(CFG_MAX_DISCHARGE_CURRENT_A * 2.0f, 4, 20.0f, 96, false, "one 4 ms sample at 2 x the limit every 100 ms");
}

// A cell over the limit in every other measurement
static uint64_t cell_flap_sadc = 0;
static bool     cell_flap_over = false;

static void ev_cell_flap_poll(void* arg)
{
    (void)arg;
    if (sim_adbms_last_sadc_us(SLAVE(7), 0) != cell_flap_sadc)
    {
        cell_flap_sadc = sim_adbms_last_sadc_us(SLAVE(7), 0);
        cell_flap_over = !cell_flap_over;
        set_cell(SLAVE(7), 3, cell_flap_over ? CFG_CELL_OV_V + 0.02f : CFG_CELL_OV_V - 0.02f);
    }
    if (sim_now_us() < SIM_S(8)) sim_schedule(sim_now_us() + 500, ev_cell_flap_poll, NULL);
    else set_cell(SLAVE(7), 3, 3.80f);
}

static void test_intermittent_cell_overvoltage(void)
{
    board_init(true);
    sim_schedule(SIM_S(5), ev_cell_flap_poll, NULL);
    sim_run_firmware(SIM_S(9));
    common_checks(true);
    uint64_t trip = first_fault_after(SIM_S(5));
    SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= SIM_MS(1000), "cell over the limit in every other sample: no fault within 1 s");
    if (trip != UINT64_MAX) sim_note("cell over the limit in every other sample: fault %.0f ms after it started", (trip - SIM_S(5)) / 1000.0);
}


/* ===================================================================================== */
/*  CAN2 robustness                                                                       */
/* ===================================================================================== */
/*
 * What the bench (USB-CAN adapter on the IVT bus) threw at the BMS: floods, junk on the IVT's IDs, spurious alive
 * messages, commands to the IVT from another node, a configuration cut off part way.
 */

static bool ivt_reading_matches(float amps)
{
    IvtReading r = BMS_IVT_GetReading();
    return r.currentFresh && fabsf(r.current_A - amps) < 0.01f;
}

static void test_can2_flood_low_priority(void)
{
    board_init(true);
    sim_can_flood(2, 0x7FF, false, 8, SIM_S(8));             // From the start: the whole bus, lowest priority
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    sim_note("%d flood frames, CAN2 load %.0f %%", sim_can_flood_sent(2), sim_can_busy_us(2) / 80000.0);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "flood of low-priority frames caused a fault");
    SIM_CHECK(sim_ivt_config_commands() == 8, "IVT reconfigured during the flood (%d config commands)", sim_ivt_config_commands());
    SIM_CHECK(ivt_reading_matches(20.0f), "IVT current not arriving during the flood");
    SIM_CHECK(sim_can_rx_lost(2) == 0, "%d CAN2 frames lost", sim_can_rx_lost(2));
}

static void ev_high_flood(void* arg) { (void)arg; sim_can_flood(2, 0x001, false, 8, SIM_S(7)); }

static void test_can2_flood_high_priority(void)
{
    board_init(true);
    sim_schedule(SIM_S(5), ev_high_flood, NULL);            // 2 s of a babbling node that wins every arbitration
    sim_run_firmware(SIM_S(11));
    common_checks(true);
    uint64_t trip = first_fault_after(SIM_S(5));
    SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= SIM_MS(CFG_FAULT_DEADLINE_MS + CFG_SAFETY_TICK_MS),
              "IVT starved by the flood: no fault within the deadline");
    if (trip != UINT64_MAX) sim_note("IVT starved: fault %.0f ms after the flood started", (trip - SIM_S(5)) / 1000.0);
    SIM_CHECK(!sim_fault_active(), "still faulted 4 s after the flood (flags 0x%02lX)", (unsigned long)BMS_Safety_GetActiveFlags());
    SIM_CHECK(sim_ivt_running() && ivt_reading_matches(20.0f), "IVT not back after the flood");
    check_ivt_configured("after the flood");
}

static int junk_sent = 0;

static void ev_junk(void* arg)
{
    (void)arg;
    static const uint8_t wrong_mux[6]  = { 0x05, 0x00, 0x7F, 0xFF, 0xFF, 0xFF };
    static const uint8_t short_i[5]    = { 0x00, 0x00, 0x7F, 0xFF, 0xFF };
    static const uint8_t ext_i[6]      = { 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF };
    static const uint8_t not_alive[8]  = { 0xBE, 0, 0, 0, 0, 0, 0, 0 };
    static const uint8_t short_alive[3] = { 0xBF, 0, 0 };
    static const uint8_t u2_on_u1[6]   = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
    switch (junk_sent % 7)
    {
    case 0: sim_can_node_send(2, 0x521, false, wrong_mux, 6); break;
    case 1: sim_can_node_send(2, 0x521, false, short_i, 5); break;
    case 2: sim_can_node_send_remote(2, 0x521, false, 6); break;
    case 3: sim_can_node_send(2, 0x521, true, ext_i, 6); break;
    case 4: sim_can_node_send(2, 0x511, false, not_alive, 8); break;
    case 5: sim_can_node_send(2, 0x511, false, short_alive, 3); break;
    default: sim_can_node_send(2, 0x522, false, u2_on_u1, 6); break;
    }
    junk_sent++;
    if (sim_now_us() < SIM_S(8)) sim_schedule(sim_now_us() + 1000, ev_junk, NULL);
}

static void test_can2_junk_frames(void)
{
    board_init(true);
    sim_schedule(SIM_S(5), ev_junk, NULL);
    sim_run_firmware(SIM_S(9.5));
    common_checks(true);
    sim_note("%d junk frames on the IVT's IDs", junk_sent);
    SIM_CHECK(first_fault_after(SIM_S(5))!=UINT64_MAX,"persistent invalid IVT packets were not faulted");
    SIM_CHECK(!sim_fault_active(),"IVT did not recover after junk ceased");
    SIM_CHECK(sim_ivt_config_commands() == 8, "junk frames made the BMS reconfigure the IVT (%d config commands)",
              sim_ivt_config_commands());
    SIM_CHECK(ivt_reading_matches(20.0f), "IVT reading disturbed by junk frames");
}

static void ev_spurious_alive(void* arg)
{
    (void)arg;
    static const uint8_t alive[8] = { 0xBF, 0x04, 0x11, 0x00, 0x99, 0x47, 0x0E, 0x00 };
    sim_can_node_send(2, 0x511, false, alive, 8);
}

// Energy counter values from one end of its range to the other (a corrupted frame, a reloaded counter): the jump
// is detected and counting restarts; SoC and energy stay sane. Every 1 ms so the IVT's own counter frames are
// overwritten before the main loop reads them. Run with SANITIZE=1 for the integer overflow.
static int32_t energy_extreme = INT32_MIN;
static void ev_energy_extreme(void* arg)
{
    (void)arg;
    uint8_t d[8] = { 0x07, 0x00 };          // Energy result, status OK
    uint32_t v = (uint32_t)energy_extreme;
    d[2] = (uint8_t)(v >> 24); d[3] = (uint8_t)(v >> 16); d[4] = (uint8_t)(v >> 8); d[5] = (uint8_t)v;
    sim_can_node_send(2, CFG_IVT_ID_RESULT_WH, false, d, 6);
    if (sim_now_us() < SIM_S(6.0)) sim_schedule(sim_now_us() + SIM_MS(1), ev_energy_extreme, NULL);
}
static void ev_energy_flip(void* arg) { (void)arg; energy_extreme = energy_extreme == INT32_MIN ? INT32_MAX : INT32_MIN; }

static void test_ivt_energy_extremes(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(4.0), ev_energy_extreme, NULL);
    for (int i = 1; i < 8; i++) sim_schedule(SIM_S(4.0) + i * SIM_MS(250), ev_energy_flip, NULL);
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    SocState s = BMS_SoC_Get();
    sim_note("%d jumps detected; SoC %.1f %%, %.1f Wh remaining, %.1f Wh used", sim_uart_count("Energy counter jumped"),
             s.soc_percent, s.remaining_Wh, s.used_Wh);
    IVTS_Snapshot raw;BMS_IVT_GetSnapshot(&raw);
    SIM_CHECK(raw.energy.duplicate_count+raw.energy.counter_gap_count>=6,"malformed counter sequence was not detected");
    SIM_CHECK(s.valid && s.soc_percent >= 0.0f && s.soc_percent <= 100.0f && fabsf(s.used_Wh) < 100.0f,
              "SoC %.1f %%, used %.1f Wh after the jumps", s.soc_percent, s.used_Wh);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "energy counter jumps caused an AMS fault");
}

static void test_ivt_spurious_alive(void)
{
    board_init(true);
    for (int i = 0; i < 20; i++) sim_schedule(SIM_S(5) + i * SIM_MS(50), ev_spurious_alive, NULL);
    sim_run_firmware(SIM_S(7));
    common_checks(true);
    SIM_CHECK(sim_ivt_config_commands() > 8, "Alive failed to invalidate/reconfigure the IVT "
              "(%d config commands)", sim_ivt_config_commands());
    SIM_CHECK(sim_uart_count("IVT restarted, configuring") >= 1, "spurious alive messages not logged");
    SIM_CHECK(BMS_IVT_GetReading().configurationVerified,"IVT configuration was not reverified after Alive");
}

// Another node stops the IVT: once out of the blue, and once right after the BMS has configured it (the worst
// case for its retry interval)
static const uint8_t ivt_stop_cmd[8] = { 0x34, 0x00, 0x01, 0, 0, 0, 0, 0 };
static bool     stop_after_run = false;
static uint64_t stray_stop_at[2];
static int      stray_stops = 0;

static void ev_stray_stop(void* arg)
{
    (void)arg;
    stray_stop_at[stray_stops++] = sim_now_us();
    sim_ivt_command(ivt_stop_cmd);
}

static uint64_t run_at[8];
static int      runs = 0;

static void stop_hook(const uint8_t* d)
{
    if (d[0] == 0x34 && d[1] == 0x01)                       // The BMS starts it
    {
        if (runs < 8) run_at[runs++] = sim_now_us();
        if (stop_after_run)
        {
            stop_after_run = false;
            sim_schedule(sim_now_us() + 1000, ev_stray_stop, NULL);
        }
    }
}

static uint64_t run_after(uint64_t t)
{
    for (int i = 0; i < runs; i++) if (run_at[i] > t) return run_at[i];
    return UINT64_MAX;
}

static void ev_arm_stop_after_run(void* arg) { (void)arg; stop_after_run = true; }

static void test_ivt_stray_stop(void)
{
    board_init(true);
    sim_ivt_set_command_hook(stop_hook);
    sim_schedule(SIM_S(5), ev_stray_stop, NULL);            // The BMS restarts it: then it's stopped again
    sim_schedule(SIM_S(5) + 1000, ev_arm_stop_after_run, NULL);
    sim_run_firmware(SIM_S(12));
    common_checks(true);
    SIM_CHECK(stray_stops == 2, "%d stray stops", stray_stops);
    for (int i = 0; i < stray_stops; i++)
    {
        // The wait, plus a measurement cycle it may have to wait out, plus the configuration (~40 ms)
        uint64_t limit = SIM_MS((i == 0 ? CFG_IVT_RESTART_WAIT_MS : CFG_IVT_RETRY_INTERVAL_MS) + CFG_MEAS_CYCLE_MAX_MS + 50);
        uint64_t back = run_after(stray_stop_at[i]);
        SIM_CHECK(back != UINT64_MAX && back - stray_stop_at[i] <= limit, "stray stop %d: IVT not restarted within %.0f ms",
                  i + 1, limit / 1000.0);
        if (back != UINT64_MAX) sim_note("stray stop %d: BMS restarted the IVT after %.0f ms", i + 1, (back - stray_stop_at[i]) / 1000.0);
    }
    // Out of the blue it's running again before its data counts as lost; right after a configuration it may not be
    SIM_CHECK(stray_stops < 2 || first_fault_after(SIM_S(5)) >= stray_stop_at[1], "stray stop 1 caused a fault");
    SIM_CHECK(!sim_fault_active(), "still faulted at the end (flags 0x%02lX)", (unsigned long)BMS_Safety_GetActiveFlags());
    check_ivt_configured("after the stray stops");
}

// Another node restarts the IVT: it comes back with a system error (as the team's unit does), which the BMS must
// fault on until the IVT is power cycled
static const uint8_t ivt_restart_cmd[8] = { 0x3F, 0, 0, 0, 0, 0, 0, 0 };
static void ev_restart_cmd(void* arg) { (void)arg; sim_ivt_command(ivt_restart_cmd); }

static void ev_check_restart(void* arg)
{
    (void)arg;
    uint64_t trip = first_fault_after(SIM_S(5));
    SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= FS_LIMIT_US, "IVT back with a system error: no fault");
    if (trip != UINT64_MAX) sim_note("restart command: fault %.0f ms later (IVT back with a system error)", (trip - SIM_S(5)) / 1000.0);
    SIM_CHECK(sim_fault_active() && (BMS_Safety_GetActiveFlags() & BMS_ERR_IVT), "not faulted while the IVT reports a system error");
    check_ivt_configured("after the restart command");
    SIM_CHECK(sim_uart_count("reset watchdog. Not cleared automatically") == 1, "watchdog reset not logged as kept");
    SIM_CHECK(sim_ivt_error_resets() == 0, "IVT errors cleared after a watchdog reset");
}

static void test_ivt_restart_command(void)
{
    board_init(true);
    sim_ivt_set_startup_ms(200);
    sim_schedule(SIM_S(5), ev_restart_cmd, NULL);
    sim_schedule(SIM_S(7.9), ev_check_restart, NULL);
    sim_schedule(SIM_S(8), ev_ivt_off, NULL);               // Power cycling the IVT clears it
    sim_schedule(SIM_S(8.5), ev_ivt_back, NULL);
    sim_run_firmware(SIM_S(12));
    common_checks(true);
    SIM_CHECK(!sim_fault_active(), "still faulted after the IVT was power cycled (flags 0x%02lX)",
              (unsigned long)BMS_Safety_GetActiveFlags());
}

// The BMS's configuration cut off part way (no acknowledgement for a few ms): the IVT must not be left stopped
static int  config_seen = 0;
static void ev_ack_back(void* arg) { (void)arg; sim_can_set_ack(2, true); }

static void cut_hook(const uint8_t* d)
{
    if ((d[0] & 0xF0) == 0x20 && ++config_seen == 8 + 3)    // Third result of the second configuration
    {
        sim_can_set_ack(2, false);
        sim_schedule(sim_now_us() + SIM_MS(8), ev_ack_back, NULL);
    }
}

static void ev_foreign_result(void* arg)
{
    (void)arg;
    static const uint8_t u3[6] = { 0x03, 0x00, 0, 0, 0, 0 };
    sim_can_node_send(2, 0x524, false, u3, 6);              // A result the BMS switched off: it reconfigures
}

static void test_ivt_config_interrupted(void)
{
    board_init(true);
    sim_ivt_set_command_hook(cut_hook);
    sim_schedule(SIM_S(5), ev_foreign_result, NULL);
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    SIM_CHECK(config_seen >= 8 + 3, "second configuration not cut off (%d config commands)", config_seen);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "IVT left stopped after a configuration was cut off");
    SIM_CHECK(sim_ivt_running(), "IVT not running");
    check_ivt_configured("after the cut-off configuration");
}


// IVT system errors (status bit 3, latched in the IVT). Bus errors log CAN receive / transmit errors and can set
// it (bench, 2026-10-12: one in seven disturbances with the adapter at the wrong bitrate): the BMS clears those,
// at most CFG_IVT_AUTO_CLEAR_MAX times per power-up. Anything else stays until the IVT is power cycled.
static int syserr_bit = 2;
static void ev_log_syserr(void* arg) { (void)arg; sim_ivt_log_system_error(syserr_bit); }

static void test_ivt_can_errors_cleared(void)
{
    board_init(true);
    syserr_bit = 2;                                         // CAN receive
    sim_schedule(SIM_S(5), ev_log_syserr, NULL);
    sim_run_firmware(SIM_S(9));
    common_checks(true);
    uint64_t trip = first_fault_after(SIM_S(5));
    SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(5) <= FS_LIMIT_US, "IVT system error: no fault");
    SIM_CHECK(sim_ivt_error_resets() == 1, "IVT errors cleared %d times, expected once", sim_ivt_error_resets());
    SIM_CHECK(sim_uart_count("system errors cleared") == 1, "clearing not logged");
    SIM_CHECK(!sim_fault_active(), "still faulted 4 s after a CAN-only system error (flags 0x%02lX)",
              (unsigned long)BMS_Safety_GetActiveFlags());
    const sim_transition* list;
    int n = sim_fault_transitions(&list);
    if (n > 0 && !list[n - 1].state) sim_note("CAN-only system error: AMS fault released %.2f s after it appeared",
                                               (list[n - 1].t_us - SIM_S(5)) / 1e6);
    check_ivt_configured("after clearing its errors");
}

static void test_ivt_system_error_kept(void)
{
    board_init(true);
    syserr_bit = 12;                                        // ADC clock: not a bus disturbance
    sim_schedule(SIM_S(5), ev_log_syserr, NULL);
    sim_run_firmware(SIM_S(10));
    common_checks(true);
    SIM_CHECK(first_fault_after(SIM_S(5)) != UINT64_MAX, "IVT system error: no fault");
    SIM_CHECK(sim_fault_active() && (BMS_Safety_GetActiveFlags() & BMS_ERR_IVT),
              "ADC clock error not kept: AMS not faulted 5 s later");
    SIM_CHECK(sim_ivt_error_resets() == 0, "IVT errors cleared although one wasn't a CAN error");
    SIM_CHECK(sim_uart_count("ADC clock. Not cleared automatically") == 1, "reason not logged");
    SIM_CHECK(sim_ivt_running(), "IVT left stopped after the check");
    check_ivt_configured("after the check");
}

static void test_ivt_can_error_clear_limit(void)
{
    board_init(true);
    syserr_bit = 3;                                         // CAN transmit
    for (int i = 0; i <= CFG_IVT_AUTO_CLEAR_MAX; i++) sim_schedule(SIM_S(5 + 4 * i), ev_log_syserr, NULL);
    sim_run_firmware(SIM_S(5 + 4 * (CFG_IVT_AUTO_CLEAR_MAX + 1) + 1));
    common_checks(true);
    SIM_CHECK(sim_ivt_error_resets() == CFG_IVT_AUTO_CLEAR_MAX, "IVT errors cleared %d times, expected %d",
              sim_ivt_error_resets(), CFG_IVT_AUTO_CLEAR_MAX);
    SIM_CHECK(sim_fault_active(), "not faulted after more than %d CAN-only system errors", CFG_IVT_AUTO_CLEAR_MAX);
    SIM_CHECK(sim_uart_count("not again: power cycle the IVT") == 1, "limit not logged");
}

// A bus disturbance (frames lost on and off for 2 s) that latches CAN errors in the IVT: the BMS waits for a clean
// bus, then clears them once (bench, 2026-10-12: checks started mid-disturbance failed or were undone at once)
// Three bursts of lost frames (100 ms off / 100 ms on for 0.6 s) with 0.9 s of clean bus between them, each
// latching CAN errors, like the bench's node at the wrong bitrate
static int disturb_steps = 0;
static void ev_disturb(void* arg)
{
    (void)arg;
    int in_burst = disturb_steps % 15;                      // 15 steps of 100 ms: 6 disturbed, 9 clean
    bool off = in_burst < 6 && (in_burst % 2) == 0;
    sim_ivt_set_connected(!off);
    sim_can_set_ack(2, true);
    if (in_burst == 0) sim_ivt_log_system_error((disturb_steps / 15) % 2 ? 3 : 2);
    if (++disturb_steps < 3 * 15) sim_schedule(sim_now_us() + SIM_MS(100), ev_disturb, NULL);
    else sim_ivt_set_connected(true);
}

static void test_ivt_can_errors_during_disturbance(void)
{
    board_init(true);
    sim_schedule(SIM_S(5), ev_disturb, NULL);
    sim_run_firmware(SIM_S(14));
    common_checks(true);
    SIM_CHECK(sim_ivt_error_resets() == 1, "IVT errors cleared %d times, expected once after the disturbance",
              sim_ivt_error_resets());
    SIM_CHECK(!sim_fault_active(), "still faulted 4 s after the disturbance (flags 0x%02lX)",
              (unsigned long)BMS_Safety_GetActiveFlags());
    check_ivt_configured("after the disturbance");
}

// A command lost on the way to one device (a bad isoSPI contact; bench 2026-10-12): the poll then finds nothing
// converting and the read returns the previous conversion, the open-wire one with every odd cell ~10 % low, with
// a valid PEC. The command counter tells: such a read must fail and its stale data must never be used.
static void ev_lose_conversion(void* arg) { (void)arg; sim_adbms_lose_next_conversion(SLAVE(5)); }

static void test_isospi_lost_command(void)
{
    board_init(true);
    for (int i = 0; i < 10; i++) sim_schedule(SIM_S(4) + i * SIM_MS(700) + i * 13000, ev_lose_conversion, NULL);
    sim_run_firmware(SIM_S(12));
    common_checks(true);
    float worst = 0;
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        uint32_t base = CFG_CAN_ID_CELL_V_BASE + SLAVE(5) * CFG_CAN_V_FRAMES_PER_SLAVE;
        if (f->bus != 1 || !f->from_mcu || f->t_us < SIM_S(3) || f->id < base || f->id >= base + CFG_CAN_V_FRAMES_PER_SLAVE) continue;
        for (int j = 0; j < 4; j++)
        {
            if ((int)(f->id - base) * 4 + j >= CFG_CELLS_PER_SLAVE) continue;
            float dv = fabsf(le16(&f->data[j * 2]) / 1000.0f - 3.80f);
            if (dv > worst) worst = dv;
        }
    }
    sim_note("%d command counter mismatches logged, cells on CAN within %.0f mV", sim_uart_count("COMMAND COUNTER"),
             worst * 1000);
    SIM_CHECK(sim_uart_count("COMMAND COUNTER") >= 10, "lost conversion commands not detected (%d of 10)",
              sim_uart_count("COMMAND COUNTER"));
    SIM_CHECK(worst < 0.02f, "stale data used: a cell on CAN %.0f mV off", worst * 1000);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(12)) == 0, "lost commands caused a fault");
}

static void ev_flood(void* arg)
{
    (void)arg;
    char text[400];
    memset(text, 'x', sizeof(text) - 2);
    text[sizeof(text) - 2] = '\n';
    text[sizeof(text) - 1] = 0;
    for (int i = 0; i < 200; i++) printfDma("%s", text);
}

static void test_uart_flood(void)
{
    board_init(true);
    sim_schedule(SIM_S(4), ev_flood, NULL);
    sim_run_firmware(SIM_S(6));
    common_checks(true);
    sim_note("%lu messages dropped during the flood (expected, buffer is 10 kB)", (unsigned long)droppedMessages);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(6)) == 0, "UART flood disturbed the AMS");
}

static void test_temperature_table_edges(void)
{
    board_init(true);
    // Exactly at the table ends, and just outside
    sim_adbms_set_gpio(SLAVE(0), temp_gpio[0], table_v[0]);
    sim_adbms_set_gpio(SLAVE(0), temp_gpio[1], table_v[sizeof(table_v) / sizeof(table_v[0]) - 1]);
    sim_adbms_set_gpio(SLAVE(0), temp_gpio[2], temp_to_voltage(25.0f));
    sim_run_firmware(SIM_S(3));

    const sim_can_frame* burst[256];
    int n = last_burst(burst, 256);
    const sim_can_frame* f = find_frame(burst, n, CFG_CAN_ID_CELL_T_BASE);
    SIM_CHECK(f != NULL, "no temperature frame");
    if (f)
    {
        float t0 = le16(&f->data[0]) / 10.0f, t1 = le16(&f->data[2]) / 10.0f, t2 = le16(&f->data[4]) / 10.0f;
        sim_note("table ends decode to %.1f C and %.1f C", t0, t1);
        SIM_CHECK(fabsf(t0 - table_c[0]) < 0.5f || t0 > 800.0f, "%.3f V decodes to %.1f C", table_v[0], t0);
        SIM_CHECK(t1 > 100.0f, "lowest table voltage decodes to %.1f C", t1);
        SIM_CHECK(fabsf(t2 - 25.0f) < 0.3f, "25 C sensor decodes to %.1f C", t2);
    }
    SIM_CHECK(sim_error_handler_calls == 0, "Error_Handler was reached");
}


/* ===================================================================================== */
/*  State of charge and energy                                                            */
/* ===================================================================================== */

// The firmware's voltage -> SoC model against the spreadsheet's own results
static void test_soc_model(void)
{
    // Battery Tracker.xlsx, sheet 12-08-2026, cached VToSoc results (C10 = 3.68 V, C11 = 3.69 V)
    const struct { float v; float soc; } ref[] = { { 3.68f, 51.65309809450301f }, { 3.69f, 52.72408394382027f } };
    for (size_t i = 0; i < sizeof(ref) / sizeof(ref[0]); i++)
    {
        float got = BMS_SoC_FromCellVoltage(ref[i].v);
        SIM_CHECK(fabsf(got - ref[i].soc) < 0.01f, "SoC(%.2f V) = %.4f %%, spreadsheet %.4f %%", ref[i].v, got, ref[i].soc);
        sim_note("SoC(%.2f V) = %.4f %% (spreadsheet %.4f %%)", ref[i].v, got, ref[i].soc);
    }

    // Whole curve for test/check_soc.py, which evaluates the spreadsheet's coefficients itself
    char path[256];
    snprintf(path, sizeof(path), "%s/soc_model.csv", out_dir);
    FILE* f = fopen(path, "w");
    float prev = -1.0f;
    int decreasing = 0;
    for (int mv = 2700; mv <= 4200; mv += 5)
    {
        float v = mv / 1000.0f;
        float soc = BMS_SoC_FromCellVoltage(v);
        if (f) fprintf(f, "%.3f,%.6f\n", v, soc);
        SIM_CHECK(soc >= 0.0f && soc <= 100.0f, "SoC(%.3f V) = %.3f %% outside 0..100", v, soc);
        if (v >= 3.0f && v <= 4.05f && soc < prev - 0.01f) decreasing++;
        prev = soc;
    }
    if (f) fclose(f);
    SIM_CHECK(decreasing == 0, "SoC curve falls with rising voltage at %d points between 3.0 and 4.05 V", decreasing);
}

static void set_all_cells(float volts)
{
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) set_cell(s, c, volts);
}

static void test_soc_startup(void)
{
    board_init(true);
    set_all_cells(3.68f);
    sim_ivt_set_current(0.0f);
    sim_run_firmware(SIM_S(4));
    common_checks(true);

    SocState soc = BMS_SoC_Get();
    float expect = BMS_SoC_FromCellVoltage(3.68f);
    SIM_CHECK(soc.valid, "no SoC estimate after start-up");
    SIM_CHECK(fabsf(soc.initialSoc_percent - 51.653f) < 0.3f, "start-up SoC %.2f %%, spreadsheet 51.65 %%", soc.initialSoc_percent);
    SIM_CHECK(fabsf(soc.remaining_Wh - expect / 100.0f * CFG_PACK_ENERGY_WH) < 30.0f, "remaining %.0f Wh, expected %.0f Wh",
              soc.remaining_Wh, expect / 100.0f * CFG_PACK_ENERGY_WH);
    SIM_CHECK(sim_uart_count("SoC estimate") == 1, "SoC estimated %d times", sim_uart_count("SoC estimate"));
    sim_note("start-up SoC %.2f %%, %.0f of %.0f Wh", soc.initialSoc_percent, soc.remaining_Wh, (double)CFG_PACK_ENERGY_WH);
}

// Discharge, regen and an IVT restart: the firmware's energy count must follow the IVT's counter
static double energy_at_start = 0;
static float  power_at_150a = 0;
static void ev_energy_start(void* arg)   { (void)arg; energy_at_start = sim_ivt_energy_wh(); }
static void ev_discharge(void* arg)      { (void)arg; sim_ivt_set_current(ivt_reported(150.0f)); }
static void ev_regen(void* arg)          { (void)arg; sim_ivt_set_current(ivt_reported(-50.0f)); }
static void ev_rest(void* arg)           { (void)arg; sim_ivt_set_current(0.0f); }
static void ev_drive(void* arg)          { (void)arg; sim_ivt_set_current(ivt_reported(100.0f)); }
static void ev_sample_power(void* arg)   { (void)arg; power_at_150a = BMS_IVT_GetReading().power_W; }

static void test_energy_tracking(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(3.0), ev_energy_start, NULL);
    sim_schedule(SIM_S(4.0), ev_discharge, NULL);
    sim_schedule(SIM_S(20.0), ev_sample_power, NULL);
    sim_schedule(SIM_S(34.0), ev_regen, NULL);
    sim_schedule(SIM_S(44.0), ev_rest, NULL);
    sim_schedule(SIM_S(46.0), ev_ivt_power_cycle, NULL);      // Counter reloads 37 Wh lower
    sim_schedule(SIM_S(50.0), ev_drive, NULL);
    sim_schedule(SIM_S(60.0), ev_rest, NULL);
    sim_run_firmware(SIM_S(62));
    common_checks(true);

    SocState soc = BMS_SoC_Get();
    double expect = (sim_ivt_energy_wh() - energy_at_start) + 37.0;     // Real energy, without the reload step
    SIM_CHECK(fabs(soc.used_Wh - expect) < 3.0, "energy used %.1f Wh, IVT measured %.1f Wh", soc.used_Wh, expect);
    float initial = soc.initialSoc_percent / 100.0f * CFG_PACK_ENERGY_WH;
    SIM_CHECK(fabsf(soc.remaining_Wh - (initial - soc.used_Wh)) < 0.5f, "remaining %.1f Wh != initial %.1f - used %.1f",
              soc.remaining_Wh, initial, soc.used_Wh);
    SIM_CHECK(fabsf(soc.soc_percent - soc.remaining_Wh / CFG_PACK_ENERGY_WH * 100.0f) < 0.01f, "live SoC inconsistent");
    float expectPower = 150.0f * sim_ivt_u1();
    SIM_CHECK(fabsf(power_at_150a - expectPower) < 0.01f * expectPower, "power at 150 A: %.0f W, expected %.0f W", power_at_150a, expectPower);
    SIM_CHECK(sim_uart_count("Energy counter restart epoch changed") >= 1, "IVT restart not detected (%d)", sim_uart_count("counting restarted"));
    sim_note("used %.1f Wh (IVT %.1f Wh), live SoC %.2f %% from %.2f %%", soc.used_Wh, expect, soc.soc_percent, soc.initialSoc_percent);
}


/* ===================================================================================== */
/*  Fast frames                                                                           */
/* ===================================================================================== */

// Latest frame with this ID the MCU put on CAN1
static const sim_can_frame* last_sent(uint32_t id)
{
    for (int i = sim_can_log_count() - 1; i >= 0; i--)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == 1 && f->from_mcu && f->id == id) return f;
    }
    return NULL;
}

static const sim_can_frame* first_sent(uint32_t id)
{
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == 1 && f->from_mcu && f->id == id) return f;
    }
    return NULL;
}

static int count_frames(int bus, bool from_mcu, int32_t id, uint64_t from, uint64_t to)
{
    int count = 0;
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == bus && f->from_mcu == from_mcu && (id < 0 || f->id == (uint32_t)id) && f->t_us >= from && f->t_us < to) count++;
    }
    return count;
}


static void ev_ts_on(void* arg);

// 1.5x the power the int16 power signals can carry, at the present pack voltage
static float saturating_current(void)
{
    return 1.5f * 32767.0f * CFG_CAN_POWER_W_PER_BIT / sim_ivt_u1();
}

static uint64_t fast_busy_start = 0;
static void ev_fast_busy_start(void* arg) { (void)arg; fast_busy_start = sim_can_busy_us(1); }

static void ev_fast_values(void* arg)
{
    (void)arg;
    double load = (sim_can_busy_us(1) - fast_busy_start) / 2e6 * 100.0;
    sim_note("CAN1 load %.1f %% at %lu bit/s (typical bit stuffing)", load, (unsigned long)sim_can_bitrate(1));
    SIM_CHECK(load < CFG_CAN1_MAX_LOAD_PERCENT, "CAN1 load %.1f %%", load);

    int n1 = count_frames(1, true, CFG_CAN_ID_FAST_1, SIM_S(4), SIM_S(6));
    int n2 = count_frames(1, true, CFG_CAN_ID_FAST_2, SIM_S(4), SIM_S(6));
    int expect = 2000 / CFG_CAN_FAST_PERIOD_MS;
    SIM_CHECK(abs(n1 - expect) <= 2 && abs(n2 - expect) <= 2, "fast frames in 2 s: %d and %d, expected %d", n1, n2, expect);
    int ivt = count_frames(2, false, -1, SIM_S(4), SIM_S(6));
    sim_note("fast frames: %d + %d in 2 s, IVT %d result frames in 2 s", n1, n2, ivt);
    SIM_CHECK(ivt / 2 <= 1000 && ivt / 2 >= 700, "IVT sent %d frames per second", ivt / 2);

    const sim_can_frame* f1 = last_sent(CFG_CAN_ID_FAST_1);
    const sim_can_frame* f2 = last_sent(CFG_CAN_ID_FAST_2);
    SIM_CHECK(f1 && f2 && f1->ext && f1->dlc == 8 && f2->ext && f2->dlc == 8, "fast frames missing or not extended 8-byte frames");
    if (!f1 || !f2) return;

    int32_t u1 = (int32_t)lroundf(sim_ivt_u1() * 1000.0f);
    SIM_CHECK(le32(&f1->data[0]) == 123456, "fast current %ld mA, IVT 123456 mA", (long)le32(&f1->data[0]));
    SIM_CHECK(le32(&f1->data[4]) == u1, "fast U1 %ld mV, IVT %ld mV", (long)le32(&f1->data[4]), (long)u1);
    SIM_CHECK(bypass_on() && le32(&f2->data[0]) == u1, "fast U2 %ld mV with the bypass %s, U1 %ld mV",
              (long)le32(&f2->data[0]), bypass_on() ? "closed" : "open", (long)u1);
    long power = lroundf(123.456f * (u1 / 1000.0f) / CFG_CAN_POWER_W_PER_BIT);
    SIM_CHECK(labs(le16(&f2->data[4]) - power) <= 1, "fast power %d steps, expected %ld", le16(&f2->data[4]), power);
    SocState soc = BMS_SoC_Get();
    uint16_t remaining = (uint16_t)le16(&f2->data[6]);
    SIM_CHECK(soc.valid && abs((int)remaining - (int)lroundf(soc.remaining_Wh)) <= 1, "fast remaining %u Wh, SoC module %.1f Wh",
              remaining, soc.remaining_Wh);
    sim_note("fast frames: %ld mA, %ld mV, U2 %ld mV, %d x %g W, %u Wh", (long)le32(&f1->data[0]), (long)le32(&f1->data[4]),
             (long)le32(&f2->data[0]), le16(&f2->data[4]), (double)CFG_CAN_POWER_W_PER_BIT, remaining);

    sim_ivt_set_current(ivt_reported(saturating_current()));
}

static void ev_fast_saturate_pos(void* arg)
{
    (void)arg;
    const sim_can_frame* f2 = last_sent(CFG_CAN_ID_FAST_2);
    SIM_CHECK(f2 && le16(&f2->data[4]) == 32767, "power above the int16 range sent as %d, expected 32767", f2 ? le16(&f2->data[4]) : 0);
}

static void ev_fast_saturate_energy(void* arg)
{
    (void)arg;
    const sim_can_frame* energy = last_sent(CFG_CAN_ID_ENERGY);     // 10 Hz: one burst later
    SIM_CHECK(energy && le16(&energy->data[0]) == 32767, "energy frame power above the int16 range: %d", energy ? le16(&energy->data[0]) : 0);
    sim_ivt_set_current(ivt_reported(-saturating_current()));
}

static void ev_fast_saturate_neg(void* arg)
{
    (void)arg;
    const sim_can_frame* f2 = last_sent(CFG_CAN_ID_FAST_2);
    SIM_CHECK(f2 && le16(&f2->data[4]) == -32767, "power below the int16 range sent as %d, expected -32767", f2 ? le16(&f2->data[4]) : 0);
    sim_ivt_set_current(ivt_reported(20.0f));
    sim_ivt_set_connected(false);
}

static void ev_fast_ivt_lost(void* arg)
{
    (void)arg;
    const sim_can_frame* f1 = last_sent(CFG_CAN_ID_FAST_1);
    const sim_can_frame* f2 = last_sent(CFG_CAN_ID_FAST_2);
    if (!f1 || !f2) return;
    SIM_CHECK(le32(&f1->data[0]) == INT32_MIN && le32(&f1->data[4]) == INT32_MIN && le32(&f2->data[0]) == INT32_MIN,
              "IVT lost: fast frames still carry %ld mA, %ld mV, %ld mV", (long)le32(&f1->data[0]), (long)le32(&f1->data[4]),
              (long)le32(&f2->data[0]));
    SIM_CHECK(le16(&f2->data[4]) == INT16_MIN, "IVT lost: power %d, expected not available", le16(&f2->data[4]));
    SIM_CHECK((uint16_t)le16(&f2->data[6]) != 0xFFFF, "IVT lost: remaining energy reported as unknown");
    const sim_can_frame* energy = last_sent(CFG_CAN_ID_ENERGY);
    SIM_CHECK(energy && le16(&energy->data[0]) == INT16_MIN, "IVT lost: energy frame power %d", energy ? le16(&energy->data[0]) : 0);
}

static void ev_fast_load(void *unused) {(void)unused;sim_ivt_set_current(ivt_reported(123.456f));}
static void test_fast_frames(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(1.5),ev_fast_load,NULL);
    sim_schedule(SIM_S(3.0), ev_ts_on, NULL);                   // Bypass closed by ~4.3 s, so U2 = U1
    sim_schedule(SIM_S(4.0), ev_fast_busy_start, NULL);
    sim_schedule(SIM_S(6.0), ev_fast_values, NULL);
    sim_schedule(SIM_S(6.05), ev_fast_saturate_pos, NULL);
    sim_schedule(SIM_S(6.15), ev_fast_saturate_energy, NULL);
    sim_schedule(SIM_S(6.20), ev_fast_saturate_neg, NULL);      // Also disconnects the IVT
    sim_schedule(SIM_S(6.60), ev_fast_ivt_lost, NULL);          // > CFG_IVT_VOLTAGE_TIMEOUT_MS later
    sim_run_firmware(SIM_S(6.7));
    common_checks(true);
    check_fast_timing();
    SIM_CHECK(canTxErrorCount == 0, "%lu CAN frames could not be queued", (unsigned long)canTxErrorCount);
    SIM_CHECK(sim_can_rx_lost(2) == 0, "%d IVT frames lost in the CAN2 RX FIFOs", sim_can_rx_lost(2));

    // Before the IVT and the first measurement: everything "not available"
    const sim_can_frame* f1 = first_sent(CFG_CAN_ID_FAST_1);
    const sim_can_frame* f2 = first_sent(CFG_CAN_ID_FAST_2);
    SIM_CHECK(f1 && f2, "no fast frames at start-up");
    if (f1 && f2)
    {
        sim_note("first fast frame at %.1f ms", f1->t_us / 1000.0);
        SIM_CHECK(le32(&f1->data[0]) == INT32_MIN && le32(&f1->data[4]) == INT32_MIN && le32(&f2->data[0]) == INT32_MIN &&
                  le16(&f2->data[4]) == INT16_MIN && (uint16_t)le16(&f2->data[6]) == 0xFFFF,
                  "first fast frames not all 'not available'");
    }
}


/* ===================================================================================== */
/*  Inverter current limits                                                               */
/* ===================================================================================== */

static int16_t be16(const uint8_t* p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

// Worked out here independently of bms_inverter.c: linear from base at start down to end_a at end
static float expect_derated(float base, float temp_c, float start, float end, float end_a)
{
    if (base <= end_a) return base;
    float f = (temp_c - start) / (end - start);
    f = fminf(fmaxf(f, 0.0f), 1.0f);
    return base - (base - end_a) * f;
}

// Expected limits at a hottest-cell temperature and pack voltage
typedef struct { float discharge, regen, power; int temp, power_bit, regen_bit, fault; } limit_expect;

// The test pack draws healthy_pack()'s 20 A. A limit above that allows for at least
// CFG_PACK_RESISTANCE_MIN_OHM of sag: I x (U - R (I - I_now)) = P
#define LIMIT_TEST_CURRENT_A 20.0f
static float expect_current_for_power(float power_w, float u1)
{
    const float r = CFG_PACK_RESISTANCE_MIN_OHM;
    float b = u1 + r * LIMIT_TEST_CURRENT_A;
    if (r <= 0) return power_w / u1;
    float disc = b * b - 4.0f * r * power_w;
    return disc >= 0 ? (b - sqrtf(disc)) / (2.0f * r) : INFINITY;     // Unreachable (small bench pack)
}

static limit_expect expected_limits(float temp_c, float u1)
{
    limit_expect e;
    float derated = expect_derated(CFG_MAX_DISCHARGE_CURRENT_A, temp_c, CFG_INVERTER_DERATE_START_C,
                                   CFG_INVERTER_DERATE_END_C, CFG_INVERTER_DERATE_CURRENT_A);
    float power_a = expect_current_for_power(CFG_INVERTER_POWER_LIMIT_W, u1);
    e.discharge = fminf(derated, power_a);
    e.regen = expect_derated(CFG_INVERTER_REGEN_CURRENT_A, temp_c, CFG_INVERTER_REGEN_DERATE_START_C,
                             CFG_INVERTER_REGEN_DERATE_END_C, CFG_INVERTER_REGEN_DERATE_CURRENT_A);
    if(temp_c<CFG_CELL_CHARGE_TEMP_MIN_C || temp_c>CFG_CELL_CHARGE_TEMP_MAX_C)e.regen=0;
    e.power_bit = power_a < derated;
    e.power = e.power_bit ? CFG_INVERTER_POWER_LIMIT_W
                          : derated * (u1 - CFG_PACK_RESISTANCE_MIN_OHM * (derated - LIMIT_TEST_CURRENT_A));
    e.temp = temp_c > CFG_INVERTER_DERATE_START_C;
    e.regen_bit = temp_c > CFG_INVERTER_REGEN_DERATE_START_C;
    e.fault = 0;
    return e;
}

// Allowed error: 0.6 A, plus 0.1 C of temperature error on the derating slope
#define LIMIT_TOL_A      0.6f
#define LIMIT_TOL_TEMP_A (0.6f + 0.1f * (CFG_MAX_DISCHARGE_CURRENT_A - CFG_INVERTER_DERATE_CURRENT_A) / \
                          (CFG_INVERTER_DERATE_END_C - CFG_INVERTER_DERATE_START_C))

// modified: runtime settings differ from the built-in limits (BMS_Limits flag bit 5)
static void check_limits(const char* when, limit_expect e, float tol, int modified)
{
    const sim_can_frame* d = last_sent(CFG_CAN_ID_INVERTER_DC_LIMIT);
    const sim_can_frame* b = last_sent(CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT);
    const sim_can_frame* l = last_sent(CFG_CAN_ID_LIMITS);
    const sim_can_frame* status = last_sent(CFG_CAN_ID_STATUS);
    SIM_CHECK(d && b && l && status, "%s: inverter limit, limits or status frames missing", when);
    if (!d || !b || !l || !status) return;

    // DTI frames, big-endian A x 10
    float got_d = be16(d->data) / 10.0f;
    float got_b = be16(b->data) / 10.0f;
    // BMS_Limits, little-endian
    float power = le16(&l->data[0]) * CFG_CAN_POWER_W_PER_BIT;
    float lim_d = le16(&l->data[2]) / 10.0f;
    float lim_r = le16(&l->data[4]) / 10.0f;
    sim_note("%s: discharge %.1f A, regen %.1f A, power limit %.1f kW, flags 0x%X", when, got_d, -got_b, power / 1000.0f,
             l->data[6]);

    SIM_CHECK(fabsf(got_d - e.discharge) <= tol && fabsf(-got_b - e.regen) <= tol,
              "%s: inverter limits %.1f / %.1f A, expected %.1f / %.1f A", when, got_d, got_b, e.discharge, -e.regen);
    SIM_CHECK(fabsf(lim_d - e.discharge) <= tol && fabsf(lim_r - e.regen) <= tol,
              "%s: BMS_Limits currents %.1f / %.1f A, expected %.1f / %.1f A", when, lim_d, lim_r, e.discharge, e.regen);
    SIM_CHECK(fabsf(power - e.power) <= tol * sim_ivt_u1() + 5.0f, "%s: power limit %.0f W, expected %.0f W", when, power, e.power);
    int flags = e.temp | (e.power_bit << 1) | (e.regen_bit << 2) | (e.fault << 3) | (modified << 5);
    SIM_CHECK(l->data[6] == flags, "%s: BMS_Limits flags 0x%X, expected 0x%X", when, l->data[6], flags);
    if (!e.fault)
    {
        SIM_CHECK(((status->data[1] >> 6) & 1) == e.temp && ((status->data[1] >> 7) & 1) == e.power_bit,
                  "%s: status derating bits %d %d, expected %d %d", when, (status->data[1] >> 6) & 1,
                  (status->data[1] >> 7) & 1, e.temp, e.power_bit);
    }
}

static void ev_lim_full(void* arg)
{
    (void)arg;
    // 3.5 V cells: the 200 A limit, not the power limit, for the car's pack
    check_limits("3.5 V cells, 25 C", expected_limits(25.0f, sim_ivt_u1()), LIMIT_TOL_A, 0);

    int n = count_frames(1, true, CFG_CAN_ID_INVERTER_DC_LIMIT, SIM_S(2.5), SIM_S(3.5));
    int nl = count_frames(1, true, CFG_CAN_ID_LIMITS, SIM_S(2.5), SIM_S(3.5));
    int expect = 1000 / CFG_INVERTER_LIMIT_PERIOD_MS;
    SIM_CHECK(abs(n - expect) <= 1 && abs(nl - expect) <= 1, "%d inverter limit and %d BMS_Limits frames in 1 s, expected %d",
              n, nl, expect);
    const sim_can_frame* d = last_sent(CFG_CAN_ID_INVERTER_DC_LIMIT);
    if (d)
    {
        bool ff = true;
        for (int i = 2; i < 8; i++) ff = ff && d->data[i] == 0xFF;
        SIM_CHECK(d->ext && d->dlc == 8 && ff, "inverter frame: ext %d, dlc %d, unused bytes not 0xFF", d->ext, d->dlc);
    }
    set_all_cells(4.0f);                                    // 448 V for the car: above 80 kW / 200 A
}

static void ev_lim_power(void* arg)
{
    (void)arg;
    float u1 = sim_ivt_u1();
    check_limits("4.0 V cells, 25 C", expected_limits(25.0f, u1), LIMIT_TOL_A, 0);
    // The car's pack is above 80 kW / 200 A here, so its power limit must have been exercised
    SIM_CHECK(CFG_TOTAL_CELLS * 4.0f < CFG_INVERTER_POWER_LIMIT_W / CFG_MAX_DISCHARGE_CURRENT_A ||
              expect_current_for_power(CFG_INVERTER_POWER_LIMIT_W, u1) < CFG_MAX_DISCHARGE_CURRENT_A,
              "power limit not reached at %.0f V", u1);
    set_temp(SLAVE(2), 1, 50.0f);                           // Regen derating only
}

static void ev_lim_regen(void* arg)
{
    (void)arg;
    check_limits("50 C", expected_limits(50.0f, sim_ivt_u1()), LIMIT_TOL_A, 0);
    set_temp(SLAVE(2), 1, 57.5f);
}

static void ev_lim_warm(void* arg)
{
    (void)arg;
    check_limits("57.5 C", expected_limits(57.5f, sim_ivt_u1()), LIMIT_TOL_TEMP_A, 0);
    set_temp(SLAVE(2), 1, 59.5f);
}

static void ev_lim_hot(void* arg)
{
    (void)arg;
    check_limits("59.5 C", expected_limits(59.5f, sim_ivt_u1()), LIMIT_TOL_TEMP_A, 0);
    set_temp(SLAVE(2), 1, 25.0f);
    set_cell(SLAVE(7), 3, CFG_CELL_OV_V + 0.05f);           // AMS fault
}

static void ev_lim_fault(void* arg)
{
    (void)arg;
    SIM_CHECK(sim_fault_active(), "no AMS fault for the over-voltage");
    limit_expect zero = { 0.0f, 0.0f, 0.0f, 0, 0, 0, 1 };
    check_limits("AMS faulted", zero, LIMIT_TOL_A, 0);
}

static void test_inverter_limits(void)
{
    board_init(true);
    set_all_cells(3.5f);                                    // 392 V
    sim_schedule(SIM_S(3.5), ev_lim_full, NULL);
    sim_schedule(SIM_S(4.2), ev_lim_power, NULL);
    sim_schedule(SIM_S(4.9), ev_lim_regen, NULL);
    sim_schedule(SIM_S(5.6), ev_lim_warm, NULL);
    sim_schedule(SIM_S(6.3), ev_lim_hot, NULL);
    sim_schedule(SIM_S(7.0), ev_lim_fault, NULL);           // > 500 ms after the fault
    sim_run_firmware(SIM_S(7.1));
    common_checks(true);
    check_fast_timing();
    SIM_CHECK(canTxErrorCount == 0, "%lu CAN frames could not be queued", (unsigned long)canTxErrorCount);
}


/* ===================================================================================== */
/*  Drive model: inverter that obeys the BMS's DC current limit, pack sag, FS data logger */
/* ===================================================================================== */

#define LOGGER_BINS 500                         // 500 ms moving average at 1 ms (FS 2027 D 10.4.1)
#define DRIVE_MAX_MS 40000

typedef float (*demand_fn)(uint64_t t_us);      // Driver's DC current request (A)

static struct
{
    demand_fn demand;
    float     ocv_cell_v;                       // Open-circuit cell voltage
    float     r_pack_ohm;                       // Pack resistance: cells and U1 sag together
    float     u1_offset_v;                      // IVT U1 error (plausibility test)
    uint64_t  lag_us;                           // Inverter: time to apply a new limit
    uint64_t  start_us, end_us;
    float     limit_a;
    float     pending_a;
    uint64_t  pending_at;
    double    bins[LOGGER_BINS];
    int       head;
    double    sum;
    double    max_avg_w;
    uint64_t  max_avg_t;
    int       n;
    float     power_w[DRIVE_MAX_MS];            // Delivered, per ms from start_us
    float     request_w[DRIVE_MAX_MS];          // Driver's request at the same voltage
    float     limit_w[DRIVE_MAX_MS];            // Inverter's applied limit x the voltage at it
} drive;

static void drive_on_frame(const sim_can_frame* f)
{
    if (f->ext && f->id == CFG_CAN_ID_INVERTER_DC_LIMIT && f->dlc >= 2)
    {
        drive.pending_a  = be16(f->data) / 10.0f;
        drive.pending_at = f->t_us + drive.lag_us;
    }
}

static void ev_drive_tick(void* arg)
{
    (void)arg;
    uint64_t now = sim_now_us();
    if (now >= drive.pending_at)
    {
        drive.limit_a = drive.pending_a;
        drive.pending_at = UINT64_MAX;
    }
    float want = fmaxf(drive.demand(now), 0.0f);
    float i = fminf(want, drive.limit_a);
    float cell = drive.ocv_cell_v - drive.r_pack_ohm * i / CFG_TOTAL_CELLS;
    for (int s = 0; s < CFG_NUM_SLAVES; s++)
        for (int c = 0; c < CFG_CELLS_PER_SLAVE; c++) sim_adbms_set_cell(s, cell_channel[c] - 1, cell);
    float u = cell * CFG_TOTAL_CELLS;
    sim_ivt_set_u1(u + drive.u1_offset_v);
    sim_ivt_set_current(ivt_reported(i));

    // The data logger: 500 ms moving average of U x I
    double p = (double)u * i;
    drive.sum += p - drive.bins[drive.head];
    drive.bins[drive.head] = p;
    drive.head = (drive.head + 1) % LOGGER_BINS;
    if (drive.sum / LOGGER_BINS > drive.max_avg_w)
    {
        drive.max_avg_w = drive.sum / LOGGER_BINS;
        drive.max_avg_t = now;
    }
    if (drive.n < DRIVE_MAX_MS)
    {
        drive.power_w[drive.n] = (float)p;
        drive.request_w[drive.n] = want * (drive.ocv_cell_v * CFG_TOTAL_CELLS - drive.r_pack_ohm * want);
        drive.limit_w[drive.n] = drive.limit_a * (drive.ocv_cell_v * CFG_TOTAL_CELLS - drive.r_pack_ohm * drive.limit_a);
        drive.n++;
    }
    if (now + 1000 < drive.end_us) sim_schedule(now + 1000, ev_drive_tick, NULL);
}

static void drive_start(demand_fn demand, float ocv_cell_v, float r_pack_ohm, uint64_t start_us, uint64_t end_us)
{
    memset(&drive, 0, sizeof(drive));
    drive.demand = demand;
    drive.ocv_cell_v = ocv_cell_v;
    drive.r_pack_ohm = r_pack_ohm;
    drive.lag_us = 5000;
    drive.start_us = start_us;
    drive.end_us = end_us;
    drive.pending_at = UINT64_MAX;
    can1_hook = drive_on_frame;
    keep_u1_matched = false;
    sim_schedule(start_us, ev_drive_tick, NULL);
}

// Delivered power, averaged over [from, to) seconds
static float drive_avg_w(double from_s, double to_s)
{
    int a = (int)((from_s - drive.start_us / 1e6) * 1000), b = (int)((to_s - drive.start_us / 1e6) * 1000);
    double sum = 0;
    for (int i = a; i < b && i < drive.n; i++) sum += drive.power_w[i];
    return b > a ? (float)(sum / (b - a)) : 0.0f;
}

static float drive_max_w(double from_s, double to_s)
{
    int a = (int)((from_s - drive.start_us / 1e6) * 1000), b = (int)((to_s - drive.start_us / 1e6) * 1000);
    float m = 0;
    for (int i = a; i < b && i < drive.n; i++) m = fmaxf(m, drive.power_w[i]);
    return m;
}

// Pack voltage high enough that 200 A is more than 80 kW, and a sag typical of the 7p VTC6 pack
#define DRIVE_OCV_CELL  4.15f
#define DRIVE_R_PACK    0.2f
#define DRIVE_START_S   3.0
#define WINDOW_LIMIT    CFG_POWER_WINDOW_LIMIT_W
#define WINDOW_TOL_W    (0.003f * WINDOW_LIMIT)    // IVT sample timing vs the logger's own sampling

// The logger's 500 ms average must stay within the limit whichever mode the firmware is in
static void check_logger(const char* when)
{
    float limit = CFG_POWER_WINDOW_LIMITER_ENABLED ? WINDOW_LIMIT : CFG_INVERTER_POWER_LIMIT_W;
    sim_note("%s: logger 500 ms average peaked at %.2f kW at %.3f s (limit %.1f kW)", when, drive.max_avg_w / 1000.0,
             drive.max_avg_t / 1e6, limit / 1000.0f);
    SIM_CHECK(drive.max_avg_w <= limit + WINDOW_TOL_W, "%s: logger 500 ms average %.2f kW over the %.1f kW limit",
              when, drive.max_avg_w / 1000.0, limit / 1000.0f);
}

// Whenever the driver has asked for more than the floor for 30 ms, at least the floor is delivered. When a
// boost ends the first lower limit assumes the worst-case pack resistance (CFG_PACK_RESISTANCE_MAX_OHM)
// and may undershoot for one limit update, so a run below the floor of up to one update period is allowed,
// never more than 5 kW below.
static void check_floor(const char* when)
{
    if (!CFG_POWER_WINDOW_LIMITER_ENABLED) return;
    int below = 0, worst_i = -1, longest_ask = 0, run = 0, longest_run = 0;
    float worst = 1e9f;
    int asked_for = 0;                                      // ms the driver has been asking for more
    for (int i = 0; i < drive.n; i++)
    {
        asked_for = drive.request_w[i] >= CFG_POWER_WINDOW_FLOOR_W + 2000.0f ? asked_for + 1 : 0;
        bool under = asked_for > 30 && drive.power_w[i] < CFG_POWER_WINDOW_FLOOR_W - 1500.0f;
        run = under ? run + 1 : 0;
        if (run > longest_run) longest_run = run;
        if (under)
        {
            if (asked_for > longest_ask) longest_ask = asked_for;
            if (drive.power_w[i] < worst) { worst = drive.power_w[i]; worst_i = i; }
        }
        if (under && (run > CFG_INVERTER_LIMIT_PERIOD_MS + 5 || drive.power_w[i] < CFG_POWER_WINDOW_FLOOR_W - 5000.0f))
        {
            below++;
        }
    }
    if (worst_i >= 0)
    {
        sim_note("%s: lowest %.1f kW while asking for more (floor %.0f kW), longest dip %d ms", when, worst / 1000.0f,
                 CFG_POWER_WINDOW_FLOOR_W / 1000.0f, longest_run);
    }
    if (below)
    {
        sim_note("%s: below the floor up to %d ms into a request", when, longest_ask);
        char path[256];
        snprintf(path, sizeof(path), "%s/floor_debug.csv", out_dir);
        FILE* f = fopen(path, "w");
        if (f)
        {
            fprintf(f, "t_s,request_kw,power_kw,limit_kw,logger_avg_kw\n");
            for (int i = worst_i - 600 < 0 ? 0 : worst_i - 600; i < drive.n && i < worst_i + 200; i++)
            {
                double avg = 0;
                for (int k = i - LOGGER_BINS + 1; k <= i; k++) avg += k >= 0 ? drive.power_w[k] : 0;
                fprintf(f, "%.3f,%.2f,%.2f,%.2f,%.2f\n", drive.start_us / 1e6 + i / 1000.0, drive.request_w[i] / 1000,
                        drive.power_w[i] / 1000, drive.limit_w[i] / 1000, avg / LOGGER_BINS / 1000);
            }
            fclose(f);
        }
    }
    SIM_CHECK(below == 0, "%s: %d ms too far or too long below the %.0f kW floor (worst %.1f kW at %.3f s)",
              when, below, CFG_POWER_WINDOW_FLOOR_W / 1000.0f, worst / 1000.0f, drive.start_us / 1e6 + worst_i / 1000.0);
}

static float demand_sustained(uint64_t t) { return t < SIM_S(DRIVE_START_S + 0.5) ? 0.0f : 300.0f; }

static void test_power_window_sustained(void)
{
    board_init(true);
    drive_start(demand_sustained, DRIVE_OCV_CELL, DRIVE_R_PACK, SIM_S(DRIVE_START_S), SIM_S(9.0));
    sim_run_firmware(SIM_S(9.0));
    common_checks(true);
    check_logger("sustained full throttle");
    check_floor("sustained full throttle");

    float t0 = DRIVE_START_S + 0.5;
    float peak = drive_max_w(t0, t0 + 0.5), first = drive_avg_w(t0, t0 + 0.5), late = drive_avg_w(t0 + 2.5, t0 + 5.5);
    float late_max = drive_max_w(t0 + 1.5, t0 + 5.5);
    sim_note("peak %.1f kW, first 500 ms %.1f kW, from 2.5 s on %.1f kW (max %.1f kW)", peak / 1000, first / 1000,
             late / 1000, late_max / 1000);
    if (CFG_POWER_WINDOW_LIMITER_ENABLED)
    {
        SIM_CHECK(peak <= WINDOW_LIMIT*1.01f, "instantaneous ceiling exceeded: %.1f kW", peak / 1000);
        // Settles at the limit: no boost repeated once the unused energy is spent
        SIM_CHECK(fabsf(late - WINDOW_LIMIT) < 1500.0f && late_max < WINDOW_LIMIT + 1500.0f,
                  "not settled at the limit: %.1f kW average, %.1f kW max", late / 1000, late_max / 1000);
    }
    else
    {
        SIM_CHECK(peak < CFG_INVERTER_POWER_LIMIT_W * 1.01f, "flat limit exceeded: %.1f kW", peak / 1000);
    }
}

// Corners and short straights: 1 s at 40 A, a 250 ms burst, 250 ms off
static float demand_bursts(uint64_t t)
{
    if (t < SIM_S(DRIVE_START_S)) return 0.0f;
    uint64_t phase = (t - SIM_S(DRIVE_START_S)) % SIM_S(1.5);
    return phase < SIM_S(1.0) ? 40.0f : (phase < SIM_S(1.25) ? 300.0f : 0.0f);
}

static void test_power_window_bursts(void)
{
    board_init(true);
    drive_start(demand_bursts, DRIVE_OCV_CELL, DRIVE_R_PACK, SIM_S(DRIVE_START_S), SIM_S(12.0));
    sim_run_firmware(SIM_S(12.0));
    common_checks(true);
    check_logger("bursts");
    check_floor("bursts");

    float worst = 1e9f, best = 0;
    for (int k = 1; k < 5; k++)
    {
        double b = DRIVE_START_S + 1.5 * k + 1.0;
        float burst = drive_avg_w(b + 0.03, b + 0.25);      // After the inverter catches up
        worst = fminf(worst, burst);
        best = fmaxf(best, burst);
    }
    sim_note("bursts deliver %.1f .. %.1f kW (limit %.1f kW)", worst / 1000, best / 1000,
             (CFG_POWER_WINDOW_LIMITER_ENABLED ? WINDOW_LIMIT : CFG_INVERTER_POWER_LIMIT_W) / 1000);
    if (CFG_POWER_WINDOW_LIMITER_ENABLED)
    {
        SIM_CHECK(worst >= CFG_POWER_WINDOW_FLOOR_W && best <= WINDOW_LIMIT*1.01f,"bursts outside floor/ceiling: %.1f..%.1f kW",worst/1000,best/1000);
    }
}

// Random driving: requests from coasting to flat out, held 30-800 ms
static uint32_t demand_seed = 7;
static float    demand_level = 0;
static uint64_t demand_next = 0;
static float demand_random(uint64_t t)
{
    while (t >= demand_next)
    {
        static const float levels[] = { 0.0f, 20.0f, 60.0f, 120.0f, 170.0f, 220.0f, 300.0f };
        demand_seed = demand_seed * 1103515245u + 12345u;
        demand_level = levels[(demand_seed >> 16) % 7];
        demand_seed = demand_seed * 1103515245u + 12345u;
        demand_next = (demand_next ? demand_next : t) + 30000 + (demand_seed >> 16) % 770 * 1000;
    }
    return demand_level;
}

static void test_power_window_random(void)
{
    board_init(true);
    drive_start(demand_random, DRIVE_OCV_CELL, DRIVE_R_PACK, SIM_S(DRIVE_START_S), SIM_S(DRIVE_START_S + 35.0));
    sim_run_firmware(SIM_S(DRIVE_START_S + 35.0));
    common_checks(true);
    check_logger("random driving");
    check_floor("random driving");
    int above = 0;
    for (int i = 0; i < drive.n; i++) above += drive.power_w[i] > WINDOW_LIMIT + 1000.0f;
    sim_note("random driving: %d ms above %.0f kW, AMS fault output asserted %d times", above, WINDOW_LIMIT / 1000.0f,
             fault_asserts_between(first_release(), SIM_S(DRIVE_START_S + 35.0)));
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(DRIVE_START_S + 35.0)) == 0, "AMS faulted during random driving");
}

// Current swinging by 150 A through a 0.4 R pack moves U1 by 60 V between cell measurements. The
// U1 vs sum-of-cells plausibility check must not trip on that, and must still trip on a real U1 error.
static float swing_half_ms = 50;
static float demand_swings(uint64_t t)
{
    if (t < SIM_S(DRIVE_START_S)) return 0.0f;
    double s = (t - SIM_S(DRIVE_START_S)) / 1e6;
    static const float halves[] = { 35, 50, 70, 100, 150 };
    swing_half_ms = halves[(int)(s / 3.0) % 5];             // A different rate every 3 s
    return fmod(s * 1000.0, 2.0 * swing_half_ms) < swing_half_ms ? 40.0f : 190.0f;
}

static uint64_t offset_time = 0;
static void ev_u1_offset(void* arg)
{
    (void)arg;
    offset_time = sim_now_us();
    drive.demand = demand_sustained;                        // Steady current, so the check can see it
    drive.u1_offset_v = CFG_PACK_V_PLAUSIBILITY_V + 10.0f;
}

static void test_load_swings(void)
{
    board_init(true);
    drive_start(demand_swings, DRIVE_OCV_CELL, 0.4f, SIM_S(DRIVE_START_S), SIM_S(DRIVE_START_S + 17.0));
    sim_schedule(SIM_S(DRIVE_START_S + 15.0), ev_u1_offset, NULL);
    sim_run_firmware(SIM_S(DRIVE_START_S + 17.0));
    common_checks(true);
    int swings_faults = fault_asserts_between(first_release(), SIM_S(DRIVE_START_S + 15.0));
    sim_note("load swings: %d AMS faults, %d implausible measurements", swings_faults, sim_uart_count("IMPLAUSIBLE"));
    SIM_CHECK(swings_faults == 0, "AMS tripped %d times on load swings (flags 0x%02lX)", swings_faults,
              (unsigned long)BMS_Safety_GetActiveFlags());
    uint64_t t = first_fault_after(offset_time);
    SIM_CHECK(t != UINT64_MAX && t - offset_time <= FS_LIMIT_US, "U1 off by %.0f V: no fault within 500 ms",
              CFG_PACK_V_PLAUSIBILITY_V + 10.0f);
    if (t != UINT64_MAX) sim_note("U1 error: fault after %.0f ms", (t - offset_time) / 1000.0);
}


/* ===================================================================================== */
/*  Precharge                                                                             */
/* ===================================================================================== */

static void send_ts(bool activate)
{
    uint8_t data[8] = { activate ? 1 : 0 };
    sim_can_node_send(1, CFG_CAN_ID_TS_REQUEST, CFG_CAN_TS_REQUEST_EXTENDED, data, 8);
}

static void ev_ts_on(void* arg)  { (void)arg; send_ts(true); }
static void ev_ts_off(void* arg) { (void)arg; send_ts(false); }

static void expect_state(PrechargeState state, bool pre, bool byp, const char* when)
{
    SIM_CHECK(BMS_Precharge_GetState() == state, "%s: precharge state %d, expected %d", when, BMS_Precharge_GetState(), state);
    SIM_CHECK(precharge_on() == pre && bypass_on() == byp, "%s: precharge %d bypass %d, expected %d %d",
              when, precharge_on(), bypass_on(), pre, byp);
}

static void ev_pc_charging(void* arg) { (void)arg; expect_state(PRECHARGE_CHARGING, true, false, "precharging"); }
static void ev_pc_active(void* arg)   { (void)arg; expect_state(PRECHARGE_ACTIVE, false, true, "TS active"); }
static void ev_pc_idle(void* arg)     { (void)arg; expect_state(PRECHARGE_IDLE, false, false, "idle"); }
static void ev_pc_failed(void* arg)   { (void)arg; expect_state(PRECHARGE_FAILED, false, false, "failed"); }

static void test_precharge_nominal(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(4.0), ev_ts_on, NULL);
    sim_schedule(SIM_S(4.03), ev_pc_charging, NULL);
    sim_schedule(SIM_S(6.0), ev_pc_active, NULL);
    sim_schedule(SIM_S(7.0), ev_ts_off, NULL);
    sim_schedule(SIM_S(7.03), ev_pc_idle, NULL);
    sim_run_firmware(SIM_S(8));
    common_checks(true);

    SIM_CHECK(bypass_closures == 1, "bypass closed %d times", bypass_closures);
    double overlap_ms = ((double)precharge_off_time - (double)bypass_on_time) / 1000.0;
    SIM_CHECK(fabs(overlap_ms - CFG_PRECHARGE_OVERLAP_MS) <= CFG_SAFETY_TICK_MS + 1, "precharge opened %.1f ms after the bypass closed", overlap_ms);
    double t_close = (bypass_on_time - SIM_S(4.0)) / 1e6;
    sim_note("bypass closed %.3f s after the request at %.1f %% (tau 0.4 s), precharge opened %.1f ms later",
             t_close, sim_ivt_bypass_closed_ratio() * 100.0f, overlap_ms);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "precharge caused an AMS fault");
}

static void ev_resistor_ok(void* arg) { (void)arg; sim_ivt_set_precharge_tau(0.4f); }

static void test_precharge_timeout_retry(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_ivt_set_precharge_tau(0.0f);                            // Precharge resistor open
    sim_schedule(SIM_S(4.0), ev_ts_on, NULL);
    sim_schedule(SIM_S(4.0 + CFG_PRECHARGE_TIMEOUT_MS / 1000.0 - 0.1), ev_pc_charging, NULL);
    sim_schedule(SIM_S(4.0 + CFG_PRECHARGE_TIMEOUT_MS / 1000.0 + 0.1), ev_pc_failed, NULL);
    sim_schedule(SIM_S(10.0), ev_resistor_ok, NULL);
    sim_schedule(SIM_S(10.0), ev_ts_on, NULL);                  // Still "activate": must not restart
    sim_schedule(SIM_S(11.0), ev_pc_failed, NULL);
    sim_schedule(SIM_S(12.0), ev_ts_off, NULL);
    sim_schedule(SIM_S(12.5), ev_ts_on, NULL);                  // New request
    sim_schedule(SIM_S(14.5), ev_pc_active, NULL);
    sim_run_firmware(SIM_S(15));
    common_checks(true);
    SIM_CHECK(bypass_closures == 1, "bypass closed %d times", bypass_closures);
}

static void ev_cell_ov(void* arg)   { (void)arg; set_cell(SLAVE(7), 3, CFG_CELL_OV_V + 0.05f); }
static void ev_cell_ok(void* arg)   { (void)arg; set_cell(SLAVE(7), 3, 3.80f); }
static void ev_hot(void* arg)       { (void)arg; set_temp(SLAVE(3), 5, CFG_CELL_TEMP_MAX_C + 2.0f); }
static uint64_t fault_inject_time = 0;
static void ev_mark(void* arg)      { (void)arg; fault_inject_time = sim_now_us(); }

static void test_precharge_fault_abort(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(4.0), ev_ts_on, NULL);
    sim_schedule(SIM_S(4.3), ev_cell_ov, NULL);                 // Fault while precharging
    sim_schedule(SIM_S(4.9), ev_pc_idle, NULL);
    sim_schedule(SIM_S(6.0), ev_cell_ok, NULL);
    sim_schedule(SIM_S(9.0), ev_pc_idle, NULL);              // No restart without a new request
    sim_schedule(SIM_S(9.0), ev_ts_off, NULL);
    sim_schedule(SIM_S(9.2), ev_ts_on, NULL);
    sim_schedule(SIM_S(11.0), ev_pc_active, NULL);
    sim_schedule(SIM_S(11.5), ev_mark, NULL);
    sim_schedule(SIM_S(11.5), ev_hot, NULL);                    // Fault with the TS active
    sim_schedule(SIM_S(12.1), ev_pc_idle, NULL);
    sim_run_firmware(SIM_S(12.5));
    common_checks(true);
    uint64_t fault = first_fault_after(fault_inject_time);
    SIM_CHECK(fault != UINT64_MAX && fault - fault_inject_time <= FS_LIMIT_US, "no fault for the over-temperature with TS active");
}

static void ev_sdc_open(void* arg)   { (void)arg; sim_gpio_set_input(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN, !CFG_SDC_IN_ACTIVE_HIGH); }
static void ev_sdc_closed(void* arg) { (void)arg; sim_gpio_set_input(CFG_PIN_SDC_IN_PORT, CFG_PIN_SDC_IN_PIN, CFG_SDC_IN_ACTIVE_HIGH); }

static void test_precharge_sdc_open(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(4.0), ev_ts_on, NULL);
    sim_schedule(SIM_S(6.0), ev_pc_active, NULL);
    sim_schedule(SIM_S(6.5), ev_sdc_open, NULL);
    sim_schedule(SIM_S(6.52), ev_pc_idle, NULL);
    sim_schedule(SIM_S(7.0), ev_sdc_closed, NULL);
    sim_schedule(SIM_S(8.0), ev_pc_idle, NULL);              // Stays open after the SDC closes again
    sim_run_firmware(SIM_S(8.5));
    common_checks(true);
}

static void test_precharge_while_faulted(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(0.8), ev_ts_on, NULL);                   // AMS still faulted at start-up
    sim_schedule(SIM_S(1.0), ev_pc_idle, NULL);
    sim_schedule(SIM_S(5.0), ev_pc_idle, NULL);
    sim_run_firmware(SIM_S(5.5));
    common_checks(true);
    SIM_CHECK(bypass_closures == 0 && sim_uart_count("request ignored") == 1, "request during a fault not ignored");
}

static void ev_u1_bad(void* arg) { (void)arg; keep_u1_matched = false; sim_ivt_set_u1(100.0f); }

static void test_precharge_bad_battery_voltage(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(3.95), ev_u1_bad, NULL);                 // U1 reads 100 V: precharge target would be met at once
    sim_schedule(SIM_S(4.0), ev_ts_on, NULL);
    sim_schedule(SIM_S(6.0), ev_pc_failed, NULL);
    sim_run_firmware(SIM_S(6.5));
    common_checks(true);
    SIM_CHECK(bypass_closures == 0, "bypass closed against an implausible battery voltage");
}


// TS request abuse: random activate / deactivate frames (other payload bytes, DLC 1..15) at random gaps, the SDC
// flickering, and an AMS fault in between. The relay invariants (relay_hook) must hold throughout, both relays may
// only be closed together for the overlap, and the spam alone must not fault the AMS.
#ifndef TS_ABUSE_SEED
#define TS_ABUSE_SEED 3
#endif
static uint32_t ts_rng = TS_ABUSE_SEED;     // Other seeds: EXTRA_CFLAGS=-DTS_ABUSE_SEED=n
static uint32_t ts_rand(uint32_t n) { ts_rng = ts_rng * 1103515245u + 12345u; return (ts_rng >> 16) % n; }
static int ts_frames = 0, ts_sdc_flickers = 0;
static uint64_t both_on_since = 0, both_on_max = 0;

static void both_relays_hook(GPIO_TypeDef* port, uint16_t pin, bool level)
{
    (void)port; (void)pin; (void)level;
    bool both = precharge_on() && bypass_on();
    if (both && both_on_since == 0) both_on_since = sim_now_us();
    if (!both && both_on_since != 0)
    {
        if (sim_now_us() - both_on_since > both_on_max) both_on_max = sim_now_us() - both_on_since;
        both_on_since = 0;
    }
}

static void ev_ts_abuse(void* arg)
{
    (void)arg;
    uint32_t r = ts_rand(100);
    uint8_t data[8] = { 0 };
    uint8_t dlc = 8;
    if (r < 45)      data[0] = 1;
    else if (r < 80) data[0] = 0;
    else if (r < 90) data[0] = (uint8_t)(2 + ts_rand(254));        // Not 1: deactivate
    else if (r < 95) { data[0] = 1; dlc = (uint8_t)(1 + ts_rand(15)); }
    if (r < 95)
    {
        sim_can_node_send(1, CFG_CAN_ID_TS_REQUEST, CFG_CAN_TS_REQUEST_EXTENDED, data, dlc);
        ts_frames++;
    }
    else
    {
        ev_sdc_open(NULL);
        sim_schedule(sim_now_us() + SIM_MS(20 + ts_rand(280)), ev_sdc_closed, NULL);
        ts_sdc_flickers++;
    }
    // Mostly short gaps, sometimes long enough for a precharge to complete
    uint64_t gap = ts_rand(4) < 3 ? SIM_MS(1 + ts_rand(60)) : SIM_MS(1000 + ts_rand(1500));
    if (sim_now_us() + gap < SIM_S(20)) sim_schedule(sim_now_us() + gap, ev_ts_abuse, NULL);
}

static void ev_abuse_ov(void* arg)      { (void)arg; set_cell(SLAVE(1), 2, CFG_CELL_OV_V + 0.05f); }
static void ev_abuse_ov_gone(void* arg) { (void)arg; set_cell(SLAVE(1), 2, 3.80f); }

static void test_ts_request_abuse(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_gpio_add_write_hook(both_relays_hook);
    sim_schedule(SIM_S(4), ev_ts_abuse, NULL);
    sim_schedule(SIM_S(9), ev_abuse_ov, NULL);
    sim_schedule(SIM_S(10), ev_abuse_ov_gone, NULL);
    sim_schedule(SIM_S(20.5), ev_sdc_closed, NULL);
    sim_schedule(SIM_S(20.5), ev_ts_off, NULL);
    sim_schedule(SIM_S(20.6), ev_pc_idle, NULL);
    sim_run_firmware(SIM_S(21));
    common_checks(true);
    sim_note("%d TS frames, %d SDC flickers, %d bypass closures, both relays closed together for at most %.1f ms",
             ts_frames, ts_sdc_flickers, bypass_closures, both_on_max / 1000.0);
    SIM_CHECK(bypass_closures > 0, "no precharge completed during the abuse");
    SIM_CHECK(both_on_max <= SIM_MS(CFG_PRECHARGE_OVERLAP_MS + CFG_SAFETY_TICK_MS + 1),
              "both relays closed together for %.1f ms", both_on_max / 1000.0);
    uint64_t t = first_fault_after(SIM_S(9));
    SIM_CHECK(t != UINT64_MAX && t - SIM_S(9) <= FS_LIMIT_US, "over-voltage during the abuse: no fault within 500 ms");
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(21)) == 1, "%d AMS faults, expected only the over-voltage",
              fault_asserts_between(first_release(), SIM_S(21)));
}


// Every IVT result at the ends of its range (INT32_MIN / INT32_MAX, as a corrupted frame could carry) for 1 s,
// with the TS requested: everything computed from them (power, SoC, inverter limits, precharge ratio, CAN values)
// must stay defined (SANITIZE=1 checks integer and float-to-int overflow), the AMS faults, and recovers after.
// The IVT itself is off meanwhile: its own frames in between would make the values look like noise (over the
// limit less than a third of the time doesn't fault, by design).
static int ivt_extreme_phase = 0;
static void ev_ivt_extremes(void* arg)
{
    (void)arg;
    static const struct { uint32_t id; uint8_t mux; } results[] = {
        { CFG_IVT_ID_RESULT_I, 0x00 }, { CFG_IVT_ID_RESULT_U1, 0x01 }, { CFG_IVT_ID_RESULT_U2, 0x02 },
        { CFG_IVT_ID_RESULT_WH, 0x07 },
    };
    for (int r = 0; r < 4; r++)
    {
        uint32_t v = ((ivt_extreme_phase + r) & 1) ? (uint32_t)INT32_MAX : (uint32_t)INT32_MIN;
        uint8_t d[8] = { results[r].mux, 0x00, (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
        sim_can_node_send(2, results[r].id, false, d, 6);
    }
    ivt_extreme_phase++;
    if (sim_now_us() < SIM_S(5.0)) sim_schedule(sim_now_us() + SIM_MS(2), ev_ivt_extremes, NULL);
}

static void test_ivt_value_extremes(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(3.9), ev_ts_on, NULL);
    sim_schedule(SIM_S(4.0), ev_ivt_off, NULL);
    sim_schedule(SIM_S(4.0), ev_ivt_extremes, NULL);
    sim_schedule(SIM_S(5.0), ev_ivt_back, NULL);
    sim_run_firmware(SIM_S(9));
    common_checks(true);
    uint64_t trip = first_fault_after(SIM_S(4.0));
    SIM_CHECK(trip != UINT64_MAX && trip - SIM_S(4.0) <= FS_LIMIT_US, "no AMS fault for IVT values at the ends of their range");
    check_healthy("4 s after the extreme IVT values");
    SocState s = BMS_SoC_Get();
    SIM_CHECK(s.soc_percent >= 0.0f && s.soc_percent <= 100.0f && fabsf(s.used_Wh) < 100.0f, "SoC %.1f %%, used %.1f Wh",
              s.soc_percent, s.used_Wh);
}


/* ===================================================================================== */
/*  Runtime settings (GUI over CAN1)                                                      */
/* ===================================================================================== */

typedef struct { int32_t def, min, max; } setting_range;

// Worked out here from bms_config.h, independently of bms_settings.c
static setting_range setting_expect(int id)
{
    switch (id)
    {
    case SETTING_POWER_LIMIT:
        return (setting_range){ lroundf(CFG_INVERTER_POWER_LIMIT_W), lroundf(CFG_SETTING_POWER_MIN_W), lroundf(CFG_INVERTER_POWER_LIMIT_W) };
    case SETTING_DISCHARGE_CURRENT:
        return (setting_range){ lroundf(CFG_MAX_DISCHARGE_CURRENT_A * 10), lroundf(CFG_SETTING_CURRENT_MIN_A * 10),
                                lroundf(CFG_MAX_DISCHARGE_CURRENT_A * 10) };
    case SETTING_TEMP_LIMIT:
        return (setting_range){ lroundf(CFG_INVERTER_DERATE_END_C * 10), lroundf(CFG_SETTING_TEMP_LIMIT_MIN_C * 10),
                                lroundf(CFG_INVERTER_DERATE_END_C * 10) };
    case SETTING_REGEN_ENABLED:
        return (setting_range){ 1, 0, 1 };
    case SETTING_REGEN_CURRENT:
        return (setting_range){ lroundf(CFG_INVERTER_REGEN_CURRENT_A * 10), 0, lroundf(CFG_INVERTER_REGEN_CURRENT_A * 10) };
    case SETTING_CHARGE_CURRENT:
        return (setting_range){ lroundf(CFG_CHARGER_CURRENT_A * 10), 0, lroundf(CFG_CHARGER_CURRENT_MAX_A * 10) };
    case SETTING_CHARGE_VOLTAGE:
        return (setting_range){ lroundf(CFG_CHARGER_VOLTAGE_V * 10), lroundf(CFG_PACK_V_MIN * 10),
                                lroundf(CFG_CHARGER_VOLTAGE_MAX_V * 10) };
    default:
        return (setting_range){ 0, 0, 0 };
    }
}

typedef struct { uint8_t command, id, result, flags; int32_t value; uint64_t t_us; } setting_response;

static void send_setting(uint8_t command, uint8_t id, int32_t value)
{
    uint8_t d[8] = { command, id, CFG_CONFIG_VERSION, 0, (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24) };
    sim_can_node_send(1, CFG_CAN_ID_CONFIG_REQUEST, true, d, 8);
}

// Responses since a time, oldest first
static int setting_responses(uint64_t from, setting_response* out, int max)
{
    int n = 0;
    for (int i = 0; i < sim_can_log_count() && n < max; i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus != 1 || !f->from_mcu || f->id != CFG_CAN_ID_CONFIG_RESPONSE || f->t_us < from) continue;
        SIM_CHECK(f->ext && f->dlc == 8, "settings response: ext %d, dlc %d", f->ext, f->dlc);
        out[n++] = (setting_response){ f->data[0], f->data[1], f->data[2], f->data[3],
                                       (int32_t)((uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8) |
                                                 ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24)), f->t_us };
    }
    return n;
}

// Requests go out 10 ms apart, as the GUI paces them
typedef struct { uint8_t command, id; int32_t value; } setting_request;
static setting_request set_queue[16];
static int             set_queue_len = 0, set_queue_next = 0;
static uint64_t        set_batch_start = 0;

static void ev_send_next_setting(void* arg)
{
    (void)arg;
    setting_request r = set_queue[set_queue_next++];
    send_setting(r.command, r.id, r.value);
    if (set_queue_next < set_queue_len) sim_schedule(sim_now_us() + 10000, ev_send_next_setting, NULL);
}

static void send_settings(const setting_request* list, int count)
{
    memcpy(set_queue, list, count * sizeof(list[0]));
    set_queue_len = count;
    set_queue_next = 0;
    set_batch_start = sim_now_us();
    ev_send_next_setting(NULL);
}

// The responses to the last batch must match: command, setting, result, value, flags
static void expect_responses(const char* when, const setting_response* expect, int count)
{
    setting_response got[32];
    int n = setting_responses(set_batch_start, got, 32);
    SIM_CHECK(n == count, "%s: %d settings responses, expected %d", when, n, count);
    for (int i = 0; i < n && i < count; i++)
    {
        const setting_response* e = &expect[i];
        SIM_CHECK(got[i].command == e->command && got[i].id == e->id && got[i].result == e->result &&
                  got[i].value == e->value && got[i].flags == e->flags,
                  "%s: response %d is cmd %d setting %d result %d value %ld flags 0x%X, expected %d %d %d %ld 0x%X", when, i,
                  got[i].command, got[i].id, got[i].result, (long)got[i].value, got[i].flags,
                  e->command, e->id, e->result, (long)e->value, e->flags);
    }
}

static void read_all_expect(setting_response* out, const int32_t* values, uint8_t flags)
{
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        out[id - 1] = (setting_response){ SETTINGS_CMD_READ_ALL, (uint8_t)id, SETTINGS_OK, flags, values[id], 0 };
    }
}

static int32_t defaults[SETTING_COUNT];
static int32_t changed[SETTING_COUNT];

#define FLAGS_STORED_MODIFIED (SETTINGS_FLAG_STORED | SETTINGS_FLAG_MODIFIED)

static void ev_set_read_defaults(void* arg)
{
    (void)arg;
    setting_request r[] = { { SETTINGS_CMD_READ_ALL, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_write(void* arg)
{
    (void)arg;
    setting_response e[SETTING_COUNT - 1];
    read_all_expect(e, defaults, 0);
    expect_responses("built-in values", e, SETTING_COUNT - 1);

    // At most one response per fast period: a fast frame between every two
    setting_response got[16];
    int n = setting_responses(set_batch_start, got, 16);
    for (int i = 1; i < n; i++)
    {
        SIM_CHECK(count_frames(1, true, CFG_CAN_ID_FAST_1, got[i - 1].t_us, got[i].t_us) >= 1,
                  "two settings responses within one fast period (%.3f and %.3f s)", got[i - 1].t_us / 1e6, got[i].t_us / 1e6);
    }

    setting_request r[] = {
        { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, changed[SETTING_POWER_LIMIT] },
        { SETTINGS_CMD_WRITE, SETTING_DISCHARGE_CURRENT, changed[SETTING_DISCHARGE_CURRENT] },
        { SETTINGS_CMD_WRITE, SETTING_TEMP_LIMIT, changed[SETTING_TEMP_LIMIT] },
        { SETTINGS_CMD_WRITE, SETTING_REGEN_ENABLED, changed[SETTING_REGEN_ENABLED] },
        { SETTINGS_CMD_WRITE, SETTING_CHARGE_CURRENT, changed[SETTING_CHARGE_CURRENT] },
        { SETTINGS_CMD_WRITE, SETTING_CHARGE_VOLTAGE, changed[SETTING_CHARGE_VOLTAGE] },
        { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, setting_expect(SETTING_POWER_LIMIT).max + 1 },     // Above the built-in limit
        { SETTINGS_CMD_WRITE, SETTING_REGEN_CURRENT, -1 },
        { SETTINGS_CMD_WRITE, 0, 0 },
        { SETTINGS_CMD_READ, SETTING_COUNT, 0 },
        { 9, SETTING_POWER_LIMIT, 0 },
    };
    send_settings(r, sizeof(r) / sizeof(r[0]));
}

static void ev_set_check_writes(void* arg)
{
    (void)arg;
    const uint8_t f = SETTINGS_FLAG_UNSAVED | SETTINGS_FLAG_MODIFIED;
    setting_response e[] = {
        { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, SETTINGS_OK, f, changed[SETTING_POWER_LIMIT], 0 },
        { SETTINGS_CMD_WRITE, SETTING_DISCHARGE_CURRENT, SETTINGS_OK, f, changed[SETTING_DISCHARGE_CURRENT], 0 },
        { SETTINGS_CMD_WRITE, SETTING_TEMP_LIMIT, SETTINGS_OK, f, changed[SETTING_TEMP_LIMIT], 0 },
        { SETTINGS_CMD_WRITE, SETTING_REGEN_ENABLED, SETTINGS_OK, f, changed[SETTING_REGEN_ENABLED], 0 },
        { SETTINGS_CMD_WRITE, SETTING_CHARGE_CURRENT, SETTINGS_OK, f, changed[SETTING_CHARGE_CURRENT], 0 },
        { SETTINGS_CMD_WRITE, SETTING_CHARGE_VOLTAGE, SETTINGS_OK, f, changed[SETTING_CHARGE_VOLTAGE], 0 },
        { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, SETTINGS_ERR_RANGE, f, changed[SETTING_POWER_LIMIT], 0 },
        { SETTINGS_CMD_WRITE, SETTING_REGEN_CURRENT, SETTINGS_ERR_RANGE, f, defaults[SETTING_REGEN_CURRENT], 0 },
        { SETTINGS_CMD_WRITE, 0, SETTINGS_ERR_UNKNOWN_SETTING, f, 0, 0 },
        { SETTINGS_CMD_READ, SETTING_COUNT, SETTINGS_ERR_UNKNOWN_SETTING, f, 0, 0 },
        { 9, SETTING_POWER_LIMIT, SETTINGS_ERR_UNKNOWN_COMMAND, f, changed[SETTING_POWER_LIMIT], 0 },
    };
    expect_responses("writes", e, sizeof(e) / sizeof(e[0]));
}

// Limits with the changed settings, at 25 C or warm
static void check_setting_limits(const char* when, float temp_c)
{
    float u1 = sim_ivt_u1();
    float power_w = changed[SETTING_POWER_LIMIT];
    float base_a = changed[SETTING_DISCHARGE_CURRENT] / 10.0f;
    float shift = CFG_INVERTER_DERATE_END_C - changed[SETTING_TEMP_LIMIT] / 10.0f;
    float derated = expect_derated(base_a, temp_c, CFG_INVERTER_DERATE_START_C - shift, CFG_INVERTER_DERATE_END_C - shift,
                                   CFG_INVERTER_DERATE_CURRENT_A);
    float power_a = expect_current_for_power(power_w, u1);
    limit_expect e;
    e.discharge = fminf(derated, power_a);
    e.regen     = 0.0f;                                         // Regen switched off
    e.power_bit = power_a < derated;
    e.power     = e.power_bit ? power_w : derated * (u1 - CFG_PACK_RESISTANCE_MIN_OHM * (derated - LIMIT_TEST_CURRENT_A));
    e.temp      = temp_c > CFG_INVERTER_DERATE_START_C - shift;
    e.regen_bit = 0;
    e.fault     = 0;
    check_limits(when, e, LIMIT_TOL_TEMP_A, 1);
}

static void ev_set_check_effect(void* arg)
{
    (void)arg;
    check_setting_limits("power, current and regen settings", 25.0f);

    // Charger setpoints on CAN3 (big-endian 0.1 V / 0.1 A)
    const sim_can_frame* c = NULL;
    for (int i = 0; i < sim_can_log_count(); i++)
    {
        const sim_can_frame* f = sim_can_log(i);
        if (f->bus == 3 && f->from_mcu && f->id == CHARGER_CONFIG_CAN_ID) c = f;
    }
    SIM_CHECK(c != NULL, "no charger command on CAN3");
    if (c)
    {
        float volts=0,amps=0;SIM_CHECK(BMS_Charger_DecodeSetpoint(c->data,&volts,&amps),"invalid current-gateway BCD");
        int v=(int)lroundf(volts*10),a=(int)lroundf(amps*10);
        SIM_CHECK(v == changed[SETTING_CHARGE_VOLTAGE] && a == 0,
                  "charger command %d.%d V %d.%d A, expected the settings %ld, %ld (0.1 V, 0.1 A)", v / 10, v % 10, a / 10,
                  a % 10, (long)changed[SETTING_CHARGE_VOLTAGE], (long)changed[SETTING_CHARGE_CURRENT]);
    }
    set_temp(SLAVE(2), 1, changed[SETTING_TEMP_LIMIT] / 10.0f - 2.5f);    // Half way down the moved ramp
}

static void ev_set_check_warm(void* arg)
{
    (void)arg;
    check_setting_limits("temperature limit", changed[SETTING_TEMP_LIMIT] / 10.0f - 2.5f);
    set_temp(SLAVE(2), 1, 25.0f);
    sim_ivt_set_current(0.0f);
    send_ts(true);                                              // TS active: saving is refused
}

static void ev_set_save_ts(void* arg)
{
    (void)arg;
    SIM_CHECK(BMS_Precharge_IsTsActive(), "TS not active for the save test");
    setting_request r[] = { { SETTINGS_CMD_SAVE, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_save(void* arg)
{
    (void)arg;
    setting_response e[] = { { SETTINGS_CMD_SAVE, 0, SETTINGS_ERR_TS_ACTIVE, SETTINGS_FLAG_UNSAVED | SETTINGS_FLAG_MODIFIED, 0, 0 } };
    expect_responses("save with the TS active", e, 1);
    SIM_CHECK(sim_flash_erases() == 0, "flash erased with the TS active");
    send_ts(false);
}

static void ev_set_save_idle(void* arg)
{
    (void)arg;
    SIM_CHECK(!BMS_Precharge_IsTsActive(), "TS still active");
    setting_request r[] = { { SETTINGS_CMD_SAVE, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_defaults(void* arg)
{
    (void)arg;
    setting_response e[] = { { SETTINGS_CMD_SAVE, 0, SETTINGS_OK, FLAGS_STORED_MODIFIED, 0, 0 } };
    expect_responses("save", e, 1);
    SIM_CHECK(sim_flash_erases() == 1, "%d flash erases for one save", sim_flash_erases());
    setting_request r[] = { { SETTINGS_CMD_DEFAULTS, 0, 0 }, { SETTINGS_CMD_READ, SETTING_POWER_LIMIT, 0 } };
    send_settings(r, 2);
}

static void ev_set_reboot(void* arg)
{
    (void)arg;
    const uint8_t f = SETTINGS_FLAG_STORED | SETTINGS_FLAG_UNSAVED;
    setting_response e[] = { { SETTINGS_CMD_DEFAULTS, 0, SETTINGS_OK, f, 0, 0 },
                             { SETTINGS_CMD_READ, SETTING_POWER_LIMIT, SETTINGS_OK, f, defaults[SETTING_POWER_LIMIT], 0 } };
    expect_responses("defaults (not saved)", e, 2);
    BMS_Settings_Init();                                        // As at start-up: the saved settings come back
    setting_request r[] = { { SETTINGS_CMD_READ_ALL, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_corrupt(void* arg)
{
    (void)arg;
    setting_response e[SETTING_COUNT - 1];
    read_all_expect(e, changed, FLAGS_STORED_MODIFIED);
    expect_responses("saved settings after a restart", e, SETTING_COUNT - 1);
    sim_flash_corrupt(CFG_SETTINGS_FLASH_ADDR + 12);            // One bit of the power limit
    BMS_Settings_Init();
    setting_request r[] = { { SETTINGS_CMD_READ_ALL, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_write_error(void* arg)
{
    (void)arg;
    setting_response e[SETTING_COUNT - 1];
    read_all_expect(e, defaults, 0);
    expect_responses("corrupted flash", e, SETTING_COUNT - 1);
    sim_flash_set_write_error(true);
    setting_request r[] = { { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, changed[SETTING_POWER_LIMIT] }, { SETTINGS_CMD_SAVE, 0, 0 } };
    send_settings(r, 2);
}

static void ev_set_single_bank(void* arg)
{
    (void)arg;
    const uint8_t f = SETTINGS_FLAG_UNSAVED | SETTINGS_FLAG_MODIFIED;
    setting_response e[] = { { SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, SETTINGS_OK, f, changed[SETTING_POWER_LIMIT], 0 },
                             { SETTINGS_CMD_SAVE, 0, SETTINGS_ERR_FLASH, f, 0, 0 } };
    expect_responses("flash write error", e, 2);
    sim_flash_set_write_error(false);
    sim_flash_set_dual_bank(false);                             // 4 KB pages: the last one is erased
    setting_request r[] = { { SETTINGS_CMD_SAVE, 0, 0 } };
    send_settings(r, 1);
}

static void ev_set_single_bank_reboot(void* arg)
{
    (void)arg;
    setting_response e[] = { { SETTINGS_CMD_SAVE, 0, SETTINGS_OK, FLAGS_STORED_MODIFIED, 0, 0 } };
    expect_responses("save in single-bank mode", e, 1);
    BMS_Settings_Init();
    BmsSettings s = BMS_Settings_Get();
    SIM_CHECK(lroundf(s.powerLimit_W) == changed[SETTING_POWER_LIMIT] && s.modified,
              "single-bank save: power limit %.0f W after a restart", s.powerLimit_W);
}

// Save abuse (a GUI or script saving over and over): the settings page survives ~10,000 erases. Unchanged
// settings aren't written again, and changed ones at most once per CFG_SETTINGS_SAVE_INTERVAL_MS.
static int wear_erases[4];
static int wear_saves_left = 0;
static void ev_save(void* arg) { (void)arg; send_setting(SETTINGS_CMD_SAVE, 0, 0); }
static void ev_save_flood(void* arg)
{
    send_setting(SETTINGS_CMD_SAVE, 0, 0);
    if (--wear_saves_left > 0) sim_schedule(sim_now_us() + SIM_MS(20), ev_save_flood, arg);
}
static void ev_wear_count(void* arg) { *(int*)arg = sim_flash_erases(); }
static void ev_change_and_save(void* arg)
{
    send_setting(SETTINGS_CMD_WRITE, SETTING_POWER_LIMIT, *(const int32_t*)arg);
    sim_schedule(sim_now_us() + SIM_MS(10), ev_save, NULL);
}

static void test_settings_save_wear(void)
{
    static const int32_t power_a = 50000, power_b = 40000;
    board_init(true);
    wear_saves_left = 100;
    sim_schedule(SIM_S(3), ev_save_flood, NULL);                     // 100 saves over 2 s
    sim_schedule(SIM_S(5.5), ev_wear_count, &wear_erases[0]);
    sim_schedule(SIM_S(6), ev_change_and_save, (void*)&power_a);
    sim_schedule(SIM_S(6.5), ev_change_and_save, (void*)&power_b);   // Within the interval: refused
    sim_schedule(SIM_S(7.5), ev_wear_count, &wear_erases[1]);
    sim_schedule(SIM_S(8.5), ev_save, NULL);                         // After it: power_b saved
    sim_schedule(SIM_S(9), ev_wear_count, &wear_erases[2]);
    sim_run_firmware(SIM_S(9.5));
    common_checks(true);
    setting_response got[300];
    int n = setting_responses(SIM_S(6.4), got, 300), busy = 0;
    for (int i = 0; i < n; i++) busy += got[i].command == SETTINGS_CMD_SAVE && got[i].result == SETTINGS_ERR_BUSY;
    sim_note("erases: %d after 100 saves of the same settings, %d after two changes 0.5 s apart, %d after a later save",
             wear_erases[0], wear_erases[1], wear_erases[2]);
    SIM_CHECK(wear_erases[0] == 1, "100 saves of unchanged settings erased the page %d times", wear_erases[0]);
    SIM_CHECK(wear_erases[1] == 2 && busy == 1, "a second change 0.5 s later: %d erases, %d busy answers",
              wear_erases[1], busy);
    SIM_CHECK(wear_erases[2] == 3, "save after the interval: %d erases", wear_erases[2]);
}

// Classic CAN frames with DLC 9..15 (8 data bytes, ISO 11898-1) on every bus. The HAL copies DLCtoBytes[DLC] bytes,
// up to 64, into the receive buffer: it must take that, and the frames must work as 8-byte frames.
static int long_dlc_sent = 0, long_dlc_reads = 0;
static void ev_long_dlc(void* arg)
{
    (void)arg;
    uint8_t dlc = (uint8_t)(9 + long_dlc_sent % 7);
    static const uint8_t ts_off[8]      = { 0 };
    static const uint8_t not_alive[8]   = { 0xBE, 0, 0, 0, 0, 0, 0, 0 };
    static const uint8_t charger[8]     = { 0 };
    uint8_t read[8] = { SETTINGS_CMD_READ, SETTING_POWER_LIMIT, 0, 0, 0, 0, 0, 0 };
    switch (long_dlc_sent % 4)
    {
    case 0: sim_can_node_send(1, CFG_CAN_ID_TS_REQUEST, CFG_CAN_TS_REQUEST_EXTENDED, ts_off, dlc); break;
    case 1: sim_can_node_send(1, CFG_CAN_ID_CONFIG_REQUEST, true, read, dlc); long_dlc_reads++; break;
    case 2: sim_can_node_send(2, 0x511, false, not_alive, dlc); break;
    default: sim_can_node_send(3, CHARGER_STATUS_CAN_ID, true, charger, dlc); break;
    }
    long_dlc_sent++;
    if (sim_now_us() < SIM_S(7)) sim_schedule(sim_now_us() + SIM_MS(5), ev_long_dlc, NULL);
}

static void test_can_long_dlc(void)
{
    board_init(true);
    sim_schedule(SIM_S(5), ev_long_dlc, NULL);
    sim_run_firmware(SIM_S(8));
    common_checks(true);
    setting_response got[600];
    int n = setting_responses(SIM_S(5), got, 600), ok = 0;
    for (int i = 0; i < n; i++)
        ok += got[i].command == SETTINGS_CMD_READ && got[i].id == SETTING_POWER_LIMIT && got[i].result == SETTINGS_OK;
    sim_note("%d frames with DLC 9..15, %d settings reads among them, %d answered", long_dlc_sent, long_dlc_reads, ok);
    SIM_CHECK(ok == 0, "%d of %d settings reads with DLC 9..15 answered", ok, long_dlc_reads);
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(8)) == 0, "frames with DLC 9..15 caused a fault");
    SIM_CHECK(!BMS_Precharge_IsTsActive(), "TS activated");
    SIM_CHECK(sim_ivt_config_commands() == 8, "the BMS reconfigured the IVT (%d config commands)", sim_ivt_config_commands());
    SIM_CHECK(ivt_reading_matches(20.0f), "IVT reading disturbed");
}

// Debug UART ring buffer (10,000 bytes): a message crossing the end of the buffer while a transfer is running
// must still go out completely when the transfer is done, not wait for the next message (which may never come,
// e.g. the last words before a hang). Driven directly, without the firmware's main loop.
void MX_DMA_Init(void);
void MX_LPUART1_UART_Init(void);

static void test_uart_ring_wrap(void)
{
    MX_DMA_Init();
    MX_LPUART1_UART_Init();
    char fill[251], first[201], crossing[101];
    memset(fill, 'f', 249);      fill[249] = '\n';      fill[250] = 0;
    memset(first, 'a', 199);     first[199] = '\n';     first[200] = 0;
    memset(crossing, 'w', 99);   crossing[99] = '\n';   crossing[100] = 0;
    memcpy(crossing, "CROSSING", 8);
    for (int i = 0; i < 39; i++)
    {
        printfDma("%s", fill);
        sim_advance_us(5000);                   // Sent: 2.5 ms each at 1 Mbaud
    }
    printfDma("%s", first);                     // Bytes 9750..9949, transfer running for 2 ms
    printfDma("%s", crossing);                  // 9950..9999 and 0..49 while it runs
    sim_advance_us(SIM_MS(20));
    const char* text = sim_uart_text();
    size_t len = strlen(text);
    sim_note("%lu bytes sent of %d", (unsigned long)len, 39 * 250 + 200 + 100);
    SIM_CHECK(len == 39 * 250 + 200 + 100 && strcmp(text + len - 100, crossing) == 0,
              "message across the end of the ring buffer not sent (%lu of %d bytes)", (unsigned long)len,
              39 * 250 + 200 + 100);
    SIM_CHECK(droppedMessages == 0, "%lu messages dropped", (unsigned long)droppedMessages);
}

// Settings fuzz: random requests (any command, setting and value, DLC 0..15) every few ms for 10 s, with the TS
// active for a while. Every value in force and every value answered stays in its range, results and flags are
// valid codes, the flash is never written with the TS active and at most once per CFG_SETTINGS_SAVE_INTERVAL_MS.
#ifndef FUZZ_SEED
#define FUZZ_SEED 777
#endif
static uint32_t fuzz_rng = FUZZ_SEED;       // Other seeds: EXTRA_CFLAGS=-DFUZZ_SEED=n
static uint32_t fuzz_rand(uint32_t n) { fuzz_rng = fuzz_rng * 1103515245u + 12345u; return (fuzz_rng >> 8) % n; }
static int fuzz_sent = 0, fuzz_range_errors = 0, fuzz_erases_ts_on = -1, fuzz_erases_ts_off = -1;

static int32_t fuzz_value(int id)
{
    setting_range r = setting_expect(id);
    switch (fuzz_rand(8))
    {
    case 0:  return r.min;
    case 1:  return r.max;
    case 2:  return r.min - 1;
    case 3:  return r.max + 1;
    case 4:  return fuzz_rand(2) ? INT32_MAX : INT32_MIN;
    case 5:  return (int32_t)(fuzz_rand(65536) << 16 | fuzz_rand(65536));
    default: return r.max > r.min ? r.min + (int32_t)fuzz_rand((uint32_t)(r.max - r.min + 1)) : r.min;
    }
}

static void check_settings_in_range(void)
{
    BmsSettings s = BMS_Settings_Get();
    const float in_force[SETTING_COUNT] = { 0, s.powerLimit_W, s.dischargeCurrent_A * 10, s.tempLimit_C * 10,
                                            s.regenEnabled, s.regenCurrent_A * 10, s.chargeCurrent_A * 10,
                                            s.chargeVoltage_V * 10 };
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        setting_range r = setting_expect(id);
        if (in_force[id] < r.min - 0.01f || in_force[id] > r.max + 0.01f)
        {
            if (fuzz_range_errors++ == 0) sim_fail("setting %d in force: %.1f, outside %ld..%ld", id, in_force[id],
                                                   (long)r.min, (long)r.max);
        }
    }
}

static void ev_fuzz(void* arg)
{
    (void)arg;
    uint8_t d[8];
    uint32_t c = fuzz_rand(10);
    d[0] = (uint8_t)(c < 8 ? 1 + c % 5 : fuzz_rand(256));                    // Mostly real commands
    d[1] = (uint8_t)(fuzz_rand(10) < 8 ? 1 + fuzz_rand(SETTING_COUNT - 1) : fuzz_rand(256));
    d[2] = (uint8_t)fuzz_rand(256);
    d[3] = (uint8_t)fuzz_rand(256);
    int32_t v = fuzz_value(d[1] < SETTING_COUNT ? d[1] : 1);
    memcpy(&d[4], &v, 4);
    uint32_t k = fuzz_rand(20);
    uint8_t dlc = (uint8_t)(k < 16 ? 8 : (k < 18 ? 9 + fuzz_rand(7) : fuzz_rand(8)));
    sim_can_node_send(1, CFG_CAN_ID_CONFIG_REQUEST, true, d, dlc);
    fuzz_sent++;
    check_settings_in_range();
    if (sim_now_us() < SIM_S(14)) sim_schedule(sim_now_us() + SIM_MS(3 + fuzz_rand(37)), ev_fuzz, NULL);
}

static void ev_fuzz_ts_on(void* arg)  { (void)arg; send_ts(true); }
static void ev_fuzz_ts_active(void* arg) { (void)arg; fuzz_erases_ts_on = sim_flash_erases(); }
static void ev_fuzz_ts_off(void* arg) { (void)arg; fuzz_erases_ts_off = sim_flash_erases(); send_ts(false); }

static void test_settings_fuzz(void)
{
    board_init(true);
    sim_ivt_set_current(0.0f);
    sim_schedule(SIM_S(4), ev_fuzz, NULL);
    sim_schedule(SIM_S(7), ev_fuzz_ts_on, NULL);
    sim_schedule(SIM_S(8.6), ev_fuzz_ts_active, NULL);         // Precharge done: TS active
    sim_schedule(SIM_S(10), ev_fuzz_ts_off, NULL);
    sim_run_firmware(SIM_S(15));
    common_checks(true);
    check_settings_in_range();

    setting_response got[4000];
    int n = setting_responses(SIM_S(4), got, 4000), bad = 0, saves_ok = 0, busy = 0;
    for (int i = 0; i < n; i++)
    {
        const setting_response* r = &got[i];
        bool known = r->id > 0 && r->id < SETTING_COUNT;
        setting_range range = setting_expect(r->id);
        bool ok = r->result <= SETTINGS_ERR_MALFORMED &&
                  (!known || (r->value >= range.min && r->value <= range.max)) && (known || r->value == 0);
        if (!ok && bad++ < 3)
            sim_fail("response %d: command %d setting %d result %d flags 0x%02X value %ld", i, r->command, r->id,
                     r->result, r->flags, (long)r->value);
        saves_ok += r->command == SETTINGS_CMD_SAVE && r->result == SETTINGS_OK;
        busy += r->result == SETTINGS_ERR_BUSY;
    }
    int max_erases = (int)(10000 / CFG_SETTINGS_SAVE_INTERVAL_MS) + 1;
    sim_note("%d requests, %d responses, %d saves answered OK, %d busy, %d flash erases (at most %d)", fuzz_sent, n,
             saves_ok, busy, sim_flash_erases(), max_erases);
    SIM_CHECK(bad == 0, "%d invalid responses", bad);
    SIM_CHECK(fuzz_range_errors == 0, "settings in force left their range %d times", fuzz_range_errors);
    SIM_CHECK(BMS_Precharge_GetState() == PRECHARGE_IDLE, "precharge state %d at the end", BMS_Precharge_GetState());
    SIM_CHECK(fuzz_erases_ts_on >= 0 && fuzz_erases_ts_on == fuzz_erases_ts_off, "flash erased with the TS active (%d -> %d)",
              fuzz_erases_ts_on, fuzz_erases_ts_off);
    SIM_CHECK(sim_flash_erases() <= max_erases, "%d flash erases in 10 s", sim_flash_erases());
    SIM_CHECK(fault_asserts_between(first_release(), SIM_S(15)) == 0, "the fuzz caused an AMS fault");
}

// A valid settings record (right magic, layout and CRC) with values outside this build's ranges, as a build with
// other limits could leave it: those fall back to their built-in values, the in-range ones are kept, and the GUI
// sees the settings as stored, modified and unsaved.
static uint32_t test_crc32(const uint8_t* p, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return ~crc;
}

static int32_t foreign_values[SETTING_COUNT];

static void ev_foreign_record(void* arg)
{
    (void)arg;
    setting_range pw = setting_expect(SETTING_POWER_LIMIT), dc = setting_expect(SETTING_DISCHARGE_CURRENT),
                  rc = setting_expect(SETTING_REGEN_CURRENT), cc = setting_expect(SETTING_CHARGE_CURRENT),
                  cv = setting_expect(SETTING_CHARGE_VOLTAGE);
    foreign_values[SETTING_POWER_LIMIT]       = pw.max + 1000;         // Outside
    foreign_values[SETTING_DISCHARGE_CURRENT] = dc.min + 1;            // Kept
    foreign_values[SETTING_TEMP_LIMIT]        = INT32_MIN;             // Outside
    foreign_values[SETTING_REGEN_ENABLED]     = 0;                     // Kept
    foreign_values[SETTING_REGEN_CURRENT]     = rc.min - 5;            // Outside
    foreign_values[SETTING_CHARGE_CURRENT]    = cc.max;                // Kept
    foreign_values[SETTING_CHARGE_VOLTAGE]    = (cv.min + cv.max) / 2; // Kept

    // Layout of bms_settings.c's StoredSettings: magic, version, count, values[SETTING_COUNT], CRC-32, pad
    uint8_t rec[4 + 2 + 2 + 4 * SETTING_COUNT + 4 + 4];
    memset(rec, 0, sizeof(rec));
    uint32_t magic = 0x53534D42u;
    uint16_t version = CFG_CONFIG_VERSION, count = SETTING_COUNT;
    memcpy(&rec[0], &magic, 4);
    memcpy(&rec[4], &version, 2);
    memcpy(&rec[6], &count, 2);
    memcpy(&rec[8], foreign_values, 4 * SETTING_COUNT);
    uint32_t crc = test_crc32(rec, 8 + 4 * SETTING_COUNT);
    memcpy(&rec[8 + 4 * SETTING_COUNT], &crc, 4);

    FLASH_EraseInitTypeDef erase = { .TypeErase = FLASH_TYPEERASE_PAGES, .Banks = FLASH_BANK_2, .NbPages = 1,
                                     .Page = (CFG_SETTINGS_FLASH_ADDR - (FLASH_BASE + 0x40000u)) / 0x800u };
    uint32_t pageError;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &pageError);
    for (uint32_t i = 0; i < sizeof(rec) / 8; i++)
    {
        uint64_t w;
        memcpy(&w, &rec[8 * i], 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CFG_SETTINGS_FLASH_ADDR + 8 * i, w);
    }
    HAL_FLASH_Lock();

    BMS_Settings_Init();                        // As at start-up
    setting_request r[] = { { SETTINGS_CMD_READ_ALL, 0, 0 } };
    send_settings(r, 1);
}

static void ev_foreign_check(void* arg)
{
    (void)arg;
    int32_t expect[SETTING_COUNT];
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        setting_range r = setting_expect(id);
        expect[id] = (foreign_values[id] >= r.min && foreign_values[id] <= r.max) ? foreign_values[id] : r.def;
    }
    setting_response e[SETTING_COUNT - 1];
    read_all_expect(e, expect, SETTINGS_FLAG_STORED | SETTINGS_FLAG_MODIFIED | SETTINGS_FLAG_UNSAVED);
    expect_responses("record with values outside this build's ranges", e, SETTING_COUNT - 1);
}

static void test_settings_foreign_record(void)
{
    board_init(true);
    sim_schedule(SIM_S(3), ev_foreign_record, NULL);
    sim_schedule(SIM_S(4), ev_foreign_check, NULL);
    sim_run_firmware(SIM_S(4.5));
    common_checks(true);
    SIM_CHECK(sim_uart_count("3 saved values outside this build's range") == 1, "out-of-range values not reported");
}

static void test_settings(void)
{
    for (int id = 1; id < SETTING_COUNT; id++)
    {
        setting_range r = setting_expect(id);
        defaults[id] = changed[id] = r.def;
    }
    changed[SETTING_POWER_LIMIT]       = 60000;                 // 60 kW
    changed[SETTING_DISCHARGE_CURRENT] = 1500;                  // 150 A
    changed[SETTING_TEMP_LIMIT]        = 500;                   // 50 C
    changed[SETTING_REGEN_ENABLED]     = 0;
    changed[SETTING_CHARGE_CURRENT]    = 90;                   // 10 A
    changed[SETTING_CHARGE_VOLTAGE]    = lroundf(CFG_TOTAL_CELLS * 3.95f * 10);    // 3.95 V per cell (442.4 V for the car)

    board_init(true);
    set_all_cells(4.0f);                                        // 448 V for the car: 60 kW is below 150 A
    sim_schedule(SIM_S(3.0), ev_set_read_defaults, NULL);
    sim_schedule(SIM_S(3.3), ev_set_write, NULL);
    sim_schedule(SIM_S(3.6), ev_set_check_writes, NULL);
    sim_schedule(SIM_S(3.7), ev_set_check_effect, NULL);
    sim_schedule(SIM_S(4.4), ev_set_check_warm, NULL);
    sim_schedule(SIM_S(6.4), ev_set_save_ts, NULL);
    sim_schedule(SIM_S(6.6), ev_set_save, NULL);
    sim_schedule(SIM_S(7.0), ev_set_save_idle, NULL);
    sim_schedule(SIM_S(7.3), ev_set_defaults, NULL);
    sim_schedule(SIM_S(7.6), ev_set_reboot, NULL);
    sim_schedule(SIM_S(7.9), ev_set_corrupt, NULL);
    sim_schedule(SIM_S(8.2), ev_set_write_error, NULL);
    sim_schedule(SIM_S(10.3), ev_set_single_bank, NULL);       // CFG_SETTINGS_SAVE_INTERVAL_MS after the failed write
    sim_schedule(SIM_S(10.6), ev_set_single_bank_reboot, NULL);
    sim_run_firmware(SIM_S(10.8));
    common_checks(true);
    check_fast_timing();
    SIM_CHECK(canTxErrorCount == 0 && canFastTxDropped == 0, "%lu CAN frames could not be queued, %lu fast frames dropped",
              (unsigned long)canTxErrorCount, (unsigned long)canFastTxDropped);
    SIM_CHECK(sim_uart_count("Settings: built-in values (none saved)") >= 2, "no start-up message for empty or corrupted flash");
    SIM_CHECK(sim_uart_count("Settings: loaded from flash") >= 1, "no start-up message for saved settings");
    sim_note("%d flash erases, settings responses every fast period at most", sim_flash_erases());
}


/* ===================================================================================== */
/*  Randomised soak                                                                       */
/* ===================================================================================== */
/*
 * Minutes of operation with random healthy drift, harmless glitches (single bad samples, current
 * spikes, corrupted isoSPI reads, one-off IVT status bits) and real faults of every type at random
 * times and for random durations. Checked throughout:
 *  - every real fault has the fault output active (with the right flag) within 500 ms
 *  - the fault output never asserts outside a real fault
 *  - relay never on while faulted, watchdog never starved (common checks)
 */

static uint32_t rng_state = 1;

static uint32_t rnd(void)
{
    rng_state = rng_state * 1664525U + 1013904223U;
    return rng_state >> 8;
}

static float rnd_range(float lo, float hi)
{
    return lo + (hi - lo) * (float)(rnd() % 10001U) / 10000.0f;
}

static uint64_t rnd_us(double lo_s, double hi_s)
{
    return (uint64_t)(rnd_range((float)lo_s, (float)hi_s) * 1e6f);
}

typedef struct
{
    uint64_t          start, end;
    const fault_case* fc;
    bool              was_active_at_start;
} fault_window;

#define MAX_WINDOWS 1024
static fault_window windows[MAX_WINDOWS];
static int          window_count = 0;
static const fault_case* active_fault = NULL;
static uint64_t     soak_end = 0;
static uint64_t     soak_worst = 0;
static int          soak_transients = 0;

// Healthy drift avoids the cells and sensors the fault cases use
static const int drift_slaves[] = { 0, 1, 2, 4, 6, 8, 10, 13 };

static bool current_fault_active(void)
{
    return active_fault && (active_fault->flag == BMS_ERR_CURRENT || active_fault->flag == BMS_ERR_IVT);
}

static void ev_soak_drift(void* arg)
{
    (void)arg;
    int slave = drift_slaves[rnd() % (sizeof(drift_slaves) / sizeof(drift_slaves[0]))];
    switch (rnd() % 3)
    {
    case 0:
    {
        int c = (int)(rnd() % CFG_CELLS_PER_SLAVE);
        float v = sim_adbms_get_cell(slave, cell_channel[c] - 1) + rnd_range(-0.03f, 0.03f);
        if (v < 3.0f) v = 3.0f;
        if (v > 4.1f) v = 4.1f;
        set_cell(slave, c, v);
        break;
    }
    case 1:
        set_temp(slave, (int)(rnd() % CFG_TEMPS_PER_SLAVE), rnd_range(5.0f,CFG_CELL_CHARGE_TEMP_MAX_C-1.0f));
        break;
    default:
        if (!current_fault_active()) sim_ivt_set_current(ivt_reported(rnd_range(-fminf(150.0f,CFG_MAX_CHARGE_CURRENT_A-1.0f),CFG_MAX_DISCHARGE_CURRENT_A-10.0f)));
        break;
    }
    if (sim_now_us() < soak_end) sim_schedule(sim_now_us() + rnd_us(0.05, 0.4), ev_soak_drift, NULL);
}

// Single-sample glitch on slave 11 cell 2: set, then undo after the next normal S-ADC conversion
static bool     soak_glitch_armed = false;
static uint64_t soak_glitch_sadc = 0;
static float    soak_glitch_restore = 3.8f;

static void ev_soak_glitch_poll(void* arg)
{
    (void)arg;
    if (soak_glitch_armed && sim_adbms_last_sadc_us(SLAVE(11), 0) != soak_glitch_sadc)
    {
        set_cell(SLAVE(11), 2, soak_glitch_restore);
        soak_glitch_armed = false;
    }
    if (sim_now_us() < soak_end) sim_schedule(sim_now_us() + 1000, ev_soak_glitch_poll, NULL);
}

static void ev_status_clear(void* arg) { (void)arg; if (!active_fault || active_fault->flag != BMS_ERR_IVT) sim_ivt_set_status(0); }

static void ev_soak_transient(void* arg)
{
    (void)arg;
    switch (rnd() % 4)
    {
    case 0:
        if (!soak_glitch_armed)
        {
            soak_glitch_restore = sim_adbms_get_cell(SLAVE(11), cell_channel[2] - 1);
            soak_glitch_sadc = sim_adbms_last_sadc_us(SLAVE(11), 0);
            set_cell(SLAVE(11), 2, (rnd() & 1) ? CFG_CELL_OV_V + 0.1f : CFG_CELL_UV_V - 0.1f);
            soak_glitch_armed = true;
        }
        break;
    case 1:
        if (!current_fault_active())
        {
            sim_ivt_set_current(ivt_reported(rnd_range(250.0f, 450.0f)));
            sim_schedule(sim_now_us() + rnd_us(0.01, 0.04), ev_current_normal, NULL);
        }
        break;
    case 2:
        sim_adbms_corrupt_reads((int)(rnd() % CFG_NUM_SLAVES), 1);
        break;
    default:
        if (!current_fault_active())
        {
            sim_ivt_set_status(0x8);
            sim_schedule(sim_now_us() + 25000, ev_status_clear, NULL);
        }
        break;
    }
    soak_transients++;
    if (sim_now_us() < soak_end) sim_schedule(sim_now_us() + rnd_us(0.5, 3.0), ev_soak_transient, NULL);
}

static void ev_soak_fault_start(void* arg);

// VCU: re-requests TS whenever it's off and the AMS is OK, and sometimes switches it off
static int soak_ts_requests = 0;
static void ev_soak_vcu(void* arg)
{
    (void)arg;
    PrechargeState st = BMS_Precharge_GetState();
    if ((st == PRECHARGE_IDLE || st == PRECHARGE_FAILED) && !sim_fault_active())
    {
        send_ts(false);
        sim_schedule(sim_now_us() + 5000, ev_ts_on, NULL);
        soak_ts_requests++;
    }
    else if (st == PRECHARGE_ACTIVE && rnd() % 4 == 0)
    {
        send_ts(false);
    }
    if (sim_now_us() < soak_end) sim_schedule(sim_now_us() + rnd_us(1.0, 6.0), ev_soak_vcu, NULL);
}

static void ev_soak_fault_check(void* arg)
{
    fault_window* w = (fault_window*)arg;
    SIM_CHECK(sim_fault_active(), "soak: %s not faulted 500 ms after it started", w->fc->name);
    SIM_CHECK(BMS_Safety_GetActiveFlags() & w->fc->flag, "soak: %s flag 0x%02lX missing (flags 0x%02lX)",
              w->fc->name, (unsigned long)w->fc->flag, (unsigned long)BMS_Safety_GetActiveFlags());
    if (!w->was_active_at_start)
    {
        uint64_t t = first_fault_after(w->start);
        if (t != UINT64_MAX && t - w->start > soak_worst) soak_worst = t - w->start;
    }
}

static void ev_soak_fault_end(void* arg)
{
    fault_window* w = (fault_window*)arg;
    SIM_CHECK(sim_fault_active(), "soak: fault output released during %s", w->fc->name);
    w->fc->restore();
    w->end = sim_now_us();
    active_fault = NULL;
    if (sim_now_us() < soak_end - SIM_S(5)) sim_schedule(sim_now_us() + rnd_us(0.3, 4.0), ev_soak_fault_start, NULL);
}

static void ev_soak_fault_start(void* arg)
{
    (void)arg;
    if (window_count >= MAX_WINDOWS) return;
    const fault_case* fc = &fault_cases[rnd() % (sizeof(fault_cases) / sizeof(fault_cases[0]))];
    fault_window* w = &windows[window_count++];
    w->fc = fc;
    w->start = sim_now_us();
    w->end = UINT64_MAX;
    w->was_active_at_start = sim_fault_active();
    active_fault = fc;
    fc->inject();
    sim_schedule(w->start + FS_LIMIT_US + 1000, ev_soak_fault_check, w);
    sim_schedule(w->start + rnd_us(0.7, 2.5), ev_soak_fault_end, w);
}

static void run_soak(uint32_t seed, double seconds)
{
    rng_state = seed;
    soak_end = SIM_S(seconds);
    board_init(true);
    sim_schedule(SIM_S(3), ev_soak_drift, NULL);
    sim_schedule(SIM_S(3), ev_soak_glitch_poll, NULL);
    sim_schedule(SIM_S(3.5), ev_soak_transient, NULL);
    sim_schedule(SIM_S(4), ev_soak_fault_start, NULL);
    sim_schedule(SIM_S(3), ev_soak_vcu, NULL);
    sim_run_firmware(soak_end);
    common_checks(true);

    // Every assertion of the fault output must be explained by a real fault
    const sim_transition* list;
    int n = sim_fault_transitions(&list);
    uint64_t release = first_release();
    int unexplained = 0;
    for (int i = 0; i < n; i++)
    {
        if (!list[i].state || list[i].t_us <= release) continue;
        bool explained = false;
        for (int k = 0; k < window_count && !explained; k++)
        {
            uint64_t end = windows[k].end == UINT64_MAX ? soak_end : windows[k].end;
            explained = list[i].t_us >= windows[k].start && list[i].t_us <= end + 700000;
        }
        if (!explained)
        {
            unexplained++;
            sim_fail("soak: fault output asserted at %.3f s with no real fault active", list[i].t_us / 1e6);
        }
    }
    sim_note("soak seed %lu: %.0f s, %d real faults, %d harmless glitches, %d false trips, worst reaction %.1f ms",
             (unsigned long)seed, seconds, window_count, soak_transients, unexplained, soak_worst / 1000.0);
    sim_note("soak seed %lu: %d TS requests, %d precharges completed", (unsigned long)seed, soak_ts_requests, bypass_closures);
    SIM_CHECK(bypass_closures > 10, "only %d precharges completed during the soak", bypass_closures);
    check_fast_timing();
    SIM_CHECK(sim_can_rx_lost(2) == 0, "%d IVT frames lost in the CAN2 RX FIFOs", sim_can_rx_lost(2));
}

static void test_soak_1(void) { run_soak(1, 600); }
static void test_soak_2(void) { run_soak(2, 600); }
static void test_soak_3(void) { run_soak(3, 600); }


#include "integration_cases.h"

/* ===================================================================================== */
/*  Runner                                                                                */
/* ===================================================================================== */

typedef struct
{
    const char* name;
    void      (*fn)(void);
} test_entry;

static const test_entry tests[] = {
    { "numeric_and_imd_contract",test_numeric_and_imd_contract },
    { "limiter_boundaries",test_limiter_boundaries },
    { "charger_documented_codec",test_charger_documented_codec },
    { "versioned_config",test_versioned_config },
    { "transaction_contract",test_transaction_contract },
#if BMS_BOARD_PROFILE == BMS_PROFILE_BENCH
    { "bench_contract",test_bench_contract },
#endif
    { "boot",                      test_boot },
    { "boot_u1_disabled",          test_boot_u1_disabled },
    { "can_layout",                test_can_layout },
    { "glitch_immunity",           test_glitch_immunity },
    { "current_spike_immunity",    test_current_spike_immunity },
    { "pec_noise",                 test_pec_noise },
    { "balancing",                 test_balancing },
    { "balancing_hysteresis",      test_balancing_hysteresis },
    { "leds",                      test_leds },
    { "charging",                  test_charging },
    { "can_no_ack",                test_can_no_ack },
    { "can_bus_off",               test_can_bus_off },
    { "ivt_power_cycle",           test_ivt_power_cycle },
    { "ivt_power_cycle_u1_off",    test_ivt_power_cycle_u1_off },
    { "power_on_slaves_late",      test_power_on_slaves_late },
    { "power_on_ivt_late",         test_power_on_ivt_late },
    { "power_on_slaves_then_ivt",  test_power_on_slaves_then_ivt },
    { "power_on_ivt_then_slaves",  test_power_on_ivt_then_slaves },
    { "dropout_ivt_brief",         test_dropout_ivt_brief },
    { "dropout_ivt_long",          test_dropout_ivt_long },
    { "dropout_slave_brief",       test_dropout_slave_brief },
    { "dropout_slave_long",        test_dropout_slave_long },
    { "dropout_chain_brief",       test_dropout_chain_brief },
    { "isospi_chaos",              test_isospi_chaos },
    { "slave_reset_restored",      test_slave_reset_restored },
    { "isospi_lost_command",       test_isospi_lost_command },
    { "intermittent_overcurrent", test_intermittent_overcurrent },
    { "overcurrent_pulses",       test_overcurrent_pulses },
    { "current_glitches_periodic", test_current_glitches_periodic },
    { "intermittent_cell_overvoltage", test_intermittent_cell_overvoltage },
    { "can2_flood_low_priority",  test_can2_flood_low_priority },
    { "can2_flood_high_priority", test_can2_flood_high_priority },
    { "can2_junk_frames",         test_can2_junk_frames },
    { "ivt_energy_extremes",      test_ivt_energy_extremes },
    { "ivt_spurious_alive",       test_ivt_spurious_alive },
    { "ivt_stray_stop",           test_ivt_stray_stop },
    { "ivt_restart_command",      test_ivt_restart_command },
    { "ivt_config_interrupted",   test_ivt_config_interrupted },
    { "ivt_can_errors_cleared",   test_ivt_can_errors_cleared },
    { "ivt_system_error_kept",    test_ivt_system_error_kept },
    { "ivt_can_error_clear_limit",  test_ivt_can_error_clear_limit },
    { "ivt_can_errors_during_disturbance", test_ivt_can_errors_during_disturbance },
    { "uart_flood",                test_uart_flood },
    { "temperature_table_edges",   test_temperature_table_edges },
    { "soc_model",                 test_soc_model },
    { "soc_startup",               test_soc_startup },
    { "energy_tracking",           test_energy_tracking },
    { "fast_frames",               test_fast_frames },
    { "inverter_limits",           test_inverter_limits },
    { "power_window_sustained",    test_power_window_sustained },
    { "power_window_bursts",       test_power_window_bursts },
    { "power_window_random",       test_power_window_random },
    { "load_swings",               test_load_swings },
    { "settings",                  test_settings },
    { "settings_save_wear",        test_settings_save_wear },
    { "can_long_dlc",              test_can_long_dlc },
    { "uart_ring_wrap",            test_uart_ring_wrap },
    { "settings_fuzz",             test_settings_fuzz },
    { "settings_foreign_record",   test_settings_foreign_record },
    { "precharge_nominal",         test_precharge_nominal },
    { "precharge_timeout_retry",   test_precharge_timeout_retry },
    { "precharge_fault_abort",     test_precharge_fault_abort },
    { "precharge_sdc_open",        test_precharge_sdc_open },
    { "precharge_while_faulted",   test_precharge_while_faulted },
    { "precharge_bad_battery_voltage", test_precharge_bad_battery_voltage },
    { "ts_request_abuse",          test_ts_request_abuse },
    { "ivt_value_extremes",        test_ivt_value_extremes },
    { "soak_1",                    test_soak_1 },
    { "soak_2",                    test_soak_2 },
    { "soak_3",                    test_soak_3 },
};

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    if (argc < 2 || strcmp(argv[1], "--list") == 0)
    {
        for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) printf("%s\n", tests[i].name);
        for (size_t i = 0; i < sizeof(fault_cases) / sizeof(fault_cases[0]); i++) printf("fault_%s\n", fault_cases[i].name);
        return 0;
    }
    if (argc >= 3) out_dir = argv[2];

    const char* name = argv[1];
    char path[256];
    snprintf(path, sizeof(path), "%s/%s.uart.log", out_dir, name);
    FILE* log = fopen(path, "w");
    sim_uart_set_file(log);

    bool found = false;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        if (strcmp(name, tests[i].name) == 0)
        {
            tests[i].fn();
            found = true;
        }
    }
    for (size_t i = 0; i < sizeof(fault_cases) / sizeof(fault_cases[0]); i++)
    {
        if (strncmp(name, "fault_", 6) == 0 && strcmp(name + 6, fault_cases[i].name) == 0)
        {
            run_fault_case(&fault_cases[i]);
            found = true;
        }
    }
    if (log) fclose(log);

    if (!found)
    {
        printf("unknown test '%s'\n", name);
        return 2;
    }
    return sim_failure_count() == 0 ? 0 : 1;
}
