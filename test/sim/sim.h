/*
 * sim.h
 *
 * Board simulator used by the PC tests. The real firmware is linked against these fakes of the
 * STM32 HAL; time only advances when the firmware waits or talks to a peripheral, and interrupts
 * fire at those points (unless masked), like on the chip.
 */

#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define SIM_MS(x)   ((uint64_t)(x) * 1000ULL)
#define SIM_S(x)    ((uint64_t)((x) * 1000000.0))


/* ---- Time and scheduling ---------------------------------------------------------------- */

uint64_t sim_now_us(void);
void     sim_advance_us(uint64_t us);

typedef void (*sim_event_fn)(void* arg);
void sim_schedule(uint64_t at_us, sim_event_fn fn, void* arg);

// Generic timer used by the peripheral models
typedef struct sim_timer
{
    uint64_t due_us;
    bool     active;
    void   (*fire)(struct sim_timer* t);
    void*    ctx;
} sim_timer;
void sim_timer_start(sim_timer* t, uint64_t due_us);
void sim_timer_stop(sim_timer* t);

// Run the firmware's main() until the given time
void sim_run_firmware(uint64_t end_us);


/* ---- Interrupts ------------------------------------------------------------------------- */

typedef enum
{
    SIM_IRQ_DMA1_CH1,       // UART TX DMA done
    SIM_IRQ_FDCAN1,
    SIM_IRQ_TIM16,
    SIM_IRQ_FDCAN2,
    SIM_IRQ_FDCAN3,
    SIM_IRQ_COUNT
} sim_irq_t;

typedef void (*sim_irq_handler)(void);
void sim_irq_set_handler(sim_irq_t irq, sim_irq_handler handler);
void sim_irq_pend(sim_irq_t irq);
bool sim_in_isr(void);


/* ---- Test results ----------------------------------------------------------------------- */

void sim_fail(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void sim_note(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
int  sim_failure_count(void);
#define SIM_CHECK(cond, ...) do { if (!(cond)) sim_fail(__VA_ARGS__); } while (0)

// Called when the firmware reaches Error_Handler(); ends the run
extern int sim_error_handler_calls;


/* ---- GPIO ------------------------------------------------------------------------------- */

bool sim_gpio_output(GPIO_TypeDef* port, uint16_t pin);
void sim_gpio_set_input(GPIO_TypeDef* port, uint16_t pin, bool high);
bool sim_gpio_is_output(GPIO_TypeDef* port, uint16_t pin);

typedef void (*sim_gpio_hook)(GPIO_TypeDef* port, uint16_t pin, bool level);
void sim_gpio_add_write_hook(sim_gpio_hook hook);

// AMS outputs, decoded with the polarity from bms_config.h
bool sim_fault_active(void);
bool sim_tsac_ready(void);

typedef struct { uint64_t t_us; bool state; } sim_transition;
int                    sim_fault_transitions(const sim_transition** list);
int                    sim_ready_transitions(const sim_transition** list);
bool                   sim_ready_was_on_while_faulted(void);


/* ---- Watchdog ----------------------------------------------------------------------------- */

bool     sim_wdg_started(void);
uint64_t sim_wdg_max_gap_us(void);
uint64_t sim_wdg_timeout_us(void);      // Shortest timeout at the fastest LSI


/* ---- UART log ----------------------------------------------------------------------------- */

const char* sim_uart_text(void);
int         sim_uart_count(const char* needle);


/* ---- CAN ---------------------------------------------------------------------------------- */

typedef struct
{
    uint64_t t_us;
    int      bus;           // 1, 2 or 3
    uint32_t id;
    bool     ext;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     from_mcu;
    bool     rtr;           // Remote frame
} sim_can_frame;

int                  sim_can_log_count(void);
const sim_can_frame* sim_can_log(int index);
void                 sim_can_log_write_csv(const char* path);

// Another node sends a frame (subject to the MCU's acceptance filters). It waits for the bus and
// arbitrates against everything else pending: lowest identifier first.
void sim_can_node_send(int bus, uint32_t id, bool ext, const uint8_t* data, uint8_t dlc);
void sim_can_node_send_remote(int bus, uint32_t id, bool ext, uint8_t dlc);
// A babbling node: sends id back to back until until_us (wins arbitration against higher identifiers)
void sim_can_flood(int bus, uint32_t id, bool ext, uint8_t dlc, uint64_t until_us);
int  sim_can_flood_sent(int bus);
// Called for every frame the MCU puts on the bus
typedef void (*sim_can_listener)(const sim_can_frame* frame);
void sim_can_set_listener(int bus, sim_can_listener listener);

void     sim_can_set_ack(int bus, bool ack);    // false: no other node acknowledges
void     sim_can_force_bus_off(int bus);
uint32_t sim_can_bitrate(int bus);
int      sim_can_rx_lost(int bus);
uint64_t sim_can_busy_us(int bus);              // Bus time used by all frames so far (bus load)
bool     sim_can_irq_enabled(int bus);


/* ---- ADBMS6830B daisy chain ----------------------------------------------------------------- */

void     sim_adbms_init(int devices);
void     sim_adbms_set_cell(int dev, int ch, float volts);         // ch 0..15
float    sim_adbms_get_cell(int dev, int ch);
void     sim_adbms_set_gpio(int dev, int gpio, float volts);       // gpio 1..10
void     sim_adbms_set_die_temp(int dev, float celsius);
void     sim_adbms_set_open_wire(int dev, int ch, bool open);
#define  SIM_CHAIN_BROKEN_AT_MASTER  (-2)                            // Cable from the master to the first device
void     sim_adbms_set_broken_after(int dev);                      // -1: chain intact
void     sim_adbms_set_powered(int dev, bool powered);             // Off: unreachable from it on; on: in reset, asleep
void     sim_adbms_corrupt_reads(int dev, int count);
void     sim_adbms_lose_next_conversion(int dev);                  // Misses its next normal S-ADC conversion command
void     sim_adbms_set_spi_error(bool error);
uint8_t  sim_adbms_pwm(int dev, int ch);                           // ch 0..15
uint8_t  sim_adbms_dcto(int dev);
uint16_t sim_adbms_gpo(int dev);                                   // bit n = GPIO n+1
bool     sim_adbms_refon(int dev);
bool     sim_adbms_cadc_running(int dev);                           // C-ADC in continuous mode
int      sim_adbms_unknown_commands(void);
int      sim_adbms_lost_transactions(void);
uint64_t sim_adbms_last_sadc_us(int dev, int ow);                  // last S-ADC conversion start by OW mode
extern float sim_adbms_ow_open_ratio;                                  // Reading of an open input with its switch on


/* ---- IVT-S ------------------------------------------------------------------------------- */

void  sim_ivt_init(bool u1_enabled_by_default);
void  sim_ivt_set_current(float amps);          // As the IVT reports it (sensor's own sign)
void  sim_ivt_set_u1(float volts);
void  sim_ivt_set_status(uint8_t nibble);
void  sim_ivt_set_u23_open(bool open);       // U3 unconnected: 'any result has an error' bit set
void  sim_ivt_set_precharge_tau(float seconds);   // 0: precharge resistor open
bool  sim_ivt_channel_cyclic(int index);
float sim_ivt_u2(void);
float sim_ivt_u1(void);
double sim_ivt_energy_wh(void);
float sim_ivt_bypass_closed_ratio(void);    // Lowest U2/U1 at any bypass closure
int   sim_ivt_bypass_closures(void);
void  sim_ivt_set_connected(bool connected);
void  sim_ivt_power_cycle(void);
void  sim_ivt_set_startup_ms(int ms);       // Default 400 (datasheet maximum)
int   sim_ivt_config_commands(void);
int   sim_ivt_boot_commands(void);          // Commands sent to it while it was starting up (and lost)
void  sim_ivt_command(const uint8_t* data); // Another node sends it a command (8 bytes)
void  sim_ivt_set_command_hook(void (*hook)(const uint8_t* data));  // Each command it takes
bool  sim_ivt_running(void);                // Sending results (connected, run mode, started up)
void  sim_ivt_log_system_error(int bit);    // Logs a system error (bit as in its bitmask: 2 CAN receive, 3 CAN
                                            // transmit, 12 ADC clock, 14 reset watchdog, ...): status bit 3 latched
int   sim_ivt_system_error_count(int bit);
int   sim_ivt_error_resets(void);           // Reset Error- and Logdata commands it carried out
bool  sim_ivt_u1_cyclic(void);
int   sim_ivt_config_violations(void);      // Run mode entered with a configuration the datasheet forbids
float sim_ivt_messages_per_s(void);         // Result messages per second of the current configuration

/* ---- Flash (runtime settings area, the last 4 KB) ----------------------------------------- */

void sim_flash_set_dual_bank(bool dual);    // Default: dual bank (2 KB pages), as from the factory
void sim_flash_corrupt(uint32_t address);   // Flips a bit
void sim_flash_set_write_error(bool error); // Programming fails
int  sim_flash_erases(void);

#endif /* SIM_H */
