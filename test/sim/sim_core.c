/*
 * sim_core.c
 *
 * Simulated time, interrupts, watchdog, GPIO, timers, UART and the remaining HAL calls.
 */

#include "sim.h"
#include "tim.h"
#include "usart.h"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* ---- Peripheral register blocks the firmware touches directly ---------------------------- */

TIM_TypeDef         sim_TIM2, sim_TIM3, sim_TIM15, sim_TIM16;
SPI_TypeDef         sim_SPI1;
FDCAN_GlobalTypeDef sim_FDCAN1, sim_FDCAN2, sim_FDCAN3;
USART_TypeDef       sim_LPUART1;
IWDG_TypeDef        sim_IWDG;
DBGMCU_TypeDef      sim_DBGMCU;
RCC_TypeDef         sim_RCC;

uint32_t SystemCoreClock = 128000000;

#define APB_TIMER_CLOCK_HZ  128000000ULL


/* ---- Time --------------------------------------------------------------------------------- */

static uint64_t now_us = 0;
static uint64_t end_us = 0;
static jmp_buf  end_jump;
static bool     running = false;

#define MAX_TIMERS 64
static sim_timer* timers[MAX_TIMERS];
static int        timer_count = 0;

uint64_t sim_now_us(void)
{
    return now_us;
}

void sim_timer_start(sim_timer* t, uint64_t due_us)
{
    t->due_us = due_us;
    t->active = true;
    for (int i = 0; i < timer_count; i++)
    {
        if (timers[i] == t) return;
    }
    if (timer_count >= MAX_TIMERS)
    {
        fprintf(stderr, "sim: too many timers\n");
        exit(2);
    }
    timers[timer_count++] = t;
}

void sim_timer_stop(sim_timer* t)
{
    t->active = false;
}


/* ---- Scheduled test events ----------------------------------------------------------------- */

typedef struct
{
    sim_timer    timer;
    sim_event_fn fn;
    void*        arg;
    bool         used;
} sim_event;

#define MAX_EVENTS 512
static sim_event events[MAX_EVENTS];

static void remove_timer(sim_timer* t)
{
    for (int i = 0; i < timer_count; i++)
    {
        if (timers[i] == t)
        {
            timers[i] = timers[--timer_count];
            return;
        }
    }
}

static void fire_event(sim_timer* t)
{
    sim_event* e = (sim_event*)t->ctx;
    t->active = false;
    remove_timer(t);
    e->used = false;
    e->fn(e->arg);
}

void sim_schedule(uint64_t at_us, sim_event_fn fn, void* arg)
{
    for (int i = 0; i < MAX_EVENTS; i++)
    {
        sim_event* e = &events[i];
        if (e->used) continue;
        e->used = true;
        e->fn = fn;
        e->arg = arg;
        e->timer.fire = fire_event;
        e->timer.ctx = e;
        sim_timer_start(&e->timer, at_us);
        return;
    }
    fprintf(stderr, "sim: too many pending events\n");
    exit(2);
}


/* ---- Results -------------------------------------------------------------------------------- */

static int failures = 0;
int sim_error_handler_calls = 0;

void sim_fail(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    printf("  FAIL [t=%.3f s] ", now_us / 1e6);
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
    failures++;
}

void sim_note(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    printf("  note [t=%.3f s] ", now_us / 1e6);
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
}

int sim_failure_count(void)
{
    return failures;
}


/* ---- Interrupts ----------------------------------------------------------------------------- */

static uint32_t        primask = 0;
static bool            in_isr = false;
static bool            pending[SIM_IRQ_COUNT];
static sim_irq_handler handlers[SIM_IRQ_COUNT];

void sim_irq_set_handler(sim_irq_t irq, sim_irq_handler handler)
{
    handlers[irq] = handler;
}

void sim_irq_pend(sim_irq_t irq)
{
    pending[irq] = true;
}

bool sim_in_isr(void)
{
    return in_isr;
}

// All firmware interrupts share priority 0, so they never nest; lower IRQ numbers go first
static void service_interrupts(void)
{
    if (primask || in_isr) return;

    bool any = true;
    while (any)
    {
        any = false;
        for (int i = 0; i < SIM_IRQ_COUNT; i++)
        {
            if (pending[i] && handlers[i])
            {
                pending[i] = false;
                in_isr = true;
                handlers[i]();
                in_isr = false;
                any = true;
                break;
            }
        }
    }
}

void __enable_irq(void)
{
    primask = 0;
    service_interrupts();
}

void __disable_irq(void)
{
    primask = 1;
}

uint32_t __get_PRIMASK(void)
{
    return primask;
}

void __set_PRIMASK(uint32_t value)
{
    primask = value;
    if (!value) service_interrupts();
}

uint32_t __get_IPSR(void)
{
    return in_isr ? 16U : 0U;
}


/* ---- Watchdog (IWDG register writes are sampled as time advances) -------------------------- */

static bool     wdg_running = false;
static uint64_t wdg_last_kick = 0;
static uint64_t wdg_max_gap = 0;
static bool     wdg_reported = false;

uint64_t sim_wdg_timeout_us(void)
{
    // (RLR + 1) counts of 4 * 2^PR LSI cycles, at the fastest LSI the datasheet allows (34 kHz)
    uint64_t prescaler = 4ULL << (sim_IWDG.PR & 0x7);
    return (sim_IWDG.RLR + 1ULL) * prescaler * 1000000ULL / 34000ULL;
}

static void wdg_poll(void)
{
    if (sim_IWDG.KR == 0xAAAA)
    {
        sim_IWDG.KR = 0;
        if (!wdg_running && sim_IWDG.RLR != 0)
        {
            wdg_running = true;
        }
        else if (wdg_running)
        {
            uint64_t gap = now_us - wdg_last_kick;
            if (gap > wdg_max_gap) wdg_max_gap = gap;
        }
        wdg_last_kick = now_us;
    }

    if (wdg_running && !wdg_reported && now_us - wdg_last_kick > sim_wdg_timeout_us())
    {
        wdg_reported = true;
        sim_fail("watchdog would reset the MCU: no kick for %.1f ms (timeout %.1f ms)",
                 (now_us - wdg_last_kick) / 1000.0, sim_wdg_timeout_us() / 1000.0);
    }
}

bool sim_wdg_started(void)
{
    return wdg_running;
}

uint64_t sim_wdg_max_gap_us(void)
{
    return wdg_max_gap;
}


/* ---- Advancing time -------------------------------------------------------------------------- */

static void check_end(void)
{
    if (running && !in_isr && now_us >= end_us)
    {
        longjmp(end_jump, 1);
    }
}

void sim_advance_us(uint64_t us)
{
    uint64_t target = now_us + us;

    for (;;)
    {
        sim_timer* next = NULL;
        for (int i = 0; i < timer_count; i++)
        {
            sim_timer* t = timers[i];
            if (t->active && t->due_us <= target && (!next || t->due_us < next->due_us))
            {
                next = t;
            }
        }
        if (!next) break;

        if (next->due_us > now_us) now_us = next->due_us;
        next->active = false;
        next->fire(next);
        wdg_poll();
        service_interrupts();
    }

    /* A callback can consume time through HAL reads; never rewind the clock. */
    if(now_us<target)now_us=target;
    wdg_poll();
    service_interrupts();
    check_end();
}

void sim_run_firmware(uint64_t run_until_us)
{
    extern int firmware_main(void);

    end_us = run_until_us;
    running = true;
    if (setjmp(end_jump) == 0)
    {
        firmware_main();
        sim_fail("firmware main() returned");
    }
    running = false;
    in_isr = false;
}


/* ---- Error handler (firmware copy in main.c is renamed for the simulation build) ------------ */

void Error_Handler(void)
{
    sim_error_handler_calls++;
    sim_fail("Error_Handler() called");
    if (running)
    {
        longjmp(end_jump, 1);
    }
}


/* ---- HAL core ------------------------------------------------------------------------------- */

HAL_StatusTypeDef HAL_Init(void)
{
    HAL_MspInit();
    return HAL_OK;
}

// SIM_TICK_OFFSET_MS (test/run_tests.sh TICK_WRAP=<s>) starts the tick just before it wraps, as after 49.7 days
#ifndef SIM_TICK_OFFSET_MS
#define SIM_TICK_OFFSET_MS 0u
#endif

uint32_t HAL_GetTick(void)
{
    sim_advance_us(2);      // Busy-wait loops poll the tick; let time move
    return (uint32_t)(now_us / 1000ULL) + (uint32_t)SIM_TICK_OFFSET_MS;
}

void HAL_Delay(uint32_t delay)
{
    uint32_t start = HAL_GetTick();
    uint32_t wait = delay;
    if (wait < HAL_MAX_DELAY) wait += 1;   // Same as the real HAL_Delay
    while (HAL_GetTick() - start < wait)
    {
        sim_advance_us(100);
    }
}

void HAL_IncTick(void) {}

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef* init) { (void)init; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef* init, uint32_t latency) { (void)init; (void)latency; return HAL_OK; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef* init) { (void)init; return HAL_OK; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return SystemCoreClock; }
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t scaling) { (void)scaling; return HAL_OK; }
void HAL_PWREx_DisableUCPDDeadBattery(void) {}

static bool irq_enabled[256];

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub) { (void)irq; (void)pre; (void)sub; }
void HAL_NVIC_EnableIRQ(IRQn_Type irq)  { irq_enabled[(int)irq & 0xFF] = true; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { irq_enabled[(int)irq & 0xFF] = false; }

bool sim_nvic_enabled(IRQn_Type irq)
{
    return irq_enabled[(int)irq & 0xFF];
}


/* ---- GPIO ----------------------------------------------------------------------------------- */

#define NUM_PORTS 7

typedef struct
{
    bool level[16];
    bool input_level[16];
    bool is_output[16];
} sim_port;

static sim_port ports[NUM_PORTS];

#define MAX_GPIO_HOOKS 8
static sim_gpio_hook gpio_hooks[MAX_GPIO_HOOKS];
static int           gpio_hook_count = 0;

#define MAX_TRANSITIONS 4096
static sim_transition fault_log[MAX_TRANSITIONS];
static int            fault_log_count = 0;
static sim_transition ready_log[MAX_TRANSITIONS];
static int            ready_log_count = 0;
static bool           ready_on_while_faulted = false;

static int port_index(GPIO_TypeDef* port)
{
    GPIO_TypeDef* list[NUM_PORTS] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG };
    for (int i = 0; i < NUM_PORTS; i++)
    {
        if (list[i] == port) return i;
    }
    fprintf(stderr, "sim: unknown GPIO port\n");
    exit(2);
}

static int pin_index(uint16_t pin)
{
    for (int i = 0; i < 16; i++)
    {
        if (pin == (1U << i)) return i;
    }
    fprintf(stderr, "sim: GPIO call with several pins (0x%04X)\n", pin);
    exit(2);
}

void sim_gpio_add_write_hook(sim_gpio_hook hook)
{
    gpio_hooks[gpio_hook_count++] = hook;
}

bool sim_fault_active(void)
{
    bool level = ports[port_index(CFG_PIN_FAULT_PORT)].level[pin_index(CFG_PIN_FAULT_PIN)];
    return level == CFG_FAULT_ACTIVE_HIGH;
}

bool sim_tsac_ready(void)
{
    int p = port_index(CFG_PIN_TSAC_READY_PORT);
    int n = pin_index(CFG_PIN_TSAC_READY_PIN);
    if (!ports[p].is_output[n]) return false;           // Floating: the PCB pull-down holds it off
    return ports[p].level[n] == CFG_TSAC_READY_ACTIVE_HIGH;
}

static void record_outputs(void)
{
    bool fault = sim_fault_active();
    bool ready = sim_tsac_ready();

    if (fault_log_count == 0 || fault_log[fault_log_count - 1].state != fault)
    {
        if (fault_log_count < MAX_TRANSITIONS)
            fault_log[fault_log_count++] = (sim_transition){ now_us, fault };
    }
    if (ready_log_count == 0 || ready_log[ready_log_count - 1].state != ready)
    {
        if (ready_log_count < MAX_TRANSITIONS)
            ready_log[ready_log_count++] = (sim_transition){ now_us, ready };
    }
    if (fault && ready)
    {
        ready_on_while_faulted = true;
    }
}

int sim_fault_transitions(const sim_transition** list)
{
    *list = fault_log;
    return fault_log_count;
}

int sim_ready_transitions(const sim_transition** list)
{
    *list = ready_log;
    return ready_log_count;
}

bool sim_ready_was_on_while_faulted(void)
{
    return ready_on_while_faulted;
}

bool sim_gpio_output(GPIO_TypeDef* port, uint16_t pin)
{
    return ports[port_index(port)].level[pin_index(pin)];
}

bool sim_gpio_is_output(GPIO_TypeDef* port, uint16_t pin)
{
    return ports[port_index(port)].is_output[pin_index(pin)];
}

void sim_gpio_set_input(GPIO_TypeDef* port, uint16_t pin, bool high)
{
    ports[port_index(port)].input_level[pin_index(pin)] = high;
}

static void gpio_init_one(GPIO_TypeDef* port, int n, uint32_t mode, uint32_t pull)
{
    sim_port* p = &ports[port_index(port)];
    bool output = (mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD);
    p->is_output[n] = output;
    if (!output && pull == GPIO_PULLUP && !p->input_level[n])
    {
        // Pulled up unless something drives it; tests set input levels explicitly
    }
}

void HAL_GPIO_Init(GPIO_TypeDef* port, GPIO_InitTypeDef* init)
{
    for (int n = 0; n < 16; n++)
    {
        if (init->Pin & (1U << n)) gpio_init_one(port, n, init->Mode, init->Pull);
    }
    record_outputs();
}

void HAL_GPIO_DeInit(GPIO_TypeDef* port, uint32_t pins)
{
    sim_port* p = &ports[port_index(port)];
    for (int n = 0; n < 16; n++)
    {
        if (pins & (1U << n)) p->is_output[n] = false;
    }
    record_outputs();
}

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pins, GPIO_PinState state)
{
    sim_port* p = &ports[port_index(port)];
    for (int n = 0; n < 16; n++)
    {
        if (!(pins & (1U << n))) continue;
        bool level = (state == GPIO_PIN_SET);
        bool changed = p->level[n] != level;
        p->level[n] = level;
        if (changed)
        {
            for (int h = 0; h < gpio_hook_count; h++) gpio_hooks[h](port, (uint16_t)(1U << n), level);
        }
    }
    record_outputs();
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin)
{
    sim_port* p = &ports[port_index(port)];
    int n = pin_index(pin);
    bool level = p->is_output[n] ? p->level[n] : p->input_level[n];
    return level ? GPIO_PIN_SET : GPIO_PIN_RESET;
}


/* ---- Timers ----------------------------------------------------------------------------------- */

static sim_timer tim16_tick;
static uint64_t  tim2_start_us = 0;

static void tim16_fire(sim_timer* t)
{
    uint64_t period_us = (htim16.Init.Prescaler + 1ULL) * (htim16.Init.Period + 1ULL) * 1000000ULL / APB_TIMER_CLOCK_HZ;
    sim_timer_start(t, t->due_us + period_us);
    sim_irq_pend(SIM_IRQ_TIM16);
}

static void tim16_isr(void)
{
    HAL_TIM_PeriodElapsedCallback(&htim16);
}

uint32_t sim_tim_get_counter(const TIM_HandleTypeDef* htim)
{
    if (htim == &htim2)
    {
        sim_advance_us(1);      // Busy-wait on the counter
        // TIM2 prescaler 128 -> 1 MHz
        uint64_t ticks_per_us = APB_TIMER_CLOCK_HZ / (htim2.Init.Prescaler + 1ULL) / 1000000ULL;
        return (uint32_t)((now_us - tim2_start_us) * ticks_per_us);
    }
    return 0;
}

void sim_tim_set_counter(TIM_HandleTypeDef* htim, uint32_t value)
{
    if (htim == &htim2 && value == 0) tim2_start_us = now_us;
}

HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef* htim) { HAL_TIM_Base_MspInit(htim); return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef* htim) { (void)htim; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef* htim) { (void)htim; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef* htim, const TIM_ClockConfigTypeDef* c) { (void)htim; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef* htim, const TIM_SlaveConfigTypeDef* c) { (void)htim; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef* htim, const TIM_IC_InitTypeDef* c, uint32_t ch) { (void)htim; (void)c; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef* htim, const TIM_OC_InitTypeDef* c, uint32_t ch) { (void)htim; (void)c; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, const TIM_MasterConfigTypeDef* c) { (void)htim; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t ch) { (void)htim; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef* htim, uint32_t ch) { (void)htim; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef* htim, uint32_t ch) { (void)htim; (void)ch; return HAL_OK; }
uint32_t HAL_TIM_ReadCapturedValue(const TIM_HandleTypeDef* htim, uint32_t ch) { return ch==TIM_CHANNEL_1 ? htim->Instance->CCR1 : htim->Instance->CCR2; }

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef* htim)
{
    if (htim == &htim2) tim2_start_us = now_us;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef* htim)
{
    (void)htim;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef* htim)
{
    if (htim == &htim16)
    {
        uint64_t period_us = (htim16.Init.Prescaler + 1ULL) * (htim16.Init.Period + 1ULL) * 1000000ULL / APB_TIMER_CLOCK_HZ;
        tim16_tick.fire = tim16_fire;
        sim_irq_set_handler(SIM_IRQ_TIM16, tim16_isr);
        sim_timer_start(&tim16_tick, now_us + period_us);
    }
    return HAL_OK;
}


/* ---- UART with DMA (debug log) ---------------------------------------------------------------- */

#define UART_LOG_SIZE (8 * 1024 * 1024)
static char*     uart_log = NULL;
static size_t    uart_log_len = 0;
static sim_timer uart_done;
static FILE*     uart_file = NULL;

static void uart_dma_isr(void)
{
    hlpuart1.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&hlpuart1);
}

static void uart_done_fire(sim_timer* t)
{
    (void)t;
    sim_irq_pend(SIM_IRQ_DMA1_CH1);
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef* huart)
{
    HAL_UART_MspInit(huart);
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef* h, uint32_t t) { (void)h; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef* h, uint32_t t) { (void)h; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef* h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef* h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef* h) { (void)h; return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t size)
{
    if (huart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    if (size == 0) return HAL_ERROR;

    if (!uart_log) uart_log = calloc(1, UART_LOG_SIZE);
    if (uart_log_len + size < UART_LOG_SIZE - 1)
    {
        memcpy(uart_log + uart_log_len, data, size);
        uart_log_len += size;
    }
    if (uart_file) fwrite(data, 1, size, uart_file);

    huart->gState = HAL_UART_STATE_BUSY_TX;
    uart_done.fire = uart_done_fire;
    sim_irq_set_handler(SIM_IRQ_DMA1_CH1, uart_dma_isr);
    // 1 Mbaud, 10 bits per byte
    sim_timer_start(&uart_done, now_us + (uint64_t)size * 10ULL * 1000000ULL / huart->Init.BaudRate);
    return HAL_OK;
}

HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef* huart)
{
    return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

const char* sim_uart_text(void)
{
    if (!uart_log) uart_log = calloc(1, UART_LOG_SIZE);
    return uart_log;
}

int sim_uart_count(const char* needle)
{
    int count = 0;
    const char* p = sim_uart_text();
    while ((p = strstr(p, needle)) != NULL)
    {
        count++;
        p += strlen(needle);
    }
    return count;
}

void sim_uart_set_file(FILE* f)
{
    uart_file = f;
}
