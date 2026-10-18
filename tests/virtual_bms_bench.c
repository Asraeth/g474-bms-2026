/* Host-only virtual bench. Execute real transport, PEC, counter, application,
 * CAN adapters and timer/recovery code. Only the MCU/physical environment is
 * simulated; there is no healthy-Celsius override or balancing-cap override. */
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"

static TIM_TypeDef virtual_tim3_registers;
#undef TIM3
#define TIM3 (&virtual_tim3_registers)

static uint32_t virtual_tick, virtual_mask;
static bool virtual_isr;
/* HAL/CMSIS headers are already parsed. Substitute only CPU intrinsics that
 * cannot execute on x86; never change the included production source. */
#define __get_PRIMASK() (virtual_mask)
#define __set_PRIMASK(value) ((void)(virtual_mask = (value)))
#define __disable_irq() ((void)(virtual_mask = 1U))
#define __get_IPSR() (virtual_isr ? 1U : 0U)
#include "../Core/custom_lib/src/bms_utility.c"
#ifdef BMS_BASELINE_WRAPPER_PATH
#include BMS_BASELINE_WRAPPER_PATH
#else
#include "../Core/custom_lib/src/bms_libWrapper.c"
#endif
#include "../Core/Src/bms_can.c"
int unused_firmware_entry_point(void);
#define main unused_firmware_entry_point
#include "../Core/Src/main.c"
#undef main

SPI_HandleTypeDef hspi1;
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim15, htim3;

/* MinGW retains exported functions from the included main.c. Satisfy their
 * link references, but fail if this bench accidentally executes MCU startup. */
#define UNEXECUTED_INIT(name) void name(void) { assert(!"MCU initialization is outside the host bench"); }
UNEXECUTED_INIT(MX_GPIO_Init)
UNEXECUTED_INIT(MX_DMA_Init)
UNEXECUTED_INIT(MX_LPUART1_UART_Init)
UNEXECUTED_INIT(MX_SPI1_Init)
UNEXECUTED_INIT(MX_TIM2_Init)
UNEXECUTED_INIT(MX_TIM16_Init)
UNEXECUTED_INIT(MX_FDCAN1_Init)
UNEXECUTED_INIT(MX_FDCAN2_Init)
UNEXECUTED_INIT(MX_TIM15_Init)
UNEXECUTED_INIT(MX_TIM3_Init)
#undef UNEXECUTED_INIT
HAL_StatusTypeDef HAL_Init(void) { assert(!"HAL startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t scale)
{ (void)scale; assert(!"MCU clock startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *config)
{ (void)config; assert(!"MCU clock startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *config, uint32_t latency)
{ (void)config; (void)latency; assert(!"MCU clock startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{ (void)timer; assert(!"MCU timer startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; assert(!"MCU timer startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; assert(!"MCU timer startup is outside the host bench"); return HAL_ERROR; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; assert(!"MCU PWM startup is outside the host bench"); return HAL_ERROR; }
static uint32_t virtual_capture_period = 1280000U, virtual_capture_high = 640000U;
uint32_t HAL_TIM_ReadCapturedValue(const TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer == &htim15 && virtual_isr);
    assert(channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2);
    return channel == TIM_CHANNEL_1 ? virtual_capture_period : virtual_capture_high;
}

enum { BOARD_COUNT = 14, CELL_COUNT = 8, CHANNEL_COUNT = 16, FRAME_COUNT = 130 };
enum Injection { INJECT_NONE, INJECT_PEC, INJECT_DUPLICATE, INJECT_SKIP };
typedef struct {
    int16_t cells[BOARD_COUNT][CHANNEL_COUNT];
    int16_t aux[BOARD_COUNT][12];
    uint8_t counters[BOARD_COUNT], last_emitted[BOARD_COUNT];
    uint8_t config_a[BOARD_COUNT][6], config_b[BOARD_COUNT][6];
    uint8_t pwm_a[BOARD_COUNT][6], pwm_b[BOARD_COUNT][6];
    bool present[BOARD_COUNT], cs, stopped, busy, rx_timeout;
    bool command_seen, write_complete;
    uint16_t command, inject_command;
    enum Injection injection;
    unsigned int target_board, injected, polls, wraps, commands, writes, reads;
    unsigned int max_spi_timeout, poll_timeouts;
} VirtualChain;

static VirtualChain model;
static uint64_t elapsed_ms;
static bool automatic_current = true, automatic_voltage = true;
static uint8_t current_counter, voltage_counter;
static int32_t current_ma = 1234, voltage_mv = 420000;
static bool shutdown_asserted = true; // Mirrors MX_GPIO_Init's initial SET request.
static unsigned int shutdown_releases, checks, failures, checkpoint_count;
static unsigned int emitted_frames, captured_count, periodic_batches;
static unsigned int scenario_number;
static bool rx_pending;
static FDCAN_RxHeaderTypeDef pending_header;
static uint8_t pending_data[8];
static FILE *evidence, *firmware_log;

static struct {
    uint64_t first;
    uint8_t packets[BOARD_COUNT][8];
    uint64_t last;
} guarded_spi = {UINT64_C(0x42D4B67E12345678), {{0}}, UINT64_C(0x42D4B67E12345678)};
static struct {
    uint64_t first;
    CanTxMsg frames[FRAME_COUNT];
    uint64_t last;
} guarded_can = {UINT64_C(0xA45E638212345678), {{{0}, {0}}}, UINT64_C(0xA45E638212345678)};

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { failures++; \
        fprintf(stderr, "CHECK FAILED scenario=%u line=%d: %s\n", scenario_number, __LINE__, #condition); } \
} while (0)

static void advance_clock(uint32_t milliseconds);
static void feed_result(bool current, uint8_t state, int32_t value);
static void feed_pair(void);

uint32_t HAL_GetTick(void) { return virtual_tick; }
void HAL_Delay(uint32_t milliseconds) { advance_clock(milliseconds); }
uint32_t bms_enterCritical(void)
{
    uint32_t saved = virtual_mask; virtual_mask = 1; return saved;
}
void bms_exitCritical(uint32_t saved) { virtual_mask = saved; }
bool bms_spiContextAllowed(void) { return !virtual_isr && virtual_mask == 0; }
int printfDma(const char *format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    int result = vfprintf(firmware_log, format, arguments);
    va_end(arguments);
    return result;
}

static uint16_t reference_crc15(const uint8_t *bytes, unsigned int length)
{
    unsigned int crc = 16;
    for (unsigned int i = 0; i < length; i++) {
        crc ^= (unsigned int)bytes[i] << 7;
        for (int bit = 0; bit < 8; bit++)
            crc = ((crc << 1) ^ ((crc & 0x4000U) ? 0x4599U : 0U)) & 0x7FFFU;
    }
    return (uint16_t)(crc << 1);
}

static uint16_t reference_crc10(const uint8_t *bytes, uint8_t counter, bool include_counter)
{
    unsigned int crc = 16;
    for (unsigned int i = 0; i < 6; i++) {
        crc ^= (unsigned int)bytes[i] << 2;
        for (int bit = 0; bit < 8; bit++)
            crc = ((crc << 1) ^ ((crc & 0x200U) ? 0x8FU : 0U)) & 0x3FFU;
    }
    if (include_counter) crc ^= (unsigned int)counter << 4;
    for (int bit = 0; bit < 6; bit++)
        crc = ((crc << 1) ^ ((crc & 0x200U) ? 0x8FU : 0U)) & 0x3FFU;
    return (uint16_t)crc;
}

static void advance_device(unsigned int board)
{
    assert(board < BOARD_COUNT);
    if (model.counters[board] == 63) { model.counters[board] = 1; model.wraps++; }
    else model.counters[board]++;
}

static bool incrementing_command(uint16_t command)
{
    /* Independent model for commands the actual foreground uses. Do not call
     * production BMS_CC_Classify: a driver classification error must be visible. */
    return command == 0x0028 || command == 0x0029
        || command == 0x002D || command == 0x002F
        || command == 0x0718 || (command >= 0x071C && command <= 0x071F)
        || (command & 0xFE68U) == 0x0260U
        || (command & 0xFF6CU) == 0x0168U
        || (command & 0xFE30U) == 0x0410U;
}

void bms_csLow(void)
{
    assert(!model.cs && bms_spiContextAllowed());
    model.cs = true; model.command_seen = model.write_complete = false;
}
void bms_csHigh(void)
{
    assert(model.cs);
    if (model.write_complete)
        for (unsigned int board = 0; board < BOARD_COUNT; board++)
            if (model.present[board]) advance_device(board);
    model.cs = false;
}

BMS_StatusTypeDef bms_wakeupChain(void)
{
    if (!bms_spiContextAllowed()) return BMS_ERR_COMMS;
    // Time abstraction only: actual MCU wake waveform is not under test.
    advance_clock(2U * BOARD_COUNT * BMS_CHAIN_WAKE_PULSE_MS);
    return BMS_OK;
}
BMS_StatusTypeDef bms_delayMsActive(uint32_t milliseconds)
{
    advance_clock(milliseconds); return BMS_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, const uint8_t *data,
        uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && model.cs && !virtual_isr);
    assert(timeout > 0 && timeout <= BMS_ADBMS_SPI_TIMEOUT_MS);
    if (timeout > model.max_spi_timeout) model.max_spi_timeout = timeout;
    if (model.stopped) { advance_clock(timeout); return HAL_TIMEOUT; }
    if (size == 4) {
        assert(!model.command_seen);
        uint16_t crc = reference_crc15(data, 2);
        assert(data[2] == (uint8_t)(crc >> 8) && data[3] == (uint8_t)crc);
        model.command = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        model.command_seen = true; model.commands++; model.polls = 0;
        for (unsigned int board = 0; board < BOARD_COUNT; board++) {
            if (!model.present[board]) continue;
            if (model.command == 0x002E || model.command == 0x0027) {
                model.counters[board] = 0;
                if (model.command == 0x0027) {
                    memset(model.pwm_a[board], 0, 6); memset(model.pwm_b[board], 0, 6);
                    memset(model.config_a[board], 0, 6); memset(model.config_b[board], 0, 6);
                }
            } else if (incrementing_command(model.command)) advance_device(board);
        }
        return HAL_OK;
    }
    assert(size == BOARD_COUNT * 8 && model.command_seen);
    uint8_t (*destination)[6] = NULL;
    if (model.command == 0x0001) destination = model.config_a;
    if (model.command == 0x0024) destination = model.config_b;
    if (model.command == 0x0020) destination = model.pwm_a;
    if (model.command == 0x0021) destination = model.pwm_b;
    assert(destination != NULL);
    for (unsigned int slot = 0; slot < BOARD_COUNT; slot++) {
        unsigned int board = BOARD_COUNT - 1U - slot;
        const uint8_t *packet = data + slot * 8;
        uint16_t crc = reference_crc10(packet, 0, false);
        assert(packet[6] == (uint8_t)(crc >> 8) && packet[7] == (uint8_t)crc);
        if (model.present[board]) memcpy(destination[board], packet, 6);
    }
    model.writes++; model.write_complete = true;
    return HAL_OK;
}

static void put_word(uint8_t *data, unsigned int offset, int16_t value)
{
    assert(offset <= 4);
    uint16_t bits = (uint16_t)value;
    data[offset] = (uint8_t)bits; data[offset + 1] = (uint8_t)(bits >> 8);
}

static void register_payload(unsigned int board, uint8_t data[6])
{
    assert(board < BOARD_COUNT);
    const uint16_t cell_commands[] = {0x0003, 0x0005, 0x0007};
    const uint16_t aux_commands[] = {0x0019, 0x001A, 0x001B, 0x001F};
    for (unsigned int group = 0; group < 3; group++)
        if (model.command == cell_commands[group]) {
            for (unsigned int word = 0; word < 3; word++)
                put_word(data, word * 2, model.cells[board][group * 3 + word]);
            return;
        }
    for (unsigned int group = 0; group < 4; group++)
        if (model.command == aux_commands[group]) {
            for (unsigned int word = 0; word < 3; word++)
                put_word(data, word * 2, model.aux[board][group * 3 + word]);
            return;
        }
    memset(data, 0, 6);
    switch (model.command) {
    case 0x0030: put_word(data, 2, 4900); break; // ~25 C die; no external conversion.
    case 0x002C: data[0] = 0x68; data[1] = (uint8_t)board; break;
    case 0x0002: memcpy(data, model.config_a[board], 6); break;
    case 0x0026: memcpy(data, model.config_b[board], 6); break;
    case 0x0022: memcpy(data, model.pwm_a[board], 6); break;
    case 0x0023: memcpy(data, model.pwm_b[board], 6); break;
    default: assert(!"unmodeled read opcode");
    }
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi, uint8_t *data,
        uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && model.cs && model.command_seen && !virtual_isr);
    assert(timeout > 0 && timeout <= BMS_ADBMS_SPI_TIMEOUT_MS);
    if (model.stopped || model.rx_timeout) { advance_clock(timeout); return HAL_TIMEOUT; }
    if (size == 1) {
        model.polls++;
        if (model.busy) { *data = 0; advance_clock(1); model.poll_timeouts++; }
        else *data = model.polls == 1 ? 0x0F : 0xFF; // Partial busy is not ready.
        return HAL_OK;
    }
    assert(size == sizeof guarded_spi.packets);
    model.reads++;
    for (unsigned int board = 0; board < BOARD_COUNT; board++) {
        uint8_t *packet = guarded_spi.packets[board];
        if (!model.present[board]) { memset(packet, 0xFF, 8); continue; }
        register_payload(board, packet);
        uint8_t counter = model.counters[board];
        bool inject = model.injection != INJECT_NONE && board == model.target_board
                && model.command == model.inject_command;
        if (inject && model.injection == INJECT_DUPLICATE) counter = model.last_emitted[board];
        if (inject && model.injection == INJECT_SKIP) { advance_device(board); counter = model.counters[board]; }
        uint16_t crc = reference_crc10(packet, counter, true);
        packet[6] = (uint8_t)(((unsigned int)counter << 2) | (crc >> 8));
        packet[7] = (uint8_t)crc;
        if (inject) {
            if (model.injection == INJECT_PEC) packet[7] ^= 1;
            model.injection = INJECT_NONE; model.injected++;
        }
        model.last_emitted[board] = counter;
    }
    memcpy(data, guarded_spi.packets, size);
    assert(guarded_spi.first == UINT64_C(0x42D4B67E12345678));
    assert(guarded_spi.last == UINT64_C(0x42D4B67E12345678));
    return HAL_OK;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin; return GPIO_PIN_SET; // Logical SDC input; no hardware claim.
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == FAULT_CTRL_GPIO_Port && pin == FAULT_CTRL_Pin);
    shutdown_asserted = state == GPIO_PIN_SET;
    if (!shutdown_asserted) shutdown_releases++;
}

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 128000000U; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *can, const FDCAN_FilterTypeDef *filter)
{
    assert(can == &hfdcan2 && filter->IdType == FDCAN_STANDARD_ID);
    assert(filter->FilterIndex < 2 && filter->FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *can, uint32_t standard,
        uint32_t extended, uint32_t remote_standard, uint32_t remote_extended)
{
    assert(can == &hfdcan2 && standard == FDCAN_REJECT && extended == FDCAN_REJECT);
    assert(remote_standard == FDCAN_REJECT_REMOTE && remote_extended == FDCAN_REJECT_REMOTE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *can)
{ assert(can == &hfdcan1 || can == &hfdcan2); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *can, uint32_t events, uint32_t index)
{ (void)can; (void)events; (void)index; return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *can, uint32_t fifo)
{ assert(can == &hfdcan2 && fifo == FDCAN_RX_FIFO0); return rx_pending ? 1U : 0U; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(can == &hfdcan2 && fifo == FDCAN_RX_FIFO0 && rx_pending);
    *header = pending_header; memcpy(data, pending_data, 8); rx_pending = false;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *can)
{ assert(can == &hfdcan1); return 1; } // Immediate successful transmission model.
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *can, uint32_t index)
{ (void)can; (void)index; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
        const FDCAN_TxHeaderTypeDef *header, const uint8_t *data)
{
    assert(can == &hfdcan1 && captured_count < FRAME_COUNT);
    assert(header->IdType == FDCAN_EXTENDED_ID && header->DataLength == 8);
    guarded_can.frames[captured_count].header = *header;
    memcpy(guarded_can.frames[captured_count].data, data, 8);
    captured_count++; emitted_frames++;
    return HAL_OK;
}

static void receive_can(uint32_t identifier, uint8_t dlc, const uint8_t data[8])
{
    assert(!rx_pending && virtual_mask == 0);
    pending_header = (FDCAN_RxHeaderTypeDef){.Identifier = identifier,
        .DataLength = dlc, .IdType = FDCAN_STANDARD_ID,
        .RxFrameType = FDCAN_DATA_FRAME, .FDFormat = FDCAN_CLASSIC_CAN};
    memcpy(pending_data, data, 8); rx_pending = true;
    bool saved = virtual_isr; virtual_isr = true;
    HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    virtual_isr = saved;
    assert(!rx_pending);
}

static void feed_result(bool current, uint8_t state, int32_t value)
{
    uint32_t bits = (uint32_t)value;
    uint8_t counter = current ? current_counter++ : voltage_counter++;
    uint8_t data[8] = {(uint8_t)(current ? 0 : 1), (uint8_t)((state << 4) | (counter & 15U)),
        (uint8_t)(bits >> 24), (uint8_t)(bits >> 16), (uint8_t)(bits >> 8), (uint8_t)bits, 0, 0};
    receive_can(current ? 0x521U : 0x522U, 6, data);
}
static void feed_pair(void)
{
    if (automatic_current) feed_result(true, 0, current_ma);
    if (automatic_voltage) feed_result(false, 0, voltage_mv);
}

static void timer_service(void)
{
    bool saved = virtual_isr; virtual_isr = true;
    captured_count = 0;
    HAL_TIM_PeriodElapsedCallback(&htim16); // Actual shutdown/telemetry callback.
    if (captured_count) {
        assert(captured_count == FRAME_COUNT); periodic_batches++;
        HAL_FDCAN_TxFifoEmptyCallback(&hfdcan1);
    }
    virtual_isr = saved;
}
static void advance_clock(uint32_t milliseconds)
{
    for (uint32_t i = 0; i < milliseconds; i++) {
        virtual_tick++; elapsed_ms++;
        if (elapsed_ms % 20U == 0) feed_pair();
        if (elapsed_ms % 100U == 0) {
            bool saved = virtual_isr; virtual_isr = true;
            HAL_TIM_IC_CaptureCallback(&htim15);
            virtual_isr = saved;
            timer_service();
        }
    }
}

static BMS_StatusTypeDef acquisition(void)
{
    BMS_StatusTypeDef status = BMS_ProgramLoop();
    /* Evaluate the existing public safety function at the observation point,
     * including when an acquisition returned early. This is an explicit bench
     * observation schedule, not a claim of MCU interrupt timing equivalence. */
    BMS_UpdateStatusFlags();
    return status;
}
static void foreground_cycle(void)
{
    BMS_ProcessImdCapture();
    BMS_StatusTypeDef status = acquisition();
    if (status != BMS_OK) BMS_FaultHandler(status); // Actual bounded recovery code.
}

static int16_t decode16(const uint8_t *bytes)
{ int16_t result; memcpy(&result, bytes, sizeof result); return result; }
static int32_t decode32(const uint8_t *bytes)
{ int32_t result; memcpy(&result, bytes, sizeof result); return result; }
static CanTxMsg *frame_at(uint32_t identifier)
{
    for (unsigned int i = 0; i < captured_count; i++)
        if (guarded_can.frames[i].header.Identifier == identifier) return &guarded_can.frames[i];
    assert(!"missing telemetry frame"); return NULL;
}

static void collect_telemetry(void)
{
    CanTxMsg *frames; uint32_t count;
    BMS_GetCanData(&frames, &count); CHECK(count == FRAME_COUNT);
    captured_count = 0;
    BMS_CAN_SendBuffer(frames, count);
    HAL_FDCAN_TxFifoEmptyCallback(&hfdcan1);
    CHECK(captured_count == FRAME_COUNT);
    CHECK(guarded_can.first == UINT64_C(0xA45E638212345678));
    CHECK(guarded_can.last == UINT64_C(0xA45E638212345678));
    for (unsigned int i = 0; i < captured_count; i++) {
        for (unsigned int j = i + 1; j < captured_count; j++)
            CHECK(guarded_can.frames[i].header.Identifier != guarded_can.frames[j].header.Identifier);
        uint32_t id = guarded_can.frames[i].header.Identifier;
        CHECK((id >= 0xB000 && id <= 0xB080) || id == CHARGER_CONFIG_CAN_ID);
    }
}

static void checkpoint(const char *label, unsigned int expected_faults, bool adbms_valid,
        bool current_valid, bool voltage_valid, int expected_pec, int expected_counter)
{
    unsigned int initial_failures = failures;
    BMS_UpdateStatusFlags();
    BMS_EnableCharging(true); // A real API request, never force a healthy fixture.
    timer_service();
    BMS_AdbmsDataStatus status; BMS_GetAdbmsDataStatus(&status);
    IVTS_Snapshot ivt; BMS_CAN_GetIVTSnapshot(&ivt);
    BMS_PackMeasurements pack; BMS_GetPackMeasurements(&pack);
    unsigned int cell_count = 0, temp_count = 0, celsius_count = 0, vpv_count = 0, die_count = 0;
    for (unsigned int board = 0; board < BOARD_COUNT; board++) {
        for (unsigned int channel = 0; channel < CELL_COUNT; channel++) {
            cell_count += status.cell_valid && isfinite(ic_ad68.v_cell[VOLTAGE_S][board][channel]);
            temp_count += status.aux_valid && ic_ad68.temp_input_valid[board][channel];
            celsius_count += status.aux_valid && ic_ad68.temp_valid[board][channel];
            CHECK(ic_ad68.isDischarging[board] == 0);
            if (adbms_valid) {
                float expected_cell = (float)((double)model.cells[board][channel] * 0.00015 + 1.5);
                CHECK(fabsf(ic_ad68.v_cell[VOLTAGE_S][board][channel] - expected_cell) < 0.00001f);
            }
        }
        vpv_count += status.aux_valid && ic_ad68.v_segment_valid[board];
        die_count += status.aux_valid && ic_ad68.temp_ic_valid[board];
        for (unsigned int byte = 0; byte < 6; byte++) {
            CHECK(model.pwm_a[board][byte] == 0 && model.pwm_b[board][byte] == 0);
        }
        CHECK(model.config_b[board][3] == 0 && model.config_b[board][4] == 0 && model.config_b[board][5] == 0);
    }
    CHECK((unsigned int)BMS_StatusFlags == expected_faults);
    CHECK(status.cell_valid == adbms_valid && status.aux_valid == adbms_valid);
    CHECK(cell_count == (adbms_valid ? 112U : 0U));
    CHECK(temp_count == (adbms_valid ? 112U : 0U) && celsius_count == 0);
    CHECK(vpv_count == (adbms_valid ? 14U : 0U) && die_count == (adbms_valid ? 14U : 0U));
    CHECK(ivt.current.valid == current_valid && ivt.u1.valid == voltage_valid);
    CHECK(!BMS_IsCharging() && shutdown_asserted && !enableBalancing && shutdown_releases == 0);
    if (expected_pec >= 0) CHECK(status.last_pec_error == (expected_pec != 0));
    if (expected_counter >= 0) CHECK(status.last_counter_error == (expected_counter != 0));
    collect_telemetry();
    // Verify every board-major cell frame, not just the last representative
    // slot used in the compact report. Tolerance is at most one encoding LSB,
    // solely for float/truncation arithmetic, not an analog hardware tolerance.
    for (unsigned int board = 0; board < BOARD_COUNT; board++) {
        for (unsigned int channel = 0; channel < CELL_COUNT; channel++) {
            CanTxMsg *slot = frame_at(0xB000U + board * 8U + channel);
            CHECK((bool)(slot->data[7] & BMS_CAN_CELL_VOLTAGE_VALID) == adbms_valid);
            CHECK(!(slot->data[7] & BMS_CAN_TEMP_C_VALID));
            CHECK(decode16(slot->data + 4) == INT16_MIN);
            if (adbms_valid) {
                double expected_mv = ((double)model.cells[board][channel] * 0.15) + 1500.0;
                CHECK(fabs((double)decode16(slot->data) - expected_mv) <= 1.0);
            } else CHECK(decode16(slot->data) == INT16_MIN);
        }
    }
    CanTxMsg *cell = frame_at(0xB06F), *board_frame = frame_at(0xB07D);
    CanTxMsg *current = frame_at(0xB07E), *voltage = frame_at(0xB07F), *cross = frame_at(0xB080);
    CHECK((bool)(cell->data[7] & BMS_CAN_CELL_VOLTAGE_VALID) == adbms_valid);
    CHECK((bool)(cell->data[7] & BMS_CAN_TEMP_INPUT_VALID) == adbms_valid);
    CHECK(!(cell->data[7] & BMS_CAN_TEMP_C_VALID) && decode16(cell->data + 4) == INT16_MIN);
    CHECK((bool)(board_frame->data[7] & BMS_CAN_CELL_VOLTAGE_VALID) == adbms_valid);
    CHECK((bool)(current->data[6] & BMS_PACK_VALID) == current_valid);
    CHECK((bool)(voltage->data[6] & BMS_PACK_VALID) == voltage_valid);
    CHECK(decode32(current->data) == (current_valid ? ivt.current.raw_value : INT32_MIN));
    CHECK(decode32(voltage->data) == (voltage_valid ? ivt.u1.raw_value : INT32_MIN));
    CHECK((decode32(cross->data) != INT32_MIN) == adbms_valid);
    CHECK((decode32(cross->data + 4) != INT32_MIN) == (adbms_valid && voltage_valid));
    CHECK(frame_at(CHARGER_CONFIG_CAN_ID)->data[4] == 1);
    if (!adbms_valid) CHECK(decode16(cell->data) == INT16_MIN);
    checkpoint_count++;
    fprintf(evidence, "{\"checkpoint\":\"%s\",\"tick\":%" PRIu32 ",\"elapsed_ms\":%" PRIu64 ",", label, virtual_tick, elapsed_ms);
    fprintf(evidence, "\"expected\":{\"faults\":%u,\"adbms_valid\":%s,\"current_valid\":%s,\"voltage_valid\":%s,\"valid_cells\":%u,\"valid_raw_temperatures\":%u,\"valid_celsius\":0,\"charging\":false,\"shutdown_asserted\":true,\"balancing\":false},",
        expected_faults, adbms_valid ? "true" : "false", current_valid ? "true" : "false", voltage_valid ? "true" : "false",
        adbms_valid ? 112U : 0U, adbms_valid ? 112U : 0U);
    fprintf(evidence, "\"actual\":{\"faults\":%u,\"cell_valid\":%s,\"aux_valid\":%s,\"valid_cells\":%u,\"valid_raw_temperatures\":%u,\"valid_celsius\":%u,\"valid_vpv\":%u,\"valid_die\":%u,",
        (unsigned int)BMS_StatusFlags, status.cell_valid ? "true" : "false", status.aux_valid ? "true" : "false",
        cell_count, temp_count, celsius_count, vpv_count, die_count);
    fprintf(evidence, "\"current_valid\":%s,\"voltage_valid\":%s,\"current_stale\":%s,\"voltage_stale\":%s,\"current_raw_status\":%u,\"voltage_raw_status\":%u,\"current_last_ms\":%" PRIu32 ",\"voltage_last_ms\":%" PRIu32 ",",
        ivt.current.valid ? "true" : "false", ivt.u1.valid ? "true" : "false", ivt.current.stale ? "true" : "false", ivt.u1.stale ? "true" : "false",
        ivt.current.raw_status, ivt.u1.raw_status, ivt.current.last_measurement_ms, ivt.u1.last_measurement_ms);
    fprintf(evidence, "\"charging\":%s,\"shutdown_asserted\":%s,\"balancing\":%s,\"pec_error\":%s,\"counter_error\":%s,\"transport_fault\":%u,\"live_transport_fault\":%u,\"stop_failed\":%s,\"counter_expected\":%u,\"counter_received\":%u,\"cell_last_ms\":%" PRIu32 ",\"aux_last_ms\":%" PRIu32 ",\"alive_count\":%" PRIu32 ",\"sum_v\":",
        BMS_IsCharging() ? "true" : "false", shutdown_asserted ? "true" : "false", enableBalancing ? "true" : "false",
        status.last_pec_error ? "true" : "false", status.last_counter_error ? "true" : "false",
        (unsigned int)status.last_acquisition_transport_fault, (unsigned int)bms_getTransportFault(), status.discharge_stop_failed ? "true" : "false",
        status.counters.device[13].expected, status.counters.device[13].received,
        status.cell_last_received_ms, status.aux_last_received_ms, ivt.alive.count);
    if (pack.cell_voltage_sum_valid) fprintf(evidence, "%.6f", (double)pack.cell_voltage_sum_v); else fputs("null", evidence);
    fputs("},\"telemetry\":[", evidence);
    for (unsigned int i = 0; i < captured_count; i++) {
        if (i) fputc(',', evidence);
        fprintf(evidence, "{\"id\":%" PRIu32 ",\"data\":[", guarded_can.frames[i].header.Identifier);
        for (unsigned int byte = 0; byte < 8; byte++) fprintf(evidence, "%s%u", byte ? "," : "", guarded_can.frames[i].data[byte]);
        fputs("]}", evidence);
    }
    fprintf(evidence, "],\"pass\":%s}\n", failures == initial_failures ? "true" : "false");
}

static void startup(void)
{
    CHECK(TOTAL_IC == BOARD_COUNT && TOTAL_CELL == CELL_COUNT && TOTAL_PACK_CELLS == 112);
    CHECK(BMS_BALANCING_MAX_PWM == 0 && BMS_BALANCING_PWM_BUDGET == 0 && !BMS_BALANCING_CONFIG_CONFIRMED);
    for (unsigned int board = 0; board < BOARD_COUNT; board++) {
        model.present[board] = true;
        for (unsigned int channel = 0; channel < CHANNEL_COUNT; channel++)
            model.cells[board][channel] = channel < CELL_COUNT ? 15000 : INT16_MIN;
        for (unsigned int channel = 0; channel < 12; channel++) model.aux[board][channel] = (int16_t)(1000U + channel * 100U);
        model.aux[board][11] = -2000; // VPV = 30.0 V.
    }
    BMS_CAN_Config(); feed_pair(); BMS_WriteFaultSignal(true);
    CHECK(bms_softReset() == BMS_OK);
    HAL_Delay(BMS_SENSOR_STARTUP_DELAY_MS);
    CHECK(bms_softReset() == BMS_OK);
    CHECK(bms_init() == BMS_OK);
    initRequired = false;
    if (scenario_number == 1) checkpoint("initialized-before-first-measurement", 7, false, true, true, 0, 0);
    foreground_cycle();
    checkpoint("startup-acquired", BMS_ERR_TEMP, true, true, true, 0, 0);
}

static void inject(enum Injection kind, uint16_t command)
{
    model.injection = kind; model.target_board = 13; model.inject_command = command;
}

static void run_scenario(void)
{
    startup();
    switch (scenario_number) {
    case 1:
    case 2:
        for (int loop = 0; loop < 3; loop++) { advance_clock(500); foreground_cycle(); }
        CHECK(fabsf(ic_common.v_pack_total - 420.0f) < 0.001f && model.reads > 0);
        checkpoint("nominal-multiple-cycles", 4, true, true, true, 0, 0);
        break;
    case 3:
    case 4:
        model.cells[13][7] = scenario_number == 3 ? 11000 : 19000; // 3.15 V / 4.35 V.
        foreground_cycle();
        CHECK(ic_ad68.isCellFaultDetected[13] & (1U << 7));
        checkpoint("one-cell-outside-limits", 6, true, true, true, 0, 0);
        break;
    case 5:
        model.present[13] = false;
        CHECK(acquisition() == BMS_ERR_COMMS);
        checkpoint("board-14-absent", 7, false, true, true, 1, -1);
        BMS_FaultHandler(BMS_ERR_COMMS);
        CHECK(programStats.counter_commsError == BMS_COMMS_RETRY_ATTEMPTS);
        checkpoint("missing-board-bounded-recovery", 7, false, true, true, 1, -1);
        break;
    case 6:
    case 7:
    case 8:
        inject(scenario_number == 6 ? INJECT_PEC : scenario_number == 7 ? INJECT_DUPLICATE : INJECT_SKIP,
            scenario_number == 6 ? 0x0005 : 0x0003);
        CHECK(acquisition() == BMS_ERR_COMMS && model.injected == 1);
        checkpoint("one-acquisition-injected-fault", 7, false, true, true,
            scenario_number == 6 ? 1 : 0, scenario_number == 6 ? 0 : 1);
        if (scenario_number != 6) {
            BMS_AdbmsDataStatus status; BMS_GetAdbmsDataStatus(&status);
            CHECK(status.counters.device[13].fault && !status.counters.device[0].fault);
            if (scenario_number == 8) CHECK(status.counters.device[13].received == status.counters.device[13].expected + 1U);
            CHECK(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS); // Latching.
        }
        BMS_FaultHandler(BMS_ERR_COMMS);
        CHECK(initRequired);
        CHECK(bms_init() == BMS_OK); foreground_cycle();
        checkpoint("recovered-after-one-acquisition-fault", 4, true, true, true, 0, 0);
        break;
    case 9:
        for (int loop = 0; loop < 25; loop++) { advance_clock(500); foreground_cycle(); }
        CHECK(model.wraps >= 2U * BOARD_COUNT);
        checkpoint("two-or-more-counter-wraps", 4, true, true, true, 0, 0);
        break;
    case 10: {
        uint32_t timestamp = ic_common.cell_last_received_ms[VOLTAGE_S];
        advance_clock(BMS_ADBMS_CELL_MAX_AGE_MS);
        CHECK(ic_common.cell_last_received_ms[VOLTAGE_S] == timestamp);
        checkpoint("foreground-acquisition-stopped-stale", 7, false, true, true, 0, 0);
        model.stopped = true;
        CHECK(acquisition() == BMS_ERR_COMMS);
        CHECK(bms_getTransportFault() == BMS_TRANSPORT_TX);
        checkpoint("queued-stop-link-timeout", 7, false, true, true, 0, 0);
        // Also fail an acquisition directly: its primary diagnosis differs
        // from the stop-only failure above and must survive cleanup transfers.
        CHECK(bms_getAuxMeasurement() == BMS_ERR_COMMS);
        BMS_AdbmsDataStatus status; BMS_GetAdbmsDataStatus(&status);
        CHECK(status.last_acquisition_transport_fault == BMS_TRANSPORT_TX);
        checkpoint("acquisition-command-timeout", 7, false, true, true, 0, 0);
        BMS_FaultHandler(BMS_ERR_COMMS);
        CHECK(programStats.counter_commsError == BMS_COMMS_RETRY_ATTEMPTS);
        model.stopped = false; CHECK(bms_init() == BMS_OK); foreground_cycle();
        model.busy = true;
        CHECK(acquisition() == BMS_ERR_COMMS);
        BMS_GetAdbmsDataStatus(&status);
        CHECK(status.last_acquisition_transport_fault == BMS_TRANSPORT_POLL_TIMEOUT);
        CHECK(model.poll_timeouts >= BMS_ADBMS_POLL_TIMEOUT_MS);
        checkpoint("bounded-adc-busy-timeout", 7, false, true, true, 0, 0);
        model.busy = false;
        BMS_FaultHandler(BMS_ERR_COMMS);
        CHECK(bms_init() == BMS_OK); foreground_cycle();
        model.rx_timeout = true;
        CHECK(bms_getAuxMeasurement() == BMS_ERR_COMMS);
        BMS_GetAdbmsDataStatus(&status);
        CHECK(status.last_acquisition_transport_fault == BMS_TRANSPORT_RX);
        checkpoint("acquisition-receive-timeout", 7, false, true, true, 0, 0);
        break;
    }
    case 11:
    case 12:
        if (scenario_number == 11) automatic_current = false; else automatic_voltage = false;
        advance_clock(scenario_number == 11 ? BMS_IVT_CURRENT_TIMEOUT_MS + 1U : BMS_IVT_U1_TIMEOUT_MS + 1U);
        foreground_cycle();
        checkpoint("one-ivt-stream-stopped", scenario_number == 11 ? 28U : 22U, true,
            scenario_number != 11, scenario_number != 12, 0, 0);
        break;
    case 13:
        automatic_current = automatic_voltage = false;
        feed_result(true, IVTS_STATE_SYSTEM_ERROR, current_ma);
        checkpoint("ivt-system-error", 30, true, false, false, 0, 0);
        automatic_current = automatic_voltage = true; advance_clock(20); foreground_cycle();
        checkpoint("ivt-clean-results-recover", 4, true, true, true, 0, 0);
        break;
    case 14: {
        uint8_t alive[8] = {0xBF, 0x04, 0x11, 0x12, 0x34, 0x56, 0x78, 0};
        receive_can(0x511, 8, alive);
        current_counter = voltage_counter = 0; // Simulated sensor restarted its emitters.
        CHECK(ivtSensor.state.alive.count == 1 && !ivtSensor.state.configuration_verified);
        CHECK(ivtSensor.state.alive.command_id == 0x411 && ivtSensor.state.alive.serial_number == 0x12345678U);
        checkpoint("ivt-alive-invalidates-cache", 30, true, false, false, 0, 0);
        automatic_voltage = false; feed_result(true, 0, current_ma);
        checkpoint("restart-current-only", 22, true, true, false, 0, 0);
        automatic_voltage = true; feed_pair(); foreground_cycle();
        checkpoint("restart-both-streams-restored", 4, true, true, true, 0, 0);
        break;
    }
    case 15:
        voltage_mv = 400000; feed_pair(); foreground_cycle();
        checkpoint("twenty-volt-disagreement-diagnostic", 4, true, true, true, 0, 0);
        CHECK(decode32(frame_at(0xB080)->data) == 420000);
        CHECK(decode32(frame_at(0xB080)->data + 4) == -20000);
        break;
    case 16: {
        const unsigned int gpio_indices[] = {8,6,7,9,3,5,4,2};
        for (unsigned int board = 0; board < BOARD_COUNT; board++)
            for (unsigned int channel = 0; channel < CELL_COUNT; channel++) {
                float expected = (float)((double)model.aux[board][gpio_indices[channel]] * 0.00015 + 1.5);
                CHECK(fabsf(ic_ad68.v_temp[board][channel] - expected) < 0.00001f);
                CHECK(isnan(ic_ad68.temp_cell[board][channel]) && !ic_ad68.temp_valid[board][channel]);
            }
        checkpoint("direct-inputs-valid-celsius-unavailable", 4, true, true, true, 0, 0);
        break;
    }
    case 17:
    case 18:
        BMS_EnableBalancing(true);
        for (unsigned int board = 0; board < BOARD_COUNT; board++)
            for (unsigned int channel = scenario_number == 17 ? 0U : 8U;
                    channel < (scenario_number == 17 ? 8U : 16U); channel++)
                bms_setPwm((uint8_t)board, (uint8_t)channel, 15);
        CHECK(bms_writeRegister(REG_PWM_A) == BMS_OK && bms_writeRegister(REG_PWM_B) == BMS_OK);
        CHECK(bms_startDischarge(3.7f) == BMS_OK);
        checkpoint("balancing-request-refused", 4, true, true, true, 0, 0);
        break;
    case 19:
        inject(INJECT_DUPLICATE, 0x0003);
        CHECK(acquisition() == BMS_ERR_COMMS);
        checkpoint("fault-before-recovery", 7, false, true, true, 0, 1);
        BMS_FaultHandler(BMS_ERR_COMMS);
        CHECK(initRequired && programStats.counter_commsError <= BMS_COMMS_RETRY_ATTEMPTS);
        checkpoint("reset-without-new-measurements", 7, false, true, true, 0, 0);
        CHECK(bms_init() == BMS_OK); initRequired = false;
        foreground_cycle();
        checkpoint("recovery-requires-new-acquisition", 4, true, true, true, 0, 0);
        break;
    case 20: {
        CHECK(virtual_tick < 2000U); // Startup crossed UINT32_MAX.
        uint32_t stamp = virtual_tick; advance_clock(700); foreground_cycle();
        CHECK((uint32_t)(virtual_tick - stamp) >= 700U);
        checkpoint("tick-wrap-fresh", 4, true, true, true, 0, 0);
        advance_clock(BMS_ADBMS_CELL_MAX_AGE_MS);
        checkpoint("tick-wrap-stale", 7, false, true, true, 0, 0);
        CHECK(bms_softReset() == BMS_OK && bms_init() == BMS_OK); foreground_cycle();
        checkpoint("tick-wrap-recovery", 4, true, true, true, 0, 0);
        break;
    }
    default: assert(!"unknown scenario");
    }
    CHECK(model.max_spi_timeout <= BMS_ADBMS_SPI_TIMEOUT_MS && !model.cs && virtual_mask == 0);
    fprintf(evidence, "{\"summary\":true,\"scenario\":%u,\"checks\":%u,\"failures\":%u,\"checkpoints\":%u,\"commands\":%u,\"register_reads\":%u,\"register_writes\":%u,\"counter_wraps\":%u,\"periodic_batches\":%u,\"emitted_frames\":%u,\"pass\":%s}\n",
        scenario_number, checks, failures, checkpoint_count, model.commands, model.reads, model.writes,
        model.wraps, periodic_batches, emitted_frames, failures ? "false" : "true");
}

#ifndef BMS_VIRTUAL_BENCH_NO_MAIN
int main(int argc, char **argv)
{
    if (argc != 4) { fprintf(stderr, "usage: virtual_bms_bench SCENARIO EVIDENCE.jsonl FIRMWARE.log\n"); return 2; }
    char *end; unsigned long number = strtoul(argv[1], &end, 10);
    if (*end || number < 1 || number > 20) return 2;
    scenario_number = (unsigned int)number;
    evidence = fopen(argv[2], "w"); firmware_log = fopen(argv[3], "w");
    if (!evidence || !firmware_log) return 2;
    virtual_tick = scenario_number == 20 ? UINT32_MAX - 400U : 1000U;
    run_scenario();
    fclose(evidence); fclose(firmware_log);
    printf("SCENARIO %u: %s (%u checks, %u checkpoints)\n", scenario_number,
        failures ? "FAIL" : "PASS", checks, checkpoint_count);
    return failures ? 1 : 0;
}
#endif
