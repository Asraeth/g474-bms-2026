/* Exercise the actual application wrapper with an IVT driver and hardware stubs. */
#include <assert.h>
#include <stdlib.h>
#include "../Core/custom_lib/src/bms_libWrapper.c"

static IVTS_Driver sensor;
static uint32_t now;
static bool badPec;
static bool failRead, failWrite, failPoll, resetCellCode, verifyTransaction, badStopReadback;
static unsigned int readCount, writeCount, commandCount, failPecRead;
static uint8_t capturedConfig[TOTAL_AD68][DATA_LEN], capturedA[TOTAL_AD68][DATA_LEN], capturedB[TOTAL_AD68][DATA_LEN];
static int16_t cellRaw = 15000;
static BMS_CommandCounterChain counterState;
static uint8_t hardwareCounter[TOTAL_IC];
static unsigned int counterFaultRead;
static int counterFaultDelta;
static bool dropCommand, missCounterReset;
static unsigned int counterChecks;

// Model physical acceptance separately from the driver's expected state.
static void accepted_command(const uint8_t command[CMD_LEN], bool failed)
{
    BMS_CounterCommand kind = BMS_CC_Classify(command);
    assert(kind != BMS_CC_UNSUPPORTED);
    if (failed) { BMS_CC_Invalidate(&counterState); return; }
    if (kind == BMS_CC_RESET) {
        BMS_CC_Reset(&counterState);
        if (!missCounterReset) memset(hardwareCounter, 0, sizeof hardwareCounter);
    } else if (kind == BMS_CC_INCREMENT || kind == BMS_CC_WRITE) {
        BMS_CC_Advance(&counterState);
        if (!dropCommand)
            for (unsigned int i = 0; i < TOTAL_IC; i++)
                hardwareCounter[i] = hardwareCounter[i] == 63 ? 1 : hardwareCounter[i] + 1;
    }
}
void bms_getCounterState(BMS_CommandCounterChain *state) { *state = counterState; }
bool bms_checkRxCounter(uint8_t cc[TOTAL_IC], const bool pecErrors[TOTAL_IC], bool errors[TOTAL_IC])
{
    counterChecks++;
    return BMS_CC_Check(&counterState, cc, pecErrors, errors);
}
BMS_TransportFault bms_getTransportFault(void) { return failPoll ? BMS_TRANSPORT_POLL_TIMEOUT : failRead ? BMS_TRANSPORT_RX : failWrite ? BMS_TRANSPORT_TX : BMS_TRANSPORT_OK; }
ChargerStatus chargerStatus;
uint32_t HAL_GetTick(void) { return now; }
uint32_t bms_enterCritical(void) { return 0; }
void bms_exitCritical(uint32_t mask) { (void)mask; }

void BMS_CAN_GetIVTSnapshot(IVTS_Snapshot *snapshot) { IVTS_GetSnapshot(&sensor, now, snapshot); }
int printfDma(const char *format, ...) { (void)format; return 0; }
void Error_Handler(void) { abort(); }
BMS_StatusTypeDef bms_wakeupChain(void) { return BMS_OK; }
BMS_StatusTypeDef bms_delayMsActive(uint32_t ms) { (void)ms; return BMS_OK; }
BMS_StatusTypeDef bms_transmitCmd(uint8_t command[CMD_LEN]) { commandCount++; accepted_command(command, failWrite); return failWrite ? BMS_ERR_COMMS : BMS_OK; }
BMS_StatusTypeDef bms_transmitPoll(uint8_t command[CMD_LEN]) { accepted_command(command, false); return failPoll ? BMS_ERR_COMMS : BMS_OK; }
BMS_StatusTypeDef bms_transmitData(uint8_t command[CMD_LEN], uint8_t data[TOTAL_IC][DATA_LEN])
{
    writeCount++;
    accepted_command(command, failWrite);
    if (command == WRCFGB) memcpy(capturedConfig, data, sizeof capturedConfig);
    if (command == WRPWMA) memcpy(capturedA, data, sizeof capturedA);
    if (command == WRPWMB) memcpy(capturedB, data, sizeof capturedB);
    return failWrite ? BMS_ERR_COMMS : BMS_OK;
}
bool bms_checkRxPec(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC],
        uint8_t cc[TOTAL_IC], bool errors[TOTAL_IC])
{
    (void)data; (void)pec; (void)cc;
    bool fault = badPec || (failPecRead && readCount == failPecRead);
    for (int i = 0; i < TOTAL_IC; i++) errors[i] = fault;
    return fault;
}
BMS_StatusTypeDef bms_receiveData(uint8_t command[CMD_LEN], uint8_t data[TOTAL_IC][DATA_LEN],
        uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    readCount++;
    if (failRead) return BMS_ERR_COMMS;
    if (verifyTransaction && command == RDSVB) {
        assert(bms_cellDataFresh(VOLTAGE_S));
        assert(fabsf(ic_ad68.v_cell[VOLTAGE_S][0][0] - 3.75f) < 0.0001f);
    }
    bool auxA = command == RDAUXA, auxB = command == RDAUXB, auxC = command == RDAUXC, auxD = command == RDAUXD;
    for (int board = 0; board < TOTAL_IC; board++) {
        for (int byte = 0; byte < 6; byte += 2) {
            data[board][byte] = cellRaw & 0xFF; data[board][byte + 1] = (uint16_t)cellRaw >> 8;
            if (auxA || auxB || auxC || auxD) { data[board][byte] = 0x60; data[board][byte+1] = 9; } // 1.86 V
            if (command == RDSTATA && byte == 2) { data[board][byte] = 0x24; data[board][byte+1] = 0x13; } // Die ~25 C
            if (auxD && byte == 4) { data[board][byte] = 0x30; data[board][byte+1] = 0xF8; } // VPV 30 V
            if (resetCellCode && command == RDSVC && board == 13 && byte == 2) { data[board][byte] = 0; data[board][byte+1] = 0x80; }
        }
        pec[board] = 0; cc[board] = hardwareCounter[board];
        if (readCount == counterFaultRead && board == 13)
            cc[board] = (uint8_t)(cc[board] + counterFaultDelta);
        if (command == RDCFGB) memcpy(data[board], capturedConfig[board], DATA_LEN);
        if (command == RDPWMA) memcpy(data[board], capturedA[board], DATA_LEN);
        if (command == RDPWMB) memcpy(data[board], capturedB[board], DATA_LEN);
        if (badStopReadback && command == RDPWMB && board == 13) data[board][0] = 1;
    }
    return BMS_OK;
}
void BMS_CAN_GetChargerMsg(const ChargerConfiguration *config, uint8_t *data)
{ (void)config; memset(data, 0, 8); }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return GPIO_PIN_SET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }

static void receive(bool current, uint8_t counter, int32_t value)
{
    uint32_t bits = (uint32_t)value;
    IVTS_Frame frame = {.id = current ? IVTS_CURRENT_ID : IVTS_U1_ID, .dlc = 6,
        .data = {current ? 0 : 1, counter, bits >> 24, bits >> 16, bits >> 8, bits}};
    IVTS_Receive(&sensor, &frame, now);
}

static void healthy_legacy_fixture(void)
{
    ic_common.aux_snapshot_valid = true;
    ic_common.aux_last_received_ms = now;
    /* These are synthetic values to isolate pack-source integration. No final
     * thermistor conversion or board threshold change is implied. */
    for (int board = 0; board < TOTAL_AD68; board++) {
        ic_ad68.v_segment[board] = 30; ic_ad68.v_segment_valid[board] = true;
        ic_ad68.temp_ic[board] = 25; ic_ad68.temp_ic_valid[board] = true;
        for (int c = 0; c < TOTAL_CELL; c++) {
            ic_ad68.temp_cell[board][c] = 25; ic_ad68.temp_valid[board][c] = true;
        }
    }
}

static CanTxMsg *find(CanTxMsg *messages, uint32_t count, uint32_t id)
{
    for (uint32_t i = 0; i < count; i++) if (messages[i].header.Identifier == id) return &messages[i];
    abort();
}

static int32_t value32(const uint8_t *data)
{
    int32_t result; memcpy(&result, data, 4); return result;
}

static void fresh_fixture(void)
{
    badPec = failRead = failWrite = failPoll = resetCellCode = verifyTransaction = badStopReadback = false;
    failPecRead = 0; cellRaw = 15000;
    counterFaultRead = 0; counterFaultDelta = 0; dropCommand = missCounterReset = false;
    readCount = writeCount = commandCount = 0;
    enableBalancing = false;
    IVTS_Config config = IVTS_DefaultConfig(); assert(IVTS_Init(&sensor, &config));
    now += 10;
    assert(bms_synchronizeCounters() == BMS_OK);
    receive(true, 0, 0); receive(false, 0, 400000);
    assert(bms_getAuxMeasurement() == BMS_OK);
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_OK);
    healthy_legacy_fixture();
    assert(BMS_UpdateStatusFlags() == BMS_OK);
}

static void assert_outputs_off(void)
{
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        const uint8_t *a = (const uint8_t *)&ic_ad68.pwma[ic];
        const uint8_t *b = (const uint8_t *)&ic_ad68.pwmb[ic];
        for (int byte = 0; byte < DATA_LEN; byte++) assert(a[byte] == 0 && b[byte] == 0);
        assert(ic_ad68.cfb_Tx[ic].dcc == 0 && ic_ad68.cfb_Tx[ic].dcto == 0);
        assert(ic_ad68.isDischarging[ic] == 0);
    }
}

static void test_safety_boundaries(void)
{
    assert(TOTAL_CELL == 8 && TOTAL_PACK_CELLS == 112);
    assert(BMS_BOARD_UV_MV == 24000 && BMS_BOARD_OV_MV == 33600);
    assert(BMS_PACK_UV_MV == 336000 && BMS_PACK_OV_MV == 470400);
    fresh_fixture();
    const float cellLimits[] = {BMS_MV_TO_V(BMS_CELL_UV_MV), BMS_MV_TO_V(BMS_CELL_OV_MV)};
    for (int bound = 0; bound < 2; bound++) {
        ic_ad68.v_cell[VOLTAGE_S][13][7] = cellLimits[bound];
        assert(BMS_UpdateStatusFlags() == BMS_OK);
        ic_ad68.v_cell[VOLTAGE_S][13][7] += bound == 0 ? -0.001f : 0.001f;
        assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE);
    }
    ic_ad68.v_cell[VOLTAGE_S][13][7] = NAN; assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE);
    ic_ad68.v_cell[VOLTAGE_S][13][7] = 3.75f;
    const float boardLimits[] = {BMS_MV_TO_V(BMS_BOARD_UV_MV), BMS_MV_TO_V(BMS_BOARD_OV_MV)};
    for (int bound = 0; bound < 2; bound++) {
        ic_ad68.v_segment[13] = boardLimits[bound]; assert(BMS_UpdateStatusFlags() == BMS_OK);
        ic_ad68.v_segment[13] += bound == 0 ? -0.001f : 0.001f;
        assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE);
    }
    ic_ad68.v_segment[13] = 30;
    ic_ad68.temp_cell[13][7] = BMS_TEMP_MIN_C; assert(BMS_UpdateStatusFlags() == BMS_OK);
    ic_ad68.temp_cell[13][7] = BMS_TEMP_MAX_C; assert(BMS_UpdateStatusFlags() == BMS_OK);
    ic_ad68.temp_cell[13][7] += 0.1f; assert(BMS_UpdateStatusFlags() == BMS_ERR_TEMP);
    puts("PASS cell/board inclusive OV/UV, nonfinite rejection, thermal boundaries and topology-derived 8/112-position limits");
}

static void test_snapshot_freshness_and_diagnostic(void)
{
    fresh_fixture();
    verifyTransaction = true; cellRaw = 16000;
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_OK);
    verifyTransaction = false;
    assert(fabsf(ic_ad68.v_cell[VOLTAGE_S][0][0] - 3.9f) < 0.0001f);
    fresh_fixture();
    BMS_PackMeasurements pack; BMS_GetPackMeasurements(&pack);
    assert(pack.ivt.u1.valid && pack.cell_voltage_sum_valid && fabsf(pack.voltage_difference_v + 20) < 0.0001f);
    assert(BMS_UpdateStatusFlags() == BMS_OK); // Disagreement is diagnostic, not an invented fault threshold.
    receive(true, 1, BMS_PACK_CURRENT_MAX_MA + 1);
    assert(BMS_CheckCurrent() == BMS_ERR_CURRENT); // Live detection before the next full acquisition.
    receive(true, 2, 0);
    assert(BMS_CheckCurrent() == BMS_OK);
    BMS_EnableCharging(true); assert(BMS_IsCharging());
    uint32_t acquired = now;
    now = acquired + BMS_ADBMS_CELL_MAX_AGE_MS;
    receive(true, 3, 0); receive(false, 1, 400000);
    assert(BMS_CheckPackSensor() == BMS_OK && BMS_CheckAdbmsData() == BMS_ERR_COMMS);
    BMS_GetPackMeasurements(&pack);
    assert(!pack.cell_voltage_sum_valid && isnan(pack.cell_voltage_sum_v) && isnan(pack.voltage_difference_v));
    assert(BMS_UpdateStatusFlags() & BMS_ERR_COMMS);
    BMS_EnableCharging(true); assert(!BMS_IsCharging());
    CanTxMsg *messages; uint32_t count; BMS_GetCanData(&messages, &count);
    assert(count == 130);
    CanTxMsg *cell = find(messages, count, BASE_CAN_ID);
    assert(cell->data[0] == 0 && cell->data[1] == 0x80 && !(cell->data[7] & BMS_CAN_CELL_VOLTAGE_VALID));
    CanTxMsg *cross = find(messages, count, BMS_PACK_CROSSCHECK_CAN_ID);
    assert(value32(cross->data) == INT32_MIN && value32(cross->data + 4) == INT32_MIN);
    assert(value32(find(messages, count, BMS_PACK_U1_CAN_ID)->data) == 400000);
    now = UINT32_MAX - 30; fresh_fixture();
    uint32_t timestamp = now; now = timestamp + BMS_ADBMS_CELL_MAX_AGE_MS - 1;
    assert(bms_cellDataFresh(VOLTAGE_S)); now++; assert(!bms_cellDataFresh(VOLTAGE_S));
    puts("PASS transactional cell publication, independent IVT/cell disagreement, stale ADBMS protection/telemetry/cross-check and tick rollover");
}

static void test_acquisition_failures(void)
{
    for (unsigned int group = 1; group <= 3; group++) {
        fresh_fixture(); readCount = 0; failPecRead = group;
        assert(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS);
        assert(!bms_cellDataFresh(VOLTAGE_S) && !bms_auxDataFresh());
        assert(isnan(ic_ad68.v_cell[VOLTAGE_S][13][7]) && isnan(ic_ad68.v_temp[13][7]));
        assert(ic_common.last_pec_error); assert_outputs_off();
    }
    for (unsigned int group = 1; group <= 5; group++) {
        fresh_fixture(); readCount = 0; failPecRead = group;
        assert(bms_getAuxMeasurement() == BMS_ERR_COMMS);
        assert(!bms_cellDataFresh(VOLTAGE_S) && !bms_auxDataFresh()); assert_outputs_off();
    }
    fresh_fixture(); failRead = true;
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS);
    assert(!ic_common.cell_voltage_sum_valid && ic_common.last_acquisition_transport_fault == BMS_TRANSPORT_RX);
    fresh_fixture(); resetCellCode = true;
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS && !ic_common.cell_voltage_sum_valid);
    fresh_fixture(); failPoll = true;
    assert(bms_getAuxMeasurement() == BMS_ERR_COMMS);
    assert(ic_common.last_acquisition_transport_fault == BMS_TRANSPORT_POLL_TIMEOUT);
    assert_outputs_off();
    fresh_fixture(); failWrite = true; writeCount = commandCount = 0; chargerConfig.disable_charging = 0;
    assert(bms_stopDischarge() == BMS_ERR_COMMS && writeCount == 3 && commandCount == 1);
    assert(ic_common.discharge_stop_failed && !BMS_IsCharging()); assert_outputs_off();
    failWrite = false;
    fresh_fixture(); badStopReadback = true;
    assert(bms_stopDischarge() == BMS_ERR_COMMS && ic_common.discharge_stop_failed);
    assert(!ic_common.cell_voltage_sum_valid); assert_outputs_off();
    badStopReadback = false;
    puts("PASS every cell/AUX PEC-failure group, HAL read/poll failures, reset-code rejection, complete invalidation and bounded failed stop attempts");
}

static void test_balancing_and_ids(void)
{
    fresh_fixture();
    memset(ic_ad68.pwma, 0xFF, sizeof ic_ad68.pwma);
    memset(ic_ad68.pwmb, 0xFF, sizeof ic_ad68.pwmb);
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        ic_ad68.cfb_Tx[ic].dcc = 0xFFFF; ic_ad68.isDischarging[ic] = 0xFFFF;
    }
    bms_resetConfig(); assert_outputs_off();
    assert(bms_init() == BMS_OK && !enableBalancing); assert_outputs_off();
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++)
        for (int b = 0; b < 6; b++) assert(capturedA[ic][b] == 0 && capturedB[ic][b] == 0);
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++)
        assert(capturedConfig[ic][0] == 0 && capturedConfig[ic][1] == 0xF8 && capturedConfig[ic][2] == 0x7F);
    fresh_fixture(); BMS_EnableBalancing(true);
#if BMS_BALANCING_CONFIG_CONFIRMED && BMS_BALANCING_MAX_PWM > 0
    assert(enableBalancing);
    assert(bms_startDischarge(3.7f) == BMS_OK);
    for (unsigned int ic = 0; ic < TOTAL_AD68; ic++) {
        assert(ic_ad68.isDischarging[ic] == BMS_POPULATED_CELL_MASK);
        assert(capturedA[ic][0] != 0); // Prove the mask with active populated channels.
        assert(capturedA[ic][4] == 0 && capturedA[ic][5] == 0);
        for (int b = 0; b < 6; b++) assert(capturedB[ic][b] == 0);
        assert(capturedConfig[ic][4] == 0 && capturedConfig[ic][5] == 0);
        for (unsigned int c = TOTAL_CELL; c < AD68_HARDWARE_CELL_CHANNELS; c++) bms_setPwm(ic, c, 15);
    }
#else
    assert(!enableBalancing && BMS_BALANCING_MAX_PWM == 0);
    assert(bms_startDischarge(3.7f) == BMS_OK); assert_outputs_off();
#endif
    assert(bms_stopDischarge() == BMS_OK); assert_outputs_off();
    assert(bms_softReset() == BMS_OK); assert_outputs_off();
    CanTxMsg *messages; uint32_t count; BMS_GetCanData(&messages, &count);
    assert(count == CAN_BUFFER_LEN && count == 130);
    bool seen[129] = {false};
    for (uint32_t a = 0; a < count; a++) {
        assert(messages[a].header.IdType == FDCAN_EXTENDED_ID);
        if (messages[a].header.Identifier != CHARGER_CONFIG_CAN_ID) {
            uint32_t slot = messages[a].header.Identifier - BASE_CAN_ID;
            assert(slot < 129 && !seen[slot]); seen[slot] = true;
        }
        for (uint32_t b = a + 1; b < count; b++) assert(messages[a].header.Identifier != messages[b].header.Identifier);
    }
    for (int slot = 0; slot < 129; slot++) assert(seen[slot]);
    assert(BMS_PACK_CURRENT_CAN_ID == 0xB07E && BMS_PACK_U1_CAN_ID == 0xB07F && BMS_PACK_CROSSCHECK_CAN_ID == 0xB080);
    puts("PASS bring-up balancing lock, all sixteen PWM positions, DCC zero, init/stop/reset masks and exhaustive 130-frame telemetry ID uniqueness");
}

static void test_counter_integration(void)
{
    fresh_fixture();
    unsigned int checks = counterChecks;
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_OK && counterChecks == checks + 3);
    // Every cell register group and every AUX/status group must reject a
    // PEC-correct unexpected counter before publishing the staged values.
    for (int aux = 0; aux < 2; aux++) {
        for (unsigned int group = 1; group <= (aux ? 5U : 3U); group++) {
            fresh_fixture();
            counterFaultRead = readCount + group; counterFaultDelta = 1;
            assert((aux ? bms_getAuxMeasurement() : bms_readCellVoltage(VOLTAGE_S)) == BMS_ERR_COMMS);
            BMS_AdbmsDataStatus status; BMS_GetAdbmsDataStatus(&status);
            assert(!status.cell_valid && !status.aux_valid && status.last_counter_error && !status.last_pec_error);
            assert(status.counters.device[13].fault && !status.counters.device[0].fault);
            assert(status.counters.device[13].received == status.counters.device[13].expected + 1);
            assert(isnan(ic_ad68.v_cell[VOLTAGE_S][0][0]) && !enableBalancing && !BMS_IsCharging());
            assert_outputs_off();
            // Even a subsequent matching counter cannot clear a latched error.
            counterFaultRead = 0;
            assert(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS);
            assert(bms_softReset() == BMS_OK);
            assert(bms_init() == BMS_OK);
            BMS_GetAdbmsDataStatus(&status);
            assert(!status.last_counter_error && !status.cell_valid && !status.aux_valid);
            assert(bms_getAuxMeasurement() == BMS_OK && bms_balancingMeasureVoltage() == BMS_OK);
        }
    }
    puts("PASS counter mismatch in every cell/AUX group invalidates snapshots, latches fault, inhibits outputs and preserves primary per-device diagnostics through stop");

    fresh_fixture();
    dropCommand = true; // HAL OK but the conversion and poll never reach ICs.
    assert(bms_balancingMeasureVoltage() == BMS_ERR_COMMS);
    dropCommand = false;
    assert(ic_common.last_counter_error && !bms_cellDataFresh(VOLTAGE_S));
    assert(bms_softReset() == BMS_OK && bms_init() == BMS_OK);
    assert(bms_getAuxMeasurement() == BMS_OK && bms_balancingMeasureVoltage() == BMS_OK);
    now += BMS_ADBMS_CELL_MAX_AGE_MS;
    assert(BMS_CheckAdbmsData() == BMS_ERR_COMMS);
    // A matching SID counter must never refresh an old cell/AUX snapshot.
    uint32_t stamp = ic_common.cell_last_received_ms[VOLTAGE_S];
    assert(bms_readRegister(REG_SID) == BMS_OK);
    assert(ic_common.cell_last_received_ms[VOLTAGE_S] == stamp && !bms_cellDataFresh(VOLTAGE_S));
    puts("PASS missed conversion/poll detection, soft-reset/init recovery and counter-correct SID reads cannot revive stale data");

    fresh_fixture(); missCounterReset = true;
    assert(bms_init() == BMS_ERR_COMMS && ic_common.last_counter_error);
    missCounterReset = false;
    assert(bms_softReset() == BMS_OK && bms_init() == BMS_OK);
    assert(!ic_common.last_counter_error);
    puts("PASS rejected counter reset cannot silently resynchronize; requested reset/reinitialization succeeds without false faults");
}

int main(void)
{
    IVTS_Config config = IVTS_DefaultConfig(); assert(IVTS_Init(&sensor, &config));
    BMS_PackMeasurements pack;
    BMS_GetPackMeasurements(&pack);
    assert(isnan(pack.current_a) && isnan(pack.voltage_u1_v));
    assert(BMS_CheckPackSensor() == BMS_ERR_IVT);

    assert(bms_synchronizeCounters() == BMS_OK);
    assert(bms_readCellVoltage(VOLTAGE_S) == BMS_OK);
    now = 100; receive(true, 0, -1234); receive(false, 0, 400000);
    BMS_GetPackMeasurements(&pack);
    assert(pack.ivt.current.raw_value == -1234 && fabsf(pack.current_a + 1.234f) < 0.0001f);
    assert(fabsf(pack.voltage_u1_v - 400) < 0.0001f);
    assert(pack.cell_voltage_sum_valid && fabsf(pack.cell_voltage_sum_v - 420) < 0.0001f);
    assert(fabsf(pack.voltage_difference_v + 20) < 0.0001f);
    healthy_legacy_fixture(); assert(BMS_UpdateStatusFlags() == BMS_OK);
    puts("PASS application current preserves sign/units; IVT U1 is primary, 112-cell sum remains independent with explicit difference");

    now = 101; receive(false, 1, 100000);
    assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE); /* Cell sum remains 420 V. */
    receive(false, 2, 400000); receive(true, 1, 10001);
    assert(BMS_UpdateStatusFlags() == BMS_ERR_CURRENT);
    receive(true, 2, -10001); assert(BMS_UpdateStatusFlags() == BMS_ERR_CURRENT);
    now = 401; receive(true, 3, 0);
    assert(BMS_UpdateStatusFlags() == (BMS_ERR_VOLTAGE | BMS_ERR_IVT));
    assert(BMS_CheckCommsFault() == BMS_OK && BMS_CheckPackSensor() == BMS_ERR_IVT);
    receive(false, 3, 400000); now = 501;
    assert(BMS_UpdateStatusFlags() == (BMS_ERR_CURRENT | BMS_ERR_IVT));
    now = 502;
    receive(true, 4, 10000); receive(false, 4, 470400);
    assert(BMS_UpdateStatusFlags() == BMS_OK);
    receive(true, 5, -10000); receive(false, 5, 336000);
    assert(BMS_UpdateStatusFlags() == BMS_OK);
    receive(false, 6, 470401); assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE);
    receive(false, 7, 335999); assert(BMS_UpdateStatusFlags() == BMS_ERR_VOLTAGE);
    puts("PASS pack protection uses IVT rather than cell sum, retains +/-10 A limits, rejects independent stale channels and separates IVT from isoSPI faults");
    puts("PASS exact 336.0/470.4 V and +/-10 A boundaries, with out-of-range 1 mV values rejected");

    now = 502; receive(true, 6, -12345); receive(false, 8, 400000);
    CanTxMsg *messages; uint32_t count;
    BMS_GetCanData(&messages, &count); assert(count == 130 && count <= CAN_BUFFER_LEN);
    for (uint32_t a = 0; a < count; a++)
        for (uint32_t b = a + 1; b < count; b++) assert(messages[a].header.Identifier != messages[b].header.Identifier);
    CanTxMsg *i = find(messages, count, BMS_PACK_CURRENT_CAN_ID);
    CanTxMsg *u = find(messages, count, BMS_PACK_U1_CAN_ID);
    CanTxMsg *cross = find(messages, count, BMS_PACK_CROSSCHECK_CAN_ID);
    assert(value32(i->data) == -12345 && (i->data[6] & BMS_PACK_VALID));
    assert(value32(u->data) == 400000 && (u->data[6] & BMS_PACK_VALID));
    assert(value32(cross->data) == 420000 && value32(cross->data + 4) == -20000);
    now = 602; BMS_GetCanData(&messages, &count);
    i = find(messages, count, BMS_PACK_CURRENT_CAN_ID); u = find(messages, count, BMS_PACK_U1_CAN_ID);
    assert(value32(i->data) == INT32_MIN && (i->data[6] & BMS_PACK_STALE) && !(i->data[6] & BMS_PACK_VALID));
    assert(value32(u->data) == 400000 && (u->data[6] & BMS_PACK_VALID));
    receive(false, 9, INT32_MIN); BMS_GetCanData(&messages, &count);
    cross = find(messages, count, BMS_PACK_CROSSCHECK_CAN_ID);
    assert(value32(cross->data + 4) == INT32_MIN); /* Negative difference does not overflow int32. */
    badPec = true; assert(bms_readCellVoltage(VOLTAGE_S) == BMS_ERR_COMMS);
    BMS_GetPackMeasurements(&pack); assert(!pack.cell_voltage_sum_valid && pack.ivt.u1.valid);
    puts("PASS 130 unique telemetry IDs, full signed 32-bit units, invalid/stale flags, safe cross-check encoding and independent cell-read failure");
    puts("ALL PACK INTEGRATION TESTS PASSED");
    test_safety_boundaries(); test_snapshot_freshness_and_diagnostic();
    test_acquisition_failures(); test_balancing_and_ids();
    test_counter_integration();
    puts("ALL SAFETY/ADBMS APPLICATION TESTS PASSED");
    return 0;
}
