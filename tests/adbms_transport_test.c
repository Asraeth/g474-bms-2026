/* Actual SPI/PEC/poll implementation, with HAL and CS mocked. */
#include <assert.h>
#include <stdio.h>
#include "../Core/custom_lib/src/bms_utility.c"

SPI_HandleTypeDef hspi1;
static uint32_t tick;
static bool contextAllowed = true, cs, failTx, failRx, neverReady;
static unsigned int reads, writes;
static unsigned int failTxAt;
static uint8_t wire[TOTAL_IC][DATAPKT_LEN];
static uint8_t captured[TOTAL_IC][DATAPKT_LEN];

uint32_t HAL_GetTick(void) { return tick++; }
bool bms_spiContextAllowed(void) { return contextAllowed; }
void bms_csLow(void) { assert(!cs); cs = true; }
void bms_csHigh(void) { assert(cs); cs = false; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)spi; assert(cs && timeout == BMS_ADBMS_SPI_TIMEOUT_MS);
    writes++;
    if (size == sizeof captured) memcpy(captured, data, size);
    return failTx || (failTxAt && writes == failTxAt) ? HAL_TIMEOUT : HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)spi; assert(cs && timeout > 0 && timeout <= BMS_ADBMS_SPI_TIMEOUT_MS);
    reads++;
    if (failRx) return HAL_TIMEOUT;
    if (size == 1) *data = neverReady ? 0 : (reads == 1 ? 0 : reads == 2 ? 0x0F : 0xFF);
    else { assert(size == sizeof wire); memcpy(data, wire, size); }
    return HAL_OK;
}

static uint16_t reference_pec(const uint8_t *data, unsigned int cc, bool includeCounter)
{
    unsigned int crc = 16;
    for (int byte = 0; byte < DATA_LEN; byte++) {
        crc ^= (unsigned int)data[byte] << 2;
        for (int bit = 0; bit < 8; bit++) crc = ((crc << 1) ^ ((crc & 0x200) ? 0x8F : 0)) & 0x3FF;
    }
    if (includeCounter) crc ^= cc << 4;
    for (int bit = 0; bit < 6; bit++) crc = ((crc << 1) ^ ((crc & 0x200) ? 0x8F : 0)) & 0x3FF;
    return crc;
}

static void counter_packet(uint8_t counter)
{
    for (unsigned int i = 0; i < TOTAL_IC; i++) {
        uint16_t crc = reference_pec(wire[i], counter, true);
        wire[i][6] = (counter << 2) | (crc >> 8); wire[i][7] = crc;
    }
}

static bool counter_read(uint8_t counter)
{
    uint8_t command[] = {0, 0x2C}, data[TOTAL_IC][DATA_LEN], cc[TOTAL_IC];
    uint16_t pec[TOTAL_IC]; bool errors[TOTAL_IC];
    counter_packet(counter);
    assert(bms_receiveData(command, data, pec, cc) == BMS_OK);
    assert(!bms_checkRxPec(data, pec, cc, errors));
    return bms_checkRxCounter(cc, errors, errors);
}

static void test_counter_transport(void)
{
    uint8_t reset[] = {0,0x2E}, softReset[] = {0,0x27}, mute[] = {0,0x28};
    uint8_t adc[] = {2,0x60}, poll[] = {7,0x1D}, write[] = {0,1}, unknown[] = {7,0x19};
    uint8_t payload[TOTAL_IC][DATA_LEN] = {{0}};
    assert(bms_transmitCmd(reset) == BMS_OK && !counter_read(0));
    assert(bms_transmitCmd(adc) == BMS_OK && !counter_read(1));
    reads = 0; assert(bms_transmitPoll(poll) == BMS_OK && reads == 3);
    assert(!counter_read(2)); // Three busy/readiness bytes, one command INC.
    assert(bms_transmitData(write, payload) == BMS_OK && !counter_read(3));
    assert(!counter_read(3)); // No INC for read commands.
    for (int i = 0; i < 60; i++) assert(bms_transmitCmd(mute) == BMS_OK);
    assert(!counter_read(63));
    assert(bms_transmitCmd(mute) == BMS_OK && !counter_read(1));
    assert(bms_transmitCmd(mute) == BMS_OK && counter_read(1)); // Duplicate, valid PEC.
    assert(counter_read(2)); // Fault remains latched despite a later match.
    assert(bms_transmitCmd(softReset) == BMS_OK && !counter_read(0));
    assert(bms_transmitCmd(mute) == BMS_OK && counter_read(2)); // Skipped value.
    assert(bms_transmitCmd(reset) == BMS_OK && !counter_read(0));
    assert(bms_transmitCmd(mute) == BMS_OK && counter_read(0)); // Unexpected reset/sleep.
    assert(bms_transmitCmd(reset) == BMS_OK && !counter_read(0));
    puts("PASS real transport INC hooks: command, conditional write, poll once, nonincrementing reads, rollover, duplicate, skip and explicit resets");

    failTxAt = writes + 2; // Command succeeded, data phase failed/acceptance ambiguous.
    assert(bms_transmitData(write, payload) == BMS_ERR_COMMS);
    failTxAt = 0;
    assert(counter_read(0)); // May not learn a guessed hardware count.
    BMS_CommandCounterChain state; bms_getCounterState(&state);
    assert(!state.device[0].synchronized && state.device[0].fault);
    assert(bms_transmitCmd(reset) == BMS_OK && !counter_read(0));
    neverReady = true; reads = 0;
    assert(bms_transmitPoll(poll) == BMS_ERR_COMMS);
    neverReady = false;
    assert(!counter_read(1)); // Poll command INC happened even though polling timed out.
    unsigned int count = writes;
    assert(bms_transmitCmd(unknown) == BMS_ERR_COMMS && writes == count);
    assert(bms_getTransportFault() == BMS_TRANSPORT_COMMAND);
    assert(bms_transmitCmd(write) == BMS_ERR_COMMS && writes == count);
    assert(!counter_read(1));
    puts("PASS failed write desynchronization, verified reset recovery, timeout counter accounting and unsupported/wrong-path command rejection");
}

int main(void)
{
    uint8_t cmd[2] = {0x07, 0x1D};
    assert(bms_transmitPoll(cmd) == BMS_OK && reads == 3 && !cs);
    reads = 0; neverReady = true; tick = 0;
    assert(bms_transmitPoll(cmd) == BMS_ERR_COMMS && !cs);
    assert(bms_getTransportFault() == BMS_TRANSPORT_POLL_TIMEOUT && reads <= BMS_ADBMS_POLL_TIMEOUT_MS);
    reads = 0; tick = UINT32_MAX - 5;
    assert(bms_transmitPoll(cmd) == BMS_ERR_COMMS && !cs && reads <= BMS_ADBMS_POLL_TIMEOUT_MS);
    neverReady = false; failTx = true;
    assert(bms_transmitCmd(cmd) == BMS_ERR_COMMS && !cs && bms_getTransportFault() == BMS_TRANSPORT_TX);
    failTx = false; failRx = true;
    uint8_t data[TOTAL_IC][DATA_LEN]; uint16_t pec[TOTAL_IC]; uint8_t cc[TOTAL_IC];
    memset(data, 0xAA, sizeof data);
    cmd[0] = 0; cmd[1] = 3;
    assert(bms_receiveData(cmd, data, pec, cc) == BMS_ERR_COMMS && !cs);
    for (unsigned int ic = 0; ic < TOTAL_IC; ic++)
        for (int b = 0; b < 6; b += 2) assert(data[ic][b] == 0 && data[ic][b+1] == 0x80);
    failRx = false; contextAllowed = false;
    cmd[0] = 7; cmd[1] = 0x1D;
    unsigned int count = writes;
    assert(bms_transmitPoll(cmd) == BMS_ERR_COMMS && writes == count && !cs);
    assert(bms_getTransportFault() == BMS_TRANSPORT_CONTEXT);
    contextAllowed = true;
    cmd[0] = 0; cmd[1] = 3;
    puts("PASS finite poll/HAL deadlines, partial-ready rejection, tick wrap, CS cleanup, failed-read poisoning and ISR/masked-context rejection");

    for (unsigned int ic = 0; ic < TOTAL_IC; ic++) {
        for (int b = 0; b < 6; b++) wire[ic][b] = ic * 7 + b;
        unsigned int counter = ic + 1;
        uint16_t crc = reference_pec(wire[ic], counter, true);
        wire[ic][6] = (counter << 2) | (crc >> 8); wire[ic][7] = crc;
    }
    assert(bms_receiveData(cmd, data, pec, cc) == BMS_OK);
    bool errors[TOTAL_IC];
    assert(!bms_checkRxPec(data, pec, cc, errors));
    data[13][5] ^= 1;
    assert(bms_checkRxPec(data, pec, cc, errors) && errors[13] && !errors[0]);
    data[13][5] ^= 1; cc[0] ^= 1;
    assert(bms_checkRxPec(data, pec, cc, errors) && errors[0] && !errors[13]);
    cmd[1] = 0x20;
    assert(bms_transmitData(cmd, data) == BMS_OK);
    for (unsigned int wireIndex = 0; wireIndex < TOTAL_IC; wireIndex++) {
        assert(memcmp(captured[wireIndex], data[TOTAL_IC - 1 - wireIndex], 6) == 0);
        uint16_t crc = reference_pec(captured[wireIndex], 0, false);
        assert(captured[wireIndex][6] == (crc >> 8) && captured[wireIndex][7] == (crc & 0xFF));
    }
    puts("PASS real CRC10 vs independent bitwise reference, data/CC corruption detection and fourteen reversed six-byte write payloads");
    test_counter_transport();
    puts("ALL ADBMS TRANSPORT TESTS PASSED");
    return 0;
}
