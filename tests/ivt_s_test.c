#include "ivt_s.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static IVTS_Driver driver;

static void reset(void)
{
    IVTS_Config config = IVTS_DefaultConfig();
    assert(IVTS_Init(&driver, &config));
}

static IVTS_Frame result(uint16_t id, uint8_t mux, uint8_t counter, uint8_t state, int32_t value)
{
    uint32_t bits = (uint32_t)value;
    IVTS_Frame frame = {.id = id, .dlc = 6, .data = {mux, (uint8_t)((state << 4) | counter),
        (uint8_t)(bits >> 24), (uint8_t)(bits >> 16), (uint8_t)(bits >> 8), (uint8_t)bits}};
    return frame;
}

static IVTS_Snapshot snapshot(uint32_t now)
{
    IVTS_Snapshot value;
    IVTS_GetSnapshot(&driver, now, &value);
    return value;
}

static void test_signed_and_big_endian(void)
{
    reset();
    IVTS_Frame frame = result(0x521, 0, 3, 0, 123456);
    assert(IVTS_Receive(&driver, &frame, 10) == IVTS_CURRENT);
    assert(snapshot(10).current.raw_value == 123456 && snapshot(10).current.valid);
    frame = result(0x521, 0, 4, 0, -123456);
    IVTS_Receive(&driver, &frame, 20);
    assert(snapshot(20).current.raw_value == -123456 && snapshot(20).current.valid);
    /* Fixed datasheet example, independent of the frame builder. */
    frame = (IVTS_Frame){.id = 0x522, .dlc = 6, .data = {1, 5, 0, 0, 0x88, 0xB8}};
    assert(IVTS_Receive(&driver, &frame, 30) == IVTS_U1);
    IVTS_Snapshot s = snapshot(30);
    assert(s.u1.valid && s.u1.raw_value == 35000 && s.u1.counter == 5);
    assert(s.u1.raw_status == 5 && s.u1.result_state == 0 && s.u1.last_received_ms == 30);
    frame = result(0x522, 1, 6, 0, 420123);
    IVTS_Receive(&driver, &frame, 40);
    assert(snapshot(40).u1.raw_value == 420123);
    int32_t limits[] = {INT32_MIN, INT32_MAX, -1, 0};
    for (unsigned int i = 0; i < sizeof limits / sizeof *limits; i++) {
        frame = result(0x521, 0, 5 + i, 0, limits[i]);
        IVTS_Receive(&driver, &frame, 50 + i);
        assert(snapshot(50 + i).current.valid && snapshot(50 + i).current.raw_value == limits[i]);
    }
    puts("PASS positive/negative current, positive U1, golden big-endian vector and signed 32-bit limits");
}

static void test_frame_validation(void)
{
    for (int dlc = 0; dlc <= 15; dlc++) {
        if (dlc == 6) continue;
        reset();
        IVTS_Frame good = result(0x521, 0, 0, 0, 100);
        IVTS_Receive(&driver, &good, 10);
        good.dlc = dlc;
        assert(IVTS_Receive(&driver, &good, 20) == IVTS_REJECTED);
        assert(!snapshot(20).current.valid && snapshot(20).current.last_received_ms == 20);
        assert(snapshot(20).current.last_measurement_ms == 10);
    }
    for (int channel = 0; channel < 2; channel++) {
        reset();
        IVTS_Frame frame = result(0x521 + channel, 1 - channel, 0, 0, 1);
        assert(IVTS_Receive(&driver, &frame, 0) == IVTS_REJECTED);
        assert(!snapshot(0).current.valid && !snapshot(0).u1.valid);
    }
    reset();
    IVTS_Frame frame = result(0x520, 0, 0, 0, 123);
    assert(IVTS_Receive(&driver, &frame, 10) == IVTS_IGNORED);
    assert(!snapshot(10).current.received);
    for (int format = 0; format < 3; format++) {
        frame = result(0x521, 0, 0, 0, 123);
        frame.extended = format == 0; frame.remote = format == 1; frame.fd = format == 2;
        assert(IVTS_Receive(&driver, &frame, 20) == IVTS_REJECTED);
        assert(!snapshot(20).current.valid);
    }
    puts("PASS wrong ID, every wrong DLC, wrong current/U1 mux, extended/RTR/FD rejection and no malformed-frame freshness refresh");
}

static void test_states(void)
{
    for (int channel = 0; channel < 2; channel++) {
        for (uint8_t state = 0; state < 16; state++) {
            reset();
            IVTS_Frame frame = result(0x521 + channel, channel, 7, state, 123);
            IVTS_Receive(&driver, &frame, 100);
            IVTS_Snapshot s = snapshot(100);
            IVTS_Result r = channel == 0 ? s.current : s.u1;
            assert(r.valid == ((state & 0xE) == 0));
            assert(r.result_state == state && r.raw_status == ((state << 4) | 7));
            assert(r.fresh && !r.stale);
        }
    }
    for (int bit = 2; bit <= 3; bit++) {
        reset();
        IVTS_Frame i = result(0x521, 0, 0, 0, 10), u = result(0x522, 1, 0, 0, 400000);
        IVTS_Receive(&driver, &i, 10); IVTS_Receive(&driver, &u, 10);
        i = result(0x521, 0, 1, 1U << bit, 10);
        IVTS_Receive(&driver, &i, 20);
        assert(!snapshot(20).current.valid && !snapshot(20).u1.valid);
        u = result(0x522, 1, 1, 0, 400000); IVTS_Receive(&driver, &u, 30);
        assert(snapshot(30).u1.valid && !snapshot(30).current.valid);
    }
    puts("PASS system error, bits 1/2, all state combinations, OCS diagnostics and cross-channel error inhibition/recovery");
}

static void test_counters(void)
{
    reset();
    IVTS_Frame frame = result(0x521, 0, 15, 0, 100);
    IVTS_Receive(&driver, &frame, 0);
    frame = result(0x521, 0, 0, 0, 101); IVTS_Receive(&driver, &frame, 10);
    assert(snapshot(10).current.valid && snapshot(10).current.counter_gap_count == 0);
    frame = result(0x521, 0, 0, 0, 999); IVTS_Receive(&driver, &frame, 20);
    assert(!snapshot(20).current.valid && snapshot(20).current.duplicate_count == 1);
    assert(snapshot(20).current.last_received_ms == 20 && snapshot(20).current.last_measurement_ms == 10);
    assert(snapshot(110).current.stale); /* Duplicate never extends the watchdog. */
    frame = result(0x521, 0, 3, 0, 102); IVTS_Receive(&driver, &frame, 120);
    assert(snapshot(120).current.valid && snapshot(120).current.counter_gap_count == 1);
    puts("PASS counter rollover 15->0, duplicate rejection without freshness extension, gap diagnostics and recovery");
}

static void test_freshness(void)
{
    reset();
    assert(snapshot(0).current.stale && snapshot(0).u1.stale);
    IVTS_Frame i = result(0x521, 0, 0, 0, 1), u = result(0x522, 1, 0, 0, 400000);
    IVTS_Receive(&driver, &i, 100); IVTS_Receive(&driver, &u, 100);
    assert(snapshot(199).current.valid && !snapshot(199).current.stale);
    assert(!snapshot(200).current.valid && snapshot(200).current.stale && snapshot(200).u1.valid);
    assert(snapshot(399).u1.valid && snapshot(400).u1.stale);
    i = result(0x521, 0, 1, 0, 2); IVTS_Receive(&driver, &i, 450);
    assert(snapshot(450).current.valid && snapshot(450).u1.stale);
    u = result(0x522, 1, 1, 0, 400001); IVTS_Receive(&driver, &u, 600);
    assert(snapshot(600).current.stale && snapshot(600).u1.valid);
    reset();
    i = result(0x521, 0, 0, 0, 1); IVTS_Receive(&driver, &i, UINT32_MAX - 49U);
    assert(snapshot(49).current.valid && snapshot(50).current.stale);
    puts("PASS stale current, stale voltage, current fresh/voltage stale, voltage fresh/current stale, exact deadlines and tick wrap");
}

static void test_alive_configuration_and_transport(void)
{
    reset();
    IVTS_Frame i = result(0x521, 0, 0, 0, 1), u = result(0x522, 1, 0, 0, 400000);
    IVTS_Receive(&driver, &i, 0); IVTS_Receive(&driver, &u, 0);
    assert(!snapshot(0).configuration_verified);
    IVTS_SetConfigurationVerified(&driver, true);
    IVTS_Frame alive = {.id = 0x511, .dlc = 8, .data = {0xBF, 4, 0x11, 0x12, 0x34, 0x56, 0x78, 0}};
    assert(IVTS_Receive(&driver, &alive, 10) == IVTS_ALIVE);
    IVTS_Snapshot s = snapshot(10);
    assert(s.alive.seen && s.alive.command_id == 0x411 && s.alive.serial_number == 0x12345678);
    assert(s.alive.count == 1 && s.alive.last_received_ms == 10);
    assert(!s.current.valid && !s.u1.valid && !s.configuration_verified);
    assert(IVTS_Receive(&driver, &i, 20) == IVTS_CURRENT); /* Counter restarted after Alive. */
    assert(snapshot(20).current.valid && !snapshot(20).u1.valid);
    alive.dlc = 7; assert(IVTS_Receive(&driver, &alive, 21) == IVTS_REJECTED);
    assert(snapshot(21).alive.count == 1);
    alive.dlc = 8; alive.data[0] = 0xA1;
    assert(IVTS_Receive(&driver, &alive, 22) == IVTS_RESPONSE);
    assert(snapshot(22).response.data[0] == 0xA1 && !snapshot(22).configuration_verified);
    IVTS_InvalidateTransport(&driver);
    assert(snapshot(23).transport_error && !snapshot(23).current.valid && !snapshot(23).u1.valid);
    IVTS_Receive(&driver, &i, 30);
    assert(snapshot(30).current.valid && !snapshot(30).u1.valid);
    assert(IVTS_IsSupportedBitrate(250000) && IVTS_IsSupportedBitrate(500000) && IVTS_IsSupportedBitrate(1000000));
    assert(!IVTS_IsSupportedBitrate(125000));
    IVTS_Config config = IVTS_DefaultConfig(); config.u1_timeout_ms = 0;
    assert(!IVTS_Init(&driver, &config));
    config = IVTS_DefaultConfig(); config.current_id = 0x800;
    assert(!IVTS_Init(&driver, &config));
    puts("PASS Alive diagnostics/restart invalidation, response extension point, unverified configuration, bus-loss invalidation and supported bitrates");
}

int main(void)
{
    test_signed_and_big_endian(); test_frame_validation(); test_states();
    test_counters(); test_freshness(); test_alive_configuration_and_transport();
    puts("ALL IVT-S PROTOCOL TESTS PASSED");
    return 0;
}
