#include "ivt_s.h"
#include <limits.h>
#include <string.h>

IVTS_Config IVTS_DefaultConfig(void)
{
    return (IVTS_Config){IVTS_CURRENT_ID, IVTS_U1_ID, IVTS_RESPONSE_ID,
        IVTS_CURRENT_TIMEOUT_MS, IVTS_U1_TIMEOUT_MS};
}

bool IVTS_IsSupportedBitrate(uint32_t bitrate)
{
    return bitrate == 250000U || bitrate == 500000U || bitrate == 1000000U;
}

bool IVTS_Init(IVTS_Driver *driver, const IVTS_Config *config)
{
    if (!driver || !config) return false;
    memset(driver, 0, sizeof(*driver));
    if (config->current_id > 0x7FFU || config->u1_id > 0x7FFU || config->response_id > 0x7FFU
            || config->current_id == config->u1_id || config->current_id == config->response_id
            || config->u1_id == config->response_id
            || config->current_timeout_ms == 0 || config->current_timeout_ms > INT32_MAX
            || config->u1_timeout_ms == 0 || config->u1_timeout_ms > INT32_MAX) return false;
    driver->config = *config;
    return true;
}

static uint32_t read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16)
            | ((uint32_t)data[2] << 8) | data[3];
}

static int32_t read_i32_be(const uint8_t *data)
{
    const uint32_t bits = read_u32_be(data);
    /* Avoid implementation-defined unsigned-to-signed overflow. */
    return bits <= INT32_MAX ? (int32_t)bits : (int32_t)((int64_t)bits - INT64_C(4294967296));
}

static void invalidate_result(IVTS_Result *result)
{
    result->protocol_valid = false;
    result->has_result = false;
    result->counter_seen = false;
}

void IVTS_InvalidateTransport(IVTS_Driver *driver)
{
    driver->state.transport_error = true;
    invalidate_result(&driver->state.current);
    invalidate_result(&driver->state.u1);
}

void IVTS_SetConfigurationVerified(IVTS_Driver *driver, bool verified)
{
    driver->state.configuration_verified = verified;
}

IVTS_RxEvent IVTS_Receive(IVTS_Driver *driver, const IVTS_Frame *frame, uint32_t now_ms)
{
    if (!driver || !frame) return IVTS_REJECTED;
    if (frame->extended || frame->remote || frame->fd || frame->id > 0x7FFU) return IVTS_REJECTED;

    if (frame->id == driver->config.response_id) {
        /* Response messages have DLC 8 (p19); Alive uses DB0-DB6 (p40).
         * Treat Alive on the response ID as diagnostic, never a measurement. */
        if (frame->dlc != 8) return IVTS_REJECTED;
        if (frame->data[0] == 0xBF && frame->data[1] > 7) return IVTS_REJECTED;
        memcpy(driver->state.response.data, frame->data, 8);
        driver->state.response.received = true;
        driver->state.response.last_received_ms = now_ms;
        if (frame->data[0] != 0xBF) return IVTS_RESPONSE;
        IVTS_Alive *alive = &driver->state.alive;
        alive->seen = true;
        alive->command_id = ((uint16_t)frame->data[1] << 8) | frame->data[2];
        alive->serial_number = read_u32_be(&frame->data[3]);
        alive->last_received_ms = now_ms;
        alive->count++;
        driver->state.configuration_verified = false;
        invalidate_result(&driver->state.current);
        invalidate_result(&driver->state.u1);
        return IVTS_ALIVE;
    }

    IVTS_Result *result, *other;
    uint8_t mux;
    IVTS_RxEvent event;
    if (frame->id == driver->config.current_id) {
        result = &driver->state.current; other = &driver->state.u1;
        mux = 0; event = IVTS_CURRENT;
    } else if (frame->id == driver->config.u1_id) {
        result = &driver->state.u1; other = &driver->state.current;
        mux = 1; event = IVTS_U1;
    } else return IVTS_IGNORED;

    result->received = true;
    result->last_received_ms = now_ms;
    if (frame->dlc != 6 || frame->data[0] != mux) {
        result->protocol_valid = false;
        return IVTS_REJECTED;
    }

    const uint8_t counter = frame->data[1] & 0x0F;
    result->duplicate = result->counter_seen && counter == result->counter;
    if (result->duplicate) result->duplicate_count++;
    else {
        if (result->counter_seen && counter != ((result->counter + 1U) & 0x0FU)) result->counter_gap_count++;
        result->last_measurement_ms = now_ms;
    }
    result->counter_seen = true;
    result->has_result = true;
    result->counter = counter;
    result->raw_status = frame->data[1];
    result->result_state = frame->data[1] >> 4;
    result->raw_value = read_i32_be(&frame->data[2]);
    result->protocol_valid = true;
    /* Bit 1 cannot distinguish reduced precision from measurement error.
     * Conservatively reject it. Bit 2 is an unspecified-channel measurement
     * error: reject this result and inhibit the other cached result until it
     * receives a new clean result. Bit 3 has the same cross-channel inhibition.
     * Bit 0 is OCS; retain the measurement and expose the diagnostic unchanged. */
    result->status_valid = (result->result_state & IVTS_STATE_INVALID_MASK) == 0;
    result->peer_error = false;
    if (result->result_state & (IVTS_STATE_ANY_RESULT_ERROR | IVTS_STATE_SYSTEM_ERROR)) other->peer_error = true;
    driver->state.transport_error = false;
    return event;
}

static void evaluate_result(IVTS_Result *result, uint32_t now_ms, uint32_t timeout_ms)
{
    result->fresh = result->has_result && (uint32_t)(now_ms - result->last_measurement_ms) < timeout_ms;
    result->stale = !result->fresh; /* Includes never received / reset. */
    result->valid = result->fresh && result->protocol_valid && result->status_valid
            && !result->duplicate && !result->peer_error;
}

void IVTS_GetSnapshot(const IVTS_Driver *driver, uint32_t now_ms, IVTS_Snapshot *snapshot)
{
    *snapshot = driver->state;
    evaluate_result(&snapshot->current, now_ms, driver->config.current_timeout_ms);
    evaluate_result(&snapshot->u1, now_ms, driver->config.u1_timeout_ms);
}
