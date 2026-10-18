/* IVT-S Version 1.01 protocol. HAL-independent; caller serializes access. */
#ifndef IVT_S_H
#define IVT_S_H

#include <stdbool.h>
#include <stdint.h>
#include "config/bms_sensor_config.h"
#include "config/bms_bus_config.h"

#define IVTS_CURRENT_ID       CFG_IVT_ID_RESULT_I
#define IVTS_U1_ID            CFG_IVT_ID_RESULT_U1
#define IVTS_U2_ID            CFG_IVT_ID_RESULT_U2
#define IVTS_ENERGY_ID        CFG_IVT_ID_RESULT_WH
#define IVTS_RESPONSE_ID      CFG_IVT_ID_RESPONSE
#define IVTS_DEFAULT_BITRATE  CFG_CAN2_BITRATE
#ifndef IVTS_CAN_BITRATE
#define IVTS_CAN_BITRATE IVTS_DEFAULT_BITRATE
#endif

/* Application watchdog choices, not datasheet requirements. Independently
 * configurable; U1 might be disabled and is never assumed to be transmitting. */
#define IVTS_CURRENT_TIMEOUT_MS CFG_IVT_CURRENT_TIMEOUT_MS
#define IVTS_U1_TIMEOUT_MS      CFG_IVT_VOLTAGE_TIMEOUT_MS

#define IVTS_STATE_OCS              (1U << 0)
#define IVTS_STATE_RESULT_ERROR     (1U << 1)
#define IVTS_STATE_ANY_RESULT_ERROR (1U << 2)
#define IVTS_STATE_SYSTEM_ERROR     (1U << 3)
#define IVTS_STATE_INVALID_MASK (IVTS_STATE_RESULT_ERROR | IVTS_STATE_ANY_RESULT_ERROR | IVTS_STATE_SYSTEM_ERROR)

typedef struct {
    uint16_t current_id, u1_id, response_id;
    uint32_t current_timeout_ms, u1_timeout_ms;
    uint16_t u2_id, energy_id;
    uint32_t u2_timeout_ms, energy_timeout_ms;
} IVTS_Config;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    bool extended, remote, fd;
    uint8_t data[8];
} IVTS_Frame;

typedef struct {
    int32_t raw_value; /* Current: mA; U1: mV. Signed, with no polarity inversion. */
    uint8_t raw_status; /* Entire DB1, including its counter. */
    uint8_t result_state, counter;
    uint32_t last_received_ms; /* Also updated for malformed frames on this ID. */
    uint32_t last_measurement_ms; /* Last well-formed, nonduplicate result. */
    uint32_t counter_gap_count, duplicate_count;
    bool received, has_result, counter_seen;
    bool protocol_valid, status_valid, duplicate, peer_error;
    bool fresh, stale, valid; /* Derived at snapshot time, not latched on RX. */
} IVTS_Result;

typedef struct {
    bool seen;
    uint16_t command_id;
    uint32_t serial_number, last_received_ms, count;
} IVTS_Alive;

typedef struct {
    bool received;
    uint8_t data[8];
    uint32_t last_received_ms;
} IVTS_Response;

typedef struct {
    IVTS_Result current, u1, u2, energy;
    IVTS_Alive alive;
    IVTS_Response response; /* Extension point for GET/configuration verification. */
    bool configuration_verified; /* Never inferred from Alive or result traffic. */
    bool transport_error;
    uint32_t restart_epoch;
} IVTS_Snapshot;

typedef struct {
    IVTS_Config config;
    IVTS_Snapshot state;
} IVTS_Driver;

typedef enum {
    IVTS_IGNORED, IVTS_REJECTED, IVTS_CURRENT, IVTS_U1, IVTS_ALIVE, IVTS_RESPONSE, IVTS_U2, IVTS_ENERGY
} IVTS_RxEvent;

IVTS_Config IVTS_DefaultConfig(void);
bool IVTS_IsSupportedBitrate(uint32_t bitrate);
bool IVTS_Init(IVTS_Driver *driver, const IVTS_Config *config);
IVTS_RxEvent IVTS_Receive(IVTS_Driver *driver, const IVTS_Frame *frame, uint32_t now_ms);
void IVTS_GetSnapshot(const IVTS_Driver *driver, uint32_t now_ms, IVTS_Snapshot *snapshot);
void IVTS_InvalidateTransport(IVTS_Driver *driver);
/* A future verified GET/response transaction may call this; no SET/STORE is sent. */
void IVTS_SetConfigurationVerified(IVTS_Driver *driver, bool verified);

#endif
