/*
 * bms_can.c
 *
 *  CAN1 (hfdcan1): car / datalogger bus (with the inverter). BMS data out, TS request and settings in.
 *  CAN2 (hfdcan2): inside the TSAC: IVT-S current sensor.
 *  CAN3 (hfdcan3): charger.
 *
 * All callers (timer, CAN and safety interrupts) share interrupt priority 0, so they never interrupt each other.
 */

#include "bms_can.h"
#include "bms_ivt.h"
#include "bms_precharge.h"
#include "bms_settings.h"
#include "bms_charger.h"
#include "bms_charger_codec.h"
#include "bms_health.h"
#include "bms_libWrapper.h"
#include "uartDMA.h"
#include <string.h>
#include <stdbool.h>

// CAN1 measurement burst being sent
static CanTxMsg          txBuffer[CAN_BUFFER_LEN];
static volatile uint32_t txBufferHeadIndex = 0;
static volatile uint32_t txBufferTailIndex = 0;
static volatile bool     isBufferTransmitting = false;

// Counters for the debugger and the tests
volatile uint32_t canTxErrorCount = 0;
volatile uint32_t canFastTxDropped = 0;
volatile uint32_t canChargerTxDropped = 0;
volatile uint32_t canBusOffCount = 0;

#define CAN_TX_FIFO_DEPTH 3

_Static_assert(CAN_FAST_FRAME_COUNT < CAN_TX_FIFO_DEPTH, "No TX buffer left for the measurement burst");


static void configFilters(void)
{
    FDCAN_FilterTypeDef filter = {0};

    // CAN3: only the charger status frame
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x01000000;
    filter.FilterID2    = 0x1C000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN1: only the TS request from the VCU and settings requests from the GUI
    filter.IdType       = CFG_CAN_TS_REQUEST_EXTENDED ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CFG_CAN_ID_TS_REQUEST;
    filter.FilterID2    = CFG_CAN_TS_REQUEST_EXTENDED ? 0x1FFFFFFF : 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = CFG_CAN_TS_REQUEST_EXTENDED ? 1 : 0;
    filter.FilterID1    = CFG_CAN_ID_CONFIG_REQUEST;
    filter.FilterID2    = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN2: the IVT sends I, U1 and U2 back to back every few ms. I and U1 go to RX FIFO 0, U2 and
    // the energy counter to FIFO 1, so each 3-frame FIFO has room for more than one cycle.
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CFG_IVT_ID_RESULT_I;
    filter.FilterID2    = CFG_IVT_ID_RESULT_U1;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    filter.FilterIndex  = 1;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1    = CFG_IVT_ID_RESULT_U2;
    filter.FilterID2    = CFG_IVT_ID_RESULT_WH;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    // The IVT's responses, for its alive message after a (re)start
    filter.FilterIndex  = 2;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterID1    = CFG_IVT_ID_RESPONSE;
    filter.FilterID2    = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    // Reject everything else, including remote frames
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}


void BMS_CAN_Config(void)
{
    configFilters();

    /* Start the FDCAN modules */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}


static void BMS_CAN_SendMsg(const CanTxMsg* msg);


// The burst only ever holds one TX buffer, so the fast frames always find room and wait at most one
// burst frame (~160 us). The next burst frame is loaded when the TX FIFO runs empty.
static void recursiveTransmit(void)
{
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > CAN_FAST_FRAME_COUNT)
    {
        if (txBufferTailIndex >= txBufferHeadIndex)
        {
            return;
        }

        BMS_CAN_SendMsg(&txBuffer[txBufferTailIndex]);

        txBufferTailIndex++;
    }
}


static void BMS_CAN_SendMsg(const CanTxMsg* msg)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, (uint8_t*)msg->data) != HAL_OK)
    {
        canTxErrorCount++;      // Not fatal: receivers detect missing BMS frames themselves
    }
}


void BMS_CAN_SendFast(const CanTxMsg* msgs, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msgs[i].header, (uint8_t*)msgs[i].data) != HAL_OK)
        {
            canFastTxDropped++;     // Bus off or nobody acknowledging: the buffers are still full
        }
    }
}


// Charger command on CAN3. With no charger connected nothing acknowledges it, the TX FIFO stays full
// and the frames are just counted.
void BMS_CAN_SendCharger(const CanTxMsg* msg)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &msg->header, (uint8_t*)msg->data) != HAL_OK)
    {
        canChargerTxDropped++;
    }
}


void BMS_CAN_SendIvtCommand(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier          = id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, (uint8_t*)data) != HAL_OK)
    {
        canTxErrorCount++;
    }
}


// True once everything sent to the IVT has been acknowledged on the bus
bool BMS_CAN_IsIvtTxIdle(void)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == CAN_TX_FIFO_DEPTH;
}


// Drops IVT commands nobody acknowledged, so they aren't delivered late when the IVT reappears
void BMS_CAN_AbortIvtTx(void)
{
    HAL_FDCAN_AbortTxRequest(&hfdcan2, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
}


// The FDCAN stays in bus-off (INIT set) until software restarts it. Called from the safety tick.
void BMS_CAN_RecoverBusOff(void)
{
    FDCAN_HandleTypeDef* handles[] = { &hfdcan1, &hfdcan2, &hfdcan3 };

    for (int i = 0; i < 3; i++)
    {
        FDCAN_GlobalTypeDef* can = handles[i]->Instance;
        if(i==0 && !(can->PSR & FDCAN_PSR_BO))BMS_Health_ClearRecovered(BMS_ERR_CAN);
        if ((can->PSR & FDCAN_PSR_BO) && (can->CCCR & FDCAN_CCCR_INIT))
        {
            if(i==1)BMS_IVT_InvalidateTransport();
            if(i==0)BMS_Health_Latch(BMS_ERR_CAN);
            if(i==2 && BMS_IsCharging())BMS_Health_Latch(BMS_ERR_CHARGER);
            CLEAR_BIT(can->CCCR, FDCAN_CCCR_INIT);
            canBusOffCount++;
        }
    }
}


void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
    {
        if (txBufferTailIndex >= txBufferHeadIndex || isBufferTransmitting == false)
        {
            isBufferTransmitting = false;
            return;
        }
        else
        {
            recursiveTransmit();
        }
    }
}


static void CAN_AbortTx(void)
{
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0);
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER1);
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER2);
}



void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len)
{
    if (len > CAN_BUFFER_LEN)
    {
        printfDma("Error CANTX: len larger than buffer \n");
        return;
    }

    // Logged when the bursts stop getting out and when they get out again, not every period
    static uint32_t overwritten = 0;
    if (isBufferTransmitting)
    {
        isBufferTransmitting = false;       // Disable recursive callback in case some are still being sent
        CAN_AbortTx();
        if (overwritten++ == 0)
        {
            printfDma("Error: CAN TX Buffer Overwritten (CAN1 burst not getting out: nothing acknowledging?)\n");
        }
    }
    else if (overwritten > 0)
    {
        printfDma("CAN1 bursts getting out again (%lu overwritten)\n", (unsigned long)overwritten);
        overwritten = 0;
    }

    memcpy(txBuffer, msgArr, len * sizeof(CanTxMsg));
    txBufferHeadIndex = len;
    txBufferTailIndex = 0;
    isBufferTransmitting = true;

    recursiveTransmit();
}

ChargerStatus chargerStatus = {0};

/* Legacy Elcon frames deliberately removed. Current gateway BCD codec only. */
void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config,uint8_t *data) {
 if(!BMS_Charger_Encode(config->target_voltage,config->disable_charging ? 0 : config->max_current,data)) {
  memset(data,0,8);BMS_Health_Latch(BMS_ERR_CHARGER);
 }
}

// Each RX FIFO only holds 3 frames, so it is emptied completely on every interrupt
static void drainRxFifo(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[64];     // HAL_FDCAN_GetRxMessage copies DLCtoBytes[DLC] bytes: up to 64, also for classic frames

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxHeader, rxData) != HAL_OK)
        {
            if(hfdcan==&hfdcan2)BMS_IVT_InvalidateTransport();
            BMS_Health_Latch(BMS_ERR_CAN);
            break;
        }
        if(rxHeader.RxFrameType!=FDCAN_DATA_FRAME || rxHeader.FDFormat!=FDCAN_CLASSIC_CAN || rxHeader.DataLength>8)continue;
        // Classic CAN: DLC 9..15 means 8 data bytes (ISO 11898-1); the bytes past them aren't from this frame
        uint32_t length = rxHeader.DataLength > 8 ? 8 : rxHeader.DataLength;

        if (hfdcan == &hfdcan1)
        {
            if (rxHeader.Identifier == CFG_CAN_ID_TS_REQUEST && length == 8 &&
                (rxHeader.IdType == FDCAN_EXTENDED_ID) == (CFG_CAN_TS_REQUEST_EXTENDED != 0))
            {
                BMS_Precharge_HandleRequest(rxData[0] == 1);
            }
            else if (rxHeader.Identifier == CFG_CAN_ID_CONFIG_REQUEST && rxHeader.IdType == FDCAN_EXTENDED_ID &&
                     length == 8)
            {
                BMS_Settings_HandleRequest(rxData, length);
            }
        }
        else if (hfdcan == &hfdcan3)
        {
            if(rxHeader.IdType==FDCAN_EXTENDED_ID)BMS_Charger_HandleFrame(rxHeader.Identifier,rxData,length);
        }
        else if (hfdcan == &hfdcan2)
        {
            if (rxHeader.IdType == FDCAN_STANDARD_ID)
            {
                BMS_IVT_HandleFrame(rxHeader.Identifier, rxData, length);   // Results and responses
            }
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) && hfdcan==&hfdcan2)BMS_IVT_InvalidateTransport();
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        drainRxFifo(hfdcan, FDCAN_RX_FIFO0);
    }
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) && hfdcan==&hfdcan2)BMS_IVT_InvalidateTransport();
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
    {
        drainRxFifo(hfdcan, FDCAN_RX_FIFO1);
    }
}
