/*
 * sim_can.c
 *
 * FDCAN1/2/3 model: acceptance filters, 3-deep RX FIFOs and TX FIFO, bit timing worked out from
 * the init values (with the clock divider shared from FDCAN1 like the real chip), ACK and bus-off.
 * Each bus carries one frame at a time; when it goes idle, the pending frame with the lowest
 * identifier wins arbitration: the head of the MCU's TX FIFO, frames from the other nodes, or a
 * babbling node's flood.
 */

#include "sim.h"
#include "fdcan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FDCAN_KERNEL_CLOCK_HZ   128000000ULL
#define FIFO_DEPTH              3
#define MAX_FILTERS             8

// Bitrate the other nodes on each bus use
static const uint32_t expected_bitrate[4] = { 0, CFG_CAN1_BITRATE, CFG_CAN2_BITRATE, CFG_CAN3_BITRATE };

typedef struct
{
    FDCAN_TxHeaderTypeDef hdr;
    uint8_t               data[8];
} tx_entry;

// An Rx FIFO element of the message RAM: room for 64 data bytes (CAN FD). A classic frame fills the first 8;
// the rest holds whatever was there before.
typedef struct
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t               data[64];
} rx_entry;

// Classic CAN: DLC 9..15 means 8 data bytes (ISO 11898-1)
static uint8_t dlc_bytes(uint8_t dlc) { return dlc > 8 ? 8 : dlc; }

typedef struct
{
    int                   num;
    FDCAN_HandleTypeDef*  h;
    FDCAN_GlobalTypeDef*  regs;
    IRQn_Type             irq;
    sim_irq_t             sim_irq;
    bool                  initialised;
    bool                  started;

    FDCAN_FilterTypeDef   std_filter[MAX_FILTERS];
    bool                  std_set[MAX_FILTERS];
    FDCAN_FilterTypeDef   ext_filter[MAX_FILTERS];
    bool                  ext_set[MAX_FILTERS];
    uint32_t              non_matching_std;
    uint32_t              non_matching_ext;
    uint32_t              notifications;

    tx_entry              txq[FIFO_DEPTH];
    int                   tx_count;
    bool                  tx_busy;          // The MCU's frame is on the wire

    sim_timer             wire_timer;       // End of the frame on the wire
    bool                  wire_busy;
    sim_can_frame         wire;             // A node's frame on the wire (when !tx_busy)
    bool                  wire_flood;       // ... from the babbling node

    bool                  rtr_reject_std;   // Global filter: remote frames rejected
    bool                  rtr_reject_ext;

    rx_entry              rxq[2][FIFO_DEPTH];   // RX FIFO 0 and 1
    int                   rx_count[2];
    int                   rx_lost;

    bool                  pend_rx[2];
    bool                  pend_tx_empty;
    bool                  ack;
    bool                  ack_set;          // By a test, possibly before the MCU initialised the FDCAN
    sim_can_listener      listener;
    sim_timer             housekeeping;
    uint64_t              busy_us;          // Time the bus has carried frames, for the bus load

    // Babbling node: always has a frame pending until flood_until
    uint32_t              flood_id;
    bool                  flood_ext;
    uint8_t               flood_dlc;
    uint64_t              flood_until;
    int                   flood_sent;
} sim_bus;

static sim_bus buses[4];

#define MAX_LOG 1500000
static sim_can_frame* can_log = NULL;
static int            can_log_count = 0;

bool sim_nvic_enabled(IRQn_Type irq);


/* ---- Helpers ---------------------------------------------------------------------------- */

static sim_bus* bus_for(FDCAN_HandleTypeDef* h)
{
    if (h == &hfdcan1) return &buses[1];
    if (h == &hfdcan2) return &buses[2];
    if (h == &hfdcan3) return &buses[3];
    fprintf(stderr, "sim: unknown FDCAN handle\n");
    exit(2);
}

uint32_t sim_can_bitrate(int bus)
{
    sim_bus* b = &buses[bus];
    if (!b->initialised) return 0;

    // The kernel clock divider is shared and only taken from FDCAN1's init
    uint32_t ckdiv = hfdcan1.Init.ClockDivider;
    uint64_t divider = ckdiv == 0 ? 1 : 2ULL * ckdiv;
    uint64_t tq_per_bit = 1ULL + b->h->Init.NominalTimeSeg1 + b->h->Init.NominalTimeSeg2;
    return (uint32_t)(FDCAN_KERNEL_CLOCK_HZ / divider / b->h->Init.NominalPrescaler / tq_per_bit);
}

static bool bitrate_ok(sim_bus* b)
{
    uint32_t actual = sim_can_bitrate(b->num);
    uint32_t expected = expected_bitrate[b->num];
    uint32_t diff = actual > expected ? actual - expected : expected - actual;
    return diff * 200 <= expected;     // within 0.5 %
}

static uint64_t frame_time_us(sim_bus* b, bool ext, uint8_t dlc)
{
    uint32_t bits = (ext ? 67U : 47U) + 8U * dlc_bytes(dlc);
    bits += bits / 10;                  // Typical bit stuffing
    uint32_t rate = sim_can_bitrate(b->num);
    if (rate == 0) rate = expected_bitrate[b->num];
    return (uint64_t)bits * 1000000ULL / rate + 1;
}

static void log_frame(int bus, uint32_t id, bool ext, const uint8_t* data, uint8_t dlc, bool from_mcu)
{
    if (!can_log) can_log = calloc(MAX_LOG, sizeof(sim_can_frame));
    if (can_log_count >= MAX_LOG) return;
    sim_can_frame* f = &can_log[can_log_count++];
    f->t_us = sim_now_us();
    f->bus = bus;
    f->id = id;
    f->ext = ext;
    f->dlc = dlc;
    memcpy(f->data, data, dlc_bytes(dlc));
    f->from_mcu = from_mcu;
}

static bool in_init(sim_bus* b)
{
    return (b->regs->CCCR & FDCAN_CCCR_INIT) != 0;
}


/* ---- Interrupts --------------------------------------------------------------------------- */

static void bus_isr(sim_bus* b)
{
    if (b->pend_rx[0] && (b->notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        b->pend_rx[0] = false;
        HAL_FDCAN_RxFifo0Callback(b->h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
    if (b->pend_rx[1] && (b->notifications & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        b->pend_rx[1] = false;
        HAL_FDCAN_RxFifo1Callback(b->h, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    }
    if (b->pend_tx_empty && (b->notifications & FDCAN_IT_TX_FIFO_EMPTY))
    {
        b->pend_tx_empty = false;
        HAL_FDCAN_TxFifoEmptyCallback(b->h);
    }
}

static void fdcan1_isr(void) { bus_isr(&buses[1]); }
static void fdcan2_isr(void) { bus_isr(&buses[2]); }
static void fdcan3_isr(void) { bus_isr(&buses[3]); }

static void raise_irq(sim_bus* b)
{
    if (sim_nvic_enabled(b->irq))
    {
        sim_irq_pend(b->sim_irq);
    }
}

bool sim_can_irq_enabled(int bus)
{
    return sim_nvic_enabled(buses[bus].irq);
}


/* ---- The wire: arbitration and transmission ----------------------------------------------- */

#define NODE_QUEUE 256
static sim_can_frame node_queue[4][NODE_QUEUE];     // Frames other nodes want to send, in order
static int           node_count[4];

static void deliver(sim_bus* b, const sim_can_frame* f);

// Arbitration order: base identifier first; on a tie the standard frame wins (dominant RTR where the
// extended frame has SRR), then the extended bits
static uint32_t arbitration_key(uint32_t id, bool ext)
{
    uint32_t base = ext ? (id >> 18) & 0x7FFU : id & 0x7FFU;
    return (base << 19) | (ext ? (1U << 18) | (id & 0x3FFFFU) : 0U);
}

static bool mcu_can_send(sim_bus* b)
{
    // Without an acknowledgement the frame is retransmitted forever and never completes (the error
    // frames are not modelled: the other nodes' frames still get through)
    return b->tx_count > 0 && b->started && !in_init(b) && b->ack && bitrate_ok(b);
}

static void bus_arbitrate(sim_bus* b)
{
    if (b->wire_busy) return;

    int bus = b->num;
    uint32_t best_key = UINT32_MAX;
    int best = -3;                                  // -1: MCU, -2: flood, >= 0: node queue index
    if (mcu_can_send(b))
    {
        best_key = arbitration_key(b->txq[0].hdr.Identifier, b->txq[0].hdr.IdType == FDCAN_EXTENDED_ID);
        best = -1;
    }
    if (b->flood_until > sim_now_us() && arbitration_key(b->flood_id, b->flood_ext) < best_key)
    {
        best_key = arbitration_key(b->flood_id, b->flood_ext);
        best = -2;
    }
    for (int i = 0; i < node_count[bus]; i++)
    {
        uint32_t key = arbitration_key(node_queue[bus][i].id, node_queue[bus][i].ext);
        if (key < best_key)
        {
            best_key = key;
            best = i;
        }
    }
    if (best == -3) return;

    bool ext;
    uint8_t dlc;
    b->wire_flood = false;
    if (best == -1)
    {
        b->tx_busy = true;
        ext = b->txq[0].hdr.IdType == FDCAN_EXTENDED_ID;
        dlc = (uint8_t)b->txq[0].hdr.DataLength;
    }
    else if (best == -2)
    {
        memset(&b->wire, 0, sizeof(b->wire));
        b->wire.bus = bus;
        b->wire.id = b->flood_id;
        b->wire.ext = b->flood_ext;
        b->wire.dlc = b->flood_dlc;
        memset(b->wire.data, 0x55, b->flood_dlc);
        b->wire_flood = true;
        ext = b->flood_ext;
        dlc = b->flood_dlc;
    }
    else
    {
        b->wire = node_queue[bus][best];
        memmove(&node_queue[bus][best], &node_queue[bus][best + 1], sizeof(sim_can_frame) * (node_count[bus] - best - 1));
        node_count[bus]--;
        ext = b->wire.ext;
        dlc = b->wire.rtr ? 0 : b->wire.dlc;
    }
    b->wire_busy = true;
    sim_timer_start(&b->wire_timer, sim_now_us() + frame_time_us(b, ext, dlc));
}

static void wire_done(sim_timer* t)
{
    sim_bus* b = (sim_bus*)t->ctx;
    b->wire_busy = false;

    if (b->tx_busy)
    {
        tx_entry e = b->txq[0];
        b->busy_us += frame_time_us(b, e.hdr.IdType == FDCAN_EXTENDED_ID, (uint8_t)e.hdr.DataLength);
        memmove(&b->txq[0], &b->txq[1], sizeof(tx_entry) * (FIFO_DEPTH - 1));
        b->tx_count--;
        b->tx_busy = false;

        log_frame(b->num, e.hdr.Identifier, e.hdr.IdType == FDCAN_EXTENDED_ID, e.data, (uint8_t)e.hdr.DataLength, true);
        /* Wire observers must see the actual frame even after bounded log storage fills. */
        sim_can_frame frame={.t_us=sim_now_us(),.bus=b->num,.id=e.hdr.Identifier,.ext=e.hdr.IdType==FDCAN_EXTENDED_ID,.dlc=(uint8_t)e.hdr.DataLength,.from_mcu=true};
        memcpy(frame.data,e.data,8);
        if (b->listener) b->listener(&frame);

        if (b->tx_count == 0)
        {
            b->pend_tx_empty = true;
            raise_irq(b);
        }
    }
    else
    {
        sim_can_frame f = b->wire;
        b->busy_us += frame_time_us(b, f.ext, f.rtr ? 0 : f.dlc);
        if (b->wire_flood) b->flood_sent++;
        deliver(b, &f);
    }
    bus_arbitrate(b);
}

// The wire's timer works before the MCU has initialised the FDCAN too (other nodes talk regardless)
static void wire_setup(sim_bus* b, int num)
{
    if (b->wire_timer.fire) return;
    b->num = num;
    b->wire_timer.fire = wire_done;
    b->wire_timer.ctx = b;
}

static void housekeeping(sim_timer* t)
{
    sim_bus* b = (sim_bus*)t->ctx;
    // Software clearing INIT after bus-off restarts the controller
    if ((b->regs->PSR & FDCAN_PSR_BO) && !in_init(b))
    {
        b->regs->PSR &= ~FDCAN_PSR_BO;
    }
    bus_arbitrate(b);
    sim_timer_start(t, sim_now_us() + 1000);
}


/* ---- Receive -------------------------------------------------------------------------------- */

static bool filter_match(const FDCAN_FilterTypeDef* f, uint32_t id)
{
    switch (f->FilterType)
    {
    case FDCAN_FILTER_RANGE: return id >= f->FilterID1 && id <= f->FilterID2;
    case FDCAN_FILTER_DUAL:  return id == f->FilterID1 || id == f->FilterID2;
    case FDCAN_FILTER_MASK:  return (id & f->FilterID2) == (f->FilterID1 & f->FilterID2);
    default:                 return false;
    }
}

// RX FIFO the frame goes to, or -1 if rejected
static int accepted(sim_bus* b, uint32_t id, bool ext)
{
    uint32_t count = ext ? b->h->Init.ExtFiltersNbr : b->h->Init.StdFiltersNbr;
    FDCAN_FilterTypeDef* filters = ext ? b->ext_filter : b->std_filter;
    bool* set = ext ? b->ext_set : b->std_set;

    for (uint32_t i = 0; i < count && i < MAX_FILTERS; i++)
    {
        if (!set[i] || !filter_match(&filters[i], id)) continue;
        if (filters[i].FilterConfig == FDCAN_FILTER_TO_RXFIFO0) return 0;
        if (filters[i].FilterConfig == FDCAN_FILTER_TO_RXFIFO1) return 1;
        return -1;
    }
    uint32_t non_matching = ext ? b->non_matching_ext : b->non_matching_std;
    if (non_matching == FDCAN_ACCEPT_IN_RX_FIFO0) return 0;
    if (non_matching == FDCAN_ACCEPT_IN_RX_FIFO1) return 1;
    return -1;
}

static void deliver(sim_bus* b, const sim_can_frame* f)
{
    log_frame(b->num, f->id, f->ext, f->data, f->dlc, false);
    can_log[can_log_count - 1].rtr = f->rtr;

    if (!b->started || in_init(b) || !bitrate_ok(b)) return;
    if (f->rtr && (f->ext ? b->rtr_reject_ext : b->rtr_reject_std)) return;
    int fifo = accepted(b, f->id, f->ext);
    if (fifo < 0) return;

    if (b->rx_count[fifo] >= FIFO_DEPTH)
    {
        b->rx_lost++;
        return;
    }
    rx_entry* e = &b->rxq[fifo][b->rx_count[fifo]++];
    memset(e, 0, sizeof(*e));
    memset(e->data, 0xA5, sizeof(e->data));     // Left over in the message RAM
    e->hdr.Identifier = f->id;
    e->hdr.IdType = f->ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    e->hdr.RxFrameType = f->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    e->hdr.DataLength = f->dlc;
    e->hdr.FDFormat = FDCAN_CLASSIC_CAN;
    memcpy(e->data, f->data, dlc_bytes(f->dlc));

    b->pend_rx[fifo] = true;
    raise_irq(b);
}

static void node_send(int bus, uint32_t id, bool ext, bool rtr, const uint8_t* data, uint8_t dlc)
{
    sim_bus* b = &buses[bus];
    wire_setup(b, bus);
    if (node_count[bus] >= NODE_QUEUE)
    {
        b->rx_lost++;
        return;
    }
    if (dlc > 15) dlc = 15;     // DLC code: 9..15 are 8-byte frames
    sim_can_frame* f = &node_queue[bus][node_count[bus]++];
    memset(f, 0, sizeof(*f));
    f->bus = bus;
    f->id = id;
    f->ext = ext;
    f->rtr = rtr;
    f->dlc = dlc;
    if (data && !rtr) memcpy(f->data, data, dlc_bytes(dlc));
    bus_arbitrate(b);
}

void sim_can_node_send(int bus, uint32_t id, bool ext, const uint8_t* data, uint8_t dlc)
{
    node_send(bus, id, ext, false, data, dlc);
}

void sim_can_node_send_remote(int bus, uint32_t id, bool ext, uint8_t dlc)
{
    node_send(bus, id, ext, true, NULL, dlc);
}

void sim_can_flood(int bus, uint32_t id, bool ext, uint8_t dlc, uint64_t until_us)
{
    sim_bus* b = &buses[bus];
    wire_setup(b, bus);
    b->flood_id = id;
    b->flood_ext = ext;
    b->flood_dlc = dlc > 8 ? 8 : dlc;
    b->flood_until = until_us;
    bus_arbitrate(b);
}

int sim_can_flood_sent(int bus)
{
    return buses[bus].flood_sent;
}


/* ---- Test controls -------------------------------------------------------------------------- */

void sim_can_set_listener(int bus, sim_can_listener listener)
{
    buses[bus].listener = listener;
}

void sim_can_set_ack(int bus, bool ack)
{
    buses[bus].ack = ack;
    buses[bus].ack_set = true;
    if (buses[bus].wire_timer.fire) bus_arbitrate(&buses[bus]);
}

void sim_can_force_bus_off(int bus)
{
    sim_bus* b = &buses[bus];
    b->regs->PSR |= FDCAN_PSR_BO;
    b->regs->CCCR |= FDCAN_CCCR_INIT;
    if (b->tx_busy)
    {
        sim_timer_stop(&b->wire_timer);         // Its frame is cut off
        b->tx_busy = false;
        b->wire_busy = false;
        bus_arbitrate(b);
    }
}

int sim_can_rx_lost(int bus)
{
    return buses[bus].rx_lost;
}

uint64_t sim_can_busy_us(int bus)
{
    return buses[bus].busy_us;
}

int sim_can_log_count(void)
{
    return can_log_count;
}

const sim_can_frame* sim_can_log(int index)
{
    return &can_log[index];
}

void sim_can_log_write_csv(const char* path)
{
    FILE* f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "time_s,bus,id,ext,dlc,data,from_mcu\n");
    for (int i = 0; i < can_log_count; i++)
    {
        const sim_can_frame* c = &can_log[i];
        fprintf(f, "%.6f,%d,0x%X,%d,%d,", c->t_us / 1e6, c->bus, c->id, c->ext, c->dlc);
        for (int j = 0; j < dlc_bytes(c->dlc); j++) fprintf(f, "%02X", c->data[j]);
        fprintf(f, ",%d\n", c->from_mcu);
    }
    fclose(f);
}


/* ---- HAL ------------------------------------------------------------------------------------ */

static void bus_setup(int num, FDCAN_HandleTypeDef* h)
{
    sim_bus* b = &buses[num];
    if (b->initialised) return;

    b->num = num;
    b->h = h;
    static FDCAN_GlobalTypeDef* const regs[4] = { NULL, &sim_FDCAN1, &sim_FDCAN2, &sim_FDCAN3 };
    static const IRQn_Type irqs[4] = { 0, FDCAN1_IT0_IRQn, FDCAN2_IT0_IRQn, FDCAN3_IT0_IRQn };
    static const sim_irq_t sim_irqs[4] = { 0, SIM_IRQ_FDCAN1, SIM_IRQ_FDCAN2, SIM_IRQ_FDCAN3 };
    static const sim_irq_handler isrs[4] = { NULL, fdcan1_isr, fdcan2_isr, fdcan3_isr };
    b->regs = regs[num];
    b->irq = irqs[num];
    b->sim_irq = sim_irqs[num];
    if (!b->ack_set) b->ack = true;
    b->non_matching_std = FDCAN_ACCEPT_IN_RX_FIFO0;     // Reset value of the global filter
    b->non_matching_ext = FDCAN_ACCEPT_IN_RX_FIFO0;
    wire_setup(b, num);
    b->housekeeping.fire = housekeeping;
    b->housekeeping.ctx = b;
    b->initialised = true;
    sim_irq_set_handler(b->sim_irq, isrs[num]);
    sim_timer_start(&b->housekeeping, sim_now_us() + 1000);
}

HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef* h)
{
    bus_setup(h == &hfdcan1 ? 1 : (h == &hfdcan2 ? 2 : 3), h);
    HAL_FDCAN_MspInit(h);
    bus_for(h)->regs->CCCR |= FDCAN_CCCR_INIT;
    h->State = HAL_FDCAN_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* h, const FDCAN_FilterTypeDef* f)
{
    sim_bus* b = bus_for(h);
    if (f->FilterIndex >= MAX_FILTERS) return HAL_ERROR;
    if (f->IdType == FDCAN_STANDARD_ID)
    {
        b->std_filter[f->FilterIndex] = *f;
        b->std_set[f->FilterIndex] = true;
    }
    else
    {
        b->ext_filter[f->FilterIndex] = *f;
        b->ext_set[f->FilterIndex] = true;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* h, uint32_t std, uint32_t ext, uint32_t rtr_std, uint32_t rtr_ext)
{
    sim_bus* b = bus_for(h);
    if (h->State != HAL_FDCAN_STATE_READY) return HAL_ERROR;   // Only allowed before HAL_FDCAN_Start
    b->non_matching_std = std;
    b->non_matching_ext = ext;
    b->rtr_reject_std = rtr_std == FDCAN_REJECT_REMOTE;
    b->rtr_reject_ext = rtr_ext == FDCAN_REJECT_REMOTE;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* h)
{
    sim_bus* b = bus_for(h);
    if (h->State != HAL_FDCAN_STATE_READY) return HAL_ERROR;
    b->started = true;
    b->regs->CCCR &= ~FDCAN_CCCR_INIT;
    h->State = HAL_FDCAN_STATE_BUSY;
    if (!bitrate_ok(b))
    {
        sim_note("CAN%d runs at %lu bit/s, the other nodes at %lu bit/s",
                 b->num, (unsigned long)sim_can_bitrate(b->num), (unsigned long)expected_bitrate[b->num]);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* h, uint32_t its, uint32_t buffers)
{
    (void)buffers;
    bus_for(h)->notifications |= its;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef* h)
{
    return (uint32_t)(FIFO_DEPTH - bus_for((FDCAN_HandleTypeDef*)h)->tx_count);
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* h, const FDCAN_TxHeaderTypeDef* hdr, const uint8_t* data)
{
    sim_bus* b = bus_for(h);
    if (!b->started) return HAL_ERROR;
    if (b->tx_count >= FIFO_DEPTH)
    {
        h->ErrorCode |= HAL_FDCAN_ERROR_FIFO_FULL;
        return HAL_ERROR;
    }
    if (hdr->DataLength > 8) return HAL_ERROR;

    tx_entry* e = &b->txq[b->tx_count++];
    e->hdr = *hdr;
    memset(e->data, 0, sizeof(e->data));
    memcpy(e->data, data, hdr->DataLength);
    bus_arbitrate(b);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef* h, uint32_t buffers)
{
    (void)buffers;
    sim_bus* b = bus_for(h);
    // A frame already on the wire finishes; everything queued behind it is dropped
    b->tx_count = b->tx_busy ? 1 : 0;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef* h, uint32_t fifo)
{
    int index = (fifo == FDCAN_RX_FIFO0) ? 0 : 1;
    return (uint32_t)bus_for((FDCAN_HandleTypeDef*)h)->rx_count[index];
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* h, uint32_t fifo, FDCAN_RxHeaderTypeDef* hdr, uint8_t* data)
{
    sim_bus* b = bus_for(h);
    int index = (fifo == FDCAN_RX_FIFO0) ? 0 : 1;
    if (b->rx_count[index] == 0) return HAL_ERROR;
    // As the HAL does: DLCtoBytes[DLC] bytes, so up to 64 for a classic frame with DLC 9..15
    static const uint8_t dlc_to_bytes[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    *hdr = b->rxq[index][0].hdr;
    memcpy(data, b->rxq[index][0].data, dlc_to_bytes[hdr->DataLength & 0xF]);
    memmove(&b->rxq[index][0], &b->rxq[index][1], sizeof(rx_entry) * (FIFO_DEPTH - 1));
    b->rx_count[index]--;
    return HAL_OK;
}
