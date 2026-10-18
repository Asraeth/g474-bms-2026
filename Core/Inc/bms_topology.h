#ifndef BMS_TOPOLOGY_H
#define BMS_TOPOLOGY_H

#define TOTAL_CELL 8U
#define TOTAL_TEMP 8U
#define TOTAL_AD68 14U
#define TOTAL_IC TOTAL_AD68
#define TOTAL_PACK_CELLS (TOTAL_AD68 * TOTAL_CELL)
#define AD68_CELL_GROUP_COUNT ((TOTAL_CELL + 2U) / 3U)
#define AD68_HARDWARE_CELL_CHANNELS 16U /* Register capacity, not populated cells. */
#define BMS_POPULATED_CELL_MASK ((1U << TOTAL_CELL) - 1U)

#if TOTAL_CELL != 8 || TOTAL_AD68 != 14 || TOTAL_PACK_CELLS != 112
#error "This working fork requires the confirmed 14 x 8 topology"
#endif
#endif
