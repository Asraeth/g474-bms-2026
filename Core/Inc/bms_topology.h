#pragma once
#include "profiles/bms_board_profile.h"
#include "config/bms_topology_config.h"
#define TOTAL_CELL CFG_CELLS_PER_SLAVE
#define TOTAL_TEMP CFG_TEMPS_PER_SLAVE
#define TOTAL_AD68 CFG_NUM_SLAVES
#define TOTAL_PACK_CELLS CFG_TOTAL_CELLS
#define AD68_CELL_GROUP_COUNT ((TOTAL_CELL+2U)/3U)
#define AD68_HARDWARE_CELL_CHANNELS 16U
#define BMS_POPULATED_CELL_MASK ((1U<<TOTAL_CELL)-1U)
