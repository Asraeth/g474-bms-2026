#pragma once

/* ===================================================================== */
/*  Pack topology                                                        */
/* ===================================================================== */

#define CFG_NUM_SLAVES              BMS_PROFILE_SLAVES      // ADBMS6830B boards in the isoSPI daisy chain
#define CFG_CELLS_PER_SLAVE         8
#define CFG_TEMPS_PER_SLAVE         8

#define CFG_TOTAL_CELLS             (CFG_NUM_SLAVES * CFG_CELLS_PER_SLAVE)
#define TOTAL_IC                    CFG_NUM_SLAVES      // Devices in the isoSPI chain (used by the SPI layer)

// ADBMS6830B cell input (1..16) used for each cell on a slave, bottom cell first.
// Datasheet "Cell Depopulation": unused inputs are shorted from the top, so 8 cells = C1..C8.
// VERIFY against the slave schematic.
#define CFG_SLAVE_CELL_CHANNELS     BMS_PROFILE_CELL_CHANNELS


