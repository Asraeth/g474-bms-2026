#pragma once

/* ===================================================================== */
/*  Cell temperature sensor                                              */
/* ===================================================================== */

// Enepaq module sensor output, datasheet Table 5 (voltage falls as temperature rises).
// VERIFY: the table assumes the sensor is biased as in the Enepaq test circuit.
#define CFG_TEMP_TABLE_C    { -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5,   10,   15,   20,   25,   30,   35,   40, \
                               45,   50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100,  105,  110,  115,  120 }
#define CFG_TEMP_TABLE_V    { 2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, \
                              1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30 }

#define CFG_TEMP_INVALID_C          888.0f  // Returned for a reading outside the table (open/shorted sensor) -> trips over-temp



/* ===================================================================== */
/*  State of charge and energy                                           */
/* ===================================================================== */
/*
 * At start-up, with no load, SoC comes from the open-circuit voltage model in "Battery Tracker.xlsx"
 * (sheet Model, name VToSoc), applied to the mean cell voltage like the sheet's Pack SoC:
 *   SoC % = 100 * sum_k D_k * T_k(t),  t = 2 (v - VMIN) / (VMAX - VMIN) - 1, clamped to [-1, 1]
 * T_k are Chebyshev polynomials (the sheet's COS(k * ACOS(t))). Cells outside SOC_CELL_MIN/MAX are
 * left out of the mean, as in the sheet.
 * After that, remaining energy follows the IVT-S energy counter (Wh, from I and U1).
 */

// Energy of one series cell (one Enepaq module = VTC6 cells in parallel): 10.8 Wh per VTC6 cell.
// VERIFY: 10.8 Wh x the number of cells in parallel (75.6 Wh = 7p).
#define CFG_CELL_ENERGY_WH          75.6f
#define CFG_PACK_ENERGY_WH          (CFG_CELL_ENERGY_WH * CFG_TOTAL_CELLS)

#define CFG_SOC_VMIN_V              2.8f
#define CFG_SOC_VMAX_V              4.09959016393443f
#define CFG_SOC_COEFFS              {  0.423004261244546f,     0.49948859673559f,    0.120958931657348f,   -0.0112915923908232f, \
                                      -0.0135757684176135f,   -0.00150121928350042f, -0.00413216758222435f, -0.0104979259889331f,  \
                                      -0.00635341354977716f,  -0.000801816990438799f, -0.00226824735787767f, -6.36450868561571e-06f, \
                                       0.00450625289432844f,   0.00602363597183047f,  0.00231370791940568f, -0.000704525643415416f, \
                                      -0.00224505625907345f,  -0.00204305060329757f, -0.00158222115306415f, -0.000992284085613443f, \
                                       0.000420170472073563f,  0.000711803275347776f, 0.000678935305282581f }
#define CFG_SOC_CELL_MIN_V          2.8f
#define CFG_SOC_CELL_MAX_V          4.1f

// The start-up estimate waits for no load: TS off, or |pack current| below this
#define CFG_SOC_NO_LOAD_CURRENT_A   5.0f
// Energy counter readings further apart than this aren't trusted (IVT restart reloads an old count)
#define CFG_ENERGY_MAX_GAP_MS       1000


