<!-- Generated from doc/bringup_test_procedure.md.in by tools/generate.py with the values in Core/Inc/bms_config.h. Edit the template, not this file. -->

> Historical detailed reference. Use [the staged BENCH guide](../docs/TOMORROW-BENCH-BRINGUP.md) for physical testing. This template uses the 14-board digital reference configuration; its Celsius/actuation exercises require confirmed profiles and are not instructions to enable outputs on BENCH hardware.

# BMS bring-up test procedure

Hardware checks for the parts the PC test suite can't cover: real isoSPI signalling, ADC readings,
the open-wire threshold, CAN wiring, the IVT-S and the fault output circuit. Work through the stages
in order and **don't connect high voltage until stages 1-4 have passed**.

Every value below comes from `Core/Inc/bms_config.h` at the commit you flashed. Record results in
the table at the end. Note the git commit you flashed (`git log -1 --oneline`).

## Before you start

- [ ] `test/run_tests.sh` passes (all tests + DBC check).
- [ ] `tools/build_arm.sh debug` and `release` build with no warnings, or the CubeIDE build is clean.
- [ ] Every `VERIFY` item in `Core/Inc/bms_config.h` has been checked against the schematics:
      cell channel map, slave GPIO map and LED polarity, temperature sensor bias, IVT current
      direction and U1 wiring, charge current limit, balancing heat budget, pin assignments.
- [ ] The TSAC ready relay pin (PC8), precharge relay pin (PC6) and
      bypass pin (PC9) have pull-downs on the PCB, and the fault output (PC1, open
      drain) has a pull-up to the fault state. Check with the MCU **unpowered**: the latch must see
      a fault and the relays must be off.

**Equipment:** USB-CAN adapter(s) for 1 Mbit/s (CAN1), 500 kbit/s (CAN2) and 500 kbit/s (CAN3) with
SavvyCAN or similar, loaded with `can/bms_can1.dbc`, `can/ivt_can2.dbc` and `can/charger_can3.dbc`. Terminal at 1 Mbaud on the ST-LINK
virtual COM port. Bench PSU, DMM, oscilloscope (2 channels), resistor ladder or cell simulator for
8 cells, adjustable voltage source (1.2-2.5 V) to stand in for a temperature sensor.
Every change of the fault flags or output shows on the UART with the tick it happened on, e.g.
`Fault 11029 ms: 0x42 -> 0x62 +IVT, output ON`: use those ticks for the reaction times below.

## Bench setup: Nucleo + ADBMS6822 eval, EVAL-ADBMS6830BMSW, IVT-S, USB-CAN adapter

Stages 1, 2 and 4 run on the bench before the car's slave boards exist. Copy
`Core/Inc/bms_config_local.h.example` to `Core/Inc/bms_config_local.h` (one eval board and its
30 Ω balancing path; set `CFG_NUM_SLAVES` there for more boards) and rebuild. That file is
git-ignored. While it exists the build warns, the UART prints `BENCH BUILD` at start-up and
`BMS_Status Local_Config_Overrides` = 1. **Delete it before building for the car.**

**CAN transceivers.** Neither the Nucleo nor the ADBMS6822 eval has one. Use one 3.3 V CAN transceiver
board (SN65HVD230 type) per bus you test. On the bench, PA8 (CAN3 RX) is also wired to the ADBMS6822
eval's INT1 line; that is a 20 µA current-limited output, so the transceiver simply overrides it. Boards built around 5 V-only parts (TJA1050, MCP2551) need more than 3.3 V
on TXD. Each bus needs 120 Ω at both ends: 60 Ω between CANH and CANL with everything unpowered.

| Bus | MCU TX → transceiver TXD | MCU RX ← transceiver RXD | Bit rate | Other end |
|---|---|---|---|---|
| CAN1 | PA12 (CN10-12) | PA11 (CN10-14) | 1 Mbit/s | USB-CAN adapter |
| CAN2 | PB13 (CN10-30) | PB12 (CN10-16) | 500 kbit/s | IVT-S (120 Ω at the IVT) |
| CAN3 | PA15 (CN7-17) | PA8 (CN10-23) | 500 kbit/s | Charger, or the adapter |

**Nucleo I/O** (Morpho pins from the MB1367 schematic):

| Signal | Pin | Bench connection |
|---|---|---|
| Fault output (open drain) | PC1 (CN7-36) | 10 kΩ pull-up to 3V3, scope here: high = fault |
| TSAC ready relay | PC8 (CN10-2) | LED + 1 kΩ to GND (lit = on) |
| Precharge relay | PC6 (CN10-4) | LED + 1 kΩ to GND (lit = on) |
| Bypass (AIR) | PC9 (CN10-1) | LED + 1 kΩ to GND (lit = on) |
| SDC sense | PC2 (CN7-35) | Jumper to 3V3 = SDC closed. The input has no pull resistor: don't leave it open |
| Charger button | PC0 (CN7-38) | Open, or a button to GND |
| Transceiver supply | 3V3 CN7-16, GND CN7-20 | Both transceiver boards |

**EVAL-ADBMS6830BMSW.** ADI's 16-cell board, which differs from the car's slave:

- isoSPI: ADBMS6822 eval port A to the eval board's J3 (isoSPI A). More boards: J4 (isoSPI B) to the
  next board's J3. The eval board's isoSPI is capacitor-coupled; that works against the 6822 eval's
  transformer.
- Cells: 8 cells on J1 CELL0 (bottom) to CELL8. Tie CELL9-CELL16 to CELL8
  (unused inputs shorted from the top, datasheet "Cell Depopulation" for fully populated boards); V+
  comes from CELL16 through R55 (0 Ω) and R63 (1 kΩ). A resistor ladder on a ~30 V supply is fine for
  measurements: the ladder itself must stay in one piece (see 2.6). For balancing use real cells or a ladder
  of a few ohms: a balancing cell draws ~123 mA.
- Temperatures: no sensors on the board. Each GPIO reaches header J5 (GPIOn on pin n, V- on the
  opposite row) through 1 kΩ. A resistor from the J5 pin to V- stands in for a sensor. With only some
  inputs populated, list just those in `bms_config_local.h` (`CFG_TEMPS_PER_SLAVE`, `CFG_SLAVE_TEMP_GPIOS`),
  or the open ones keep the temperature fault on:

| J5 pins | Bias on the eval board | 25 °C | Over-temperature test | Under-temperature test |
|---|---|---|---|---|
| GPIO1-3, 6-8 | 10 kΩ pull-up to VREF2 (3 V) | 16 kΩ (26 °C) | 9.1 kΩ (75 °C) | 39 kΩ (-27 °C) |
| GPIO4-5 | EEPROM I²C: 10 kΩ pull-up to VREG (4.5-5.5 V) | 5.1 kΩ (7 to 38 °C over the VREG range) | use another GPIO | use another GPIO |

- Balancing goes through 15 Ω in both the S+ and S- legs: about V / 30 Ω per cell (the bench
  profile accounts for it).
- No LEDs: GPIO9 and GPIO10 only have pull-ups. An LED switched on pulls its J5 pin to ~0 V.

**IVT-S** (12 V supply for the -12/24 variant):

- CAN2 needs 120 Ω at the IVT end; the CAN2 variant has no termination of its own.
- Wire it like Figure 11 of the IVT-S datasheet with the cell supply as the battery: shunt in the
  supply's negative lead, U1 on its positive. U1 then matches the sum of cells, as the plausibility
  check expects (within 20 V). With one eval board U1 must be within 20.0-33.6 V.
- U2 (vehicle side of the AIRs): a second supply, starting at 0 V. Raising it by hand stands in for
  the precharge circuit.

**Precharge on the bench:** with the fault output released and the SDC input closed, send
ID 0xB300 (extended), data `01` from the USB-CAN adapter. The precharge output (PC6) turns on. Raise U2
past 95 % of U1: the bypass output (PC9) turns on and precharge turns off 50 ms
later. Leave U2 low instead: after 5 s both stay off (`Failed`) until the request goes to
`00` and back to `01`.

---

Useful voltages for the Enepaq temperature sensor (`CFG_TEMP_TABLE_*`):

| Temperature | 120 °C | 60 °C (max) | 45 °C (max charging) | 25 °C | 0 °C (min charging) | -20 °C (min) | -40 °C |
|---|---|---|---|---|---|---|---|
| Sensor voltage | 1.30 V | 1.51 V | 1.63 V | 1.86 V | 2.17 V | 2.35 V | 2.44 V |

Outside 1.30-2.44 V the BMS reads 888 °C (sensor fault) and trips.

**Measuring reaction time:** scope channel 1 on the stimulus (PSU step, the sense wire you pull,
the sensor voltage source), channel 2 on PC1 (fault output). Trigger on channel 1. The time to
PC1 going to the fault level must be **under 500 ms**; the configured worst case is
480 ms (fault deadline + safety tick).

---

## Stage 1: Controller only (Nucleo + ADBMS6822, no slaves, no IVT, no HV)

| # | Test | Expected |
|---|---|---|
| 1.1 | Power up, watch the UART | `PROGRAM START`, `SOFT RESET`, `BMS INIT`, then `PEC ERROR` / `SPI ERROR` messages (no slaves), and once `CAN TX Buffer Overwritten (CAN1 burst not getting out ...)` while nothing on CAN1 acknowledges. No hang, no reset loop. |
| 1.2 | Fault output and relay | PC1 at the fault level the whole time, PC8 low (relay off). |
| 1.3 | Reset the Nucleo (black button) while watching PC1 on the scope | PC1 stays at the fault level through the reset (external pull-up). PC8 stays low. |
| 1.4 | CAN1 (1 Mbit/s) | `BMS_Pack`, `BMS_Status`, `BMS_Summary` and `BMS_Energy` every 100 ms. `BMS_Fast_1` / `BMS_Fast_2` every 4 ms, all values "not available" (raw INT32_MIN / -32768 / 65535). No slave or cell frames (the chain never initialises). `BMS_Status`: `Fault_Startup` = 1, `Fault_IVT` = 1, `Fault_Output_Active` = 1, `TSAC_Ready` = 0. |
| 1.5 | CAN2 (500 kbit/s, adapter acknowledging) | Every 500 ms: 10 frames on 0x411 (IVT configuration, first 4 bytes shown, the rest 0): `34 00 01 00` (stop), `20 02 00 04` (I every 4 ms), `21 02 00 04` (U1 every 4 ms), `22 02 00 04` (U2 every 4 ms), `23 00 00 3C` (U3 off), `24 00 00 64` (T off), `25 00 00 1E` (W off), `26 00 00 1E` (As off), `27 02 00 64` (Wh every 100 ms), `34 01 01 00` (run). |
| 1.6 | CAN2 with nothing acknowledging | UART: `IVT not acknowledging on CAN2, configuration skipped`. No lock-up. |
| 1.7 | Short CAN1 H to L for ~1 s, then remove | Bus-off, then frames resume within ~50 ms of removing the short. |
| 1.8 | Watchdog: build with `-DBMS_TEST_WATCHDOG` (Debug build, add the define), flash, run without the debugger | After 10 s: `BMS_TEST_WATCHDOG: hanging the main loop`, then within ~200 ms `PROGRAM START` again. PC1 holds the fault level throughout. **Flash the normal build afterwards.** |
| 1.9 | CAN3 (500 kbit/s, adapter acknowledging) | `BMS_Charger_Command` every 100 ms with `Charger_Disable` = 1. Nothing from the charger on CAN1. |

## Stage 2: One slave board on a cell simulator (no HV)

Use the bench profile with one slave (see Bench setup): an EVAL-ADBMS6830BMSW wired as described
there, or one of the car's slave boards. Feed the cell inputs from a resistor ladder / cell simulator at
~3.7 V per cell. Put a 1.86 V source (25 °C), a real sensor or, on the eval board, the
resistors from the Bench setup table on each TEMP input. Set `CFG_DEBUG_PRINT_CELLS` and
`CFG_DEBUG_PRINT_TEMPS` to 1 in `bms_config_local.h` for this stage (cell voltages and temperatures on the UART every cycle).

| # | Test | Expected |
|---|---|---|
| 2.1 | Comms | No PEC errors after `BMS INIT`. `LoopTime` on the UART well under 70 ms. |
| 2.2 | Cell voltages vs DMM at the connector | Within ±10 mV per cell. Cells in the right order (change one cell and check which one moves). |
| 2.3 | Slave voltage, die temperature | Slave voltage (VPV, measured at the V+ pin) a little below the sum of cells: the slave's supply current flows through the resistor in front of V+ (1 kΩ on the eval board: 0.5-1 V low, ~0.7 V measured). The sum of cells is what the pack checks use. Die temperature near ambient. |
| 2.4 | Temperatures | Each sensor input reads its applied voltage per the table, in TEMP1-8 order. |
| 2.5 | Open-wire calibration | UART `intact ratio min` per slave. Expect ~0.85-0.95. Set `CFG_OPEN_WIRE_MIN_RATIO` comfortably below the lowest value you see (now 0.75). |
| 2.6 | Open wire | With nothing balancing, disconnect one sense wire at the connector: `BMS_SegNN_CellNN_OpenWire` = 1 (UART `OW` mask), fault output **< 500 ms** (scope). Reconnect: fault output releases ~1 s later. Repeat for the bottom wire, a middle wire and the top wire. Only the sense lead may open: on a ladder, keep the ladder joined and open a separate lead from its tap to the connector. Breaking the ladder itself (like a broken cell link) drives its current through the slave's inputs (~17 V across one eval-board input, above the 12 V absolute maximum), and the open-wire check doesn't see that as open. On the eval board's single supply the bottom and top wires carry the supply: test a middle one. |
| 2.7 | Over/under-voltage | Raise one cell above 4.20 V, then drop one below 2.50 V: fault **< 500 ms** each, `Fault_Voltage`. |
| 2.8 | Over/under-temperature | Sensor input to 1.50 V (> 60 °C) then 2.36 V (< -20 °C): fault **< 500 ms**, `Fault_Temp`. Disconnect a sensor: fault. |
| 2.9 | Glitch immunity | A single ~50 ms voltage spike on one cell input must **not** trip the AMS. |
| 2.10 | Balancing | Only runs with the AMS out of fault (on the bench: IVT connected). With the cells within 30 mV of each other, no balancing. Raise one cell 50 mV (on a ladder: a resistor ~2 % of the ladder resistor in series with that cell's): `CellNN_Balancing` = 1, current ≈ V / 15 Ω flows (V / 30 Ω on the eval board; DMM across the resistor), stops when within 20 mV (`CFG_BALANCE_HYSTERESIS_V`). A ladder of more than a few ohms per cell can't supply the current: the cell sags, reads lower after each balancing period and balancing switches on and off; real cells don't do that. Raise all 8: duty limited to 4 W per slave (`CFG_BALANCE_MAX_POWER_PER_SLAVE_W`). Check the slave board temperature after 10 minutes at the limit. |
| 2.11 | LEDs | Call `BMS_SetSlaveLeds(0, true, false)` (debugger or a temporary line): red on, green off, temperature readings unchanged. On the eval board: J5 pin 9 near 0 V, pin 10 high. |
| 2.12 | isoSPI cable | Unplug: fault < 500 ms (`Fault_Comms`). Plug back in: chain re-initialises, fault clears. |
| 2.13 | Slave supply dip | Interrupt the slave's supply for well under 0.1 s (touch and release, or a switch): UART `Slave 1 was reset ...`, readings as before within a cycle, no AMS fault. Interrupted for 1 s: fault < 500 ms (`Fault_Comms`), then the chain re-initialises and the fault clears. |

## Stage 3: Full chain, 14 slaves (no HV)

Delete `Core/Inc/bms_config_local.h` (the car's 14 slaves), or set its `CFG_NUM_SLAVES` to the number of
boards you have. All slaves on cell simulators or low-voltage modules.

| # | Test | Expected |
|---|---|---|
| 3.1 | Measurement cycle | UART `LoopTime` ≤ 70 ms and no `Measurement cycle ... longer than CFG_MEAS_CYCLE_MAX_MS` warnings. If longer, the timing in `bms_config.h` must be reworked: the compile-time checks will tell you what fits in 500 ms. |
| 3.2 | Power cycle the controller 10 times | Every start: chain wakes, no PEC errors after init, fault releases within ~2 s. |
| 3.3 | Slave order | Change one cell on slave 1, then slave 14, and check the right `BMS_SegNN` frames move. |
| 3.4 | Repeat 2.6-2.8 on the first, a middle and the last slave | Same results. |
| 3.5 | CAN1 bus load | With the rest of the car bus running, the BMS adds 24-30 % at 1 Mbit/s. No `CAN TX Buffer Overwritten` on the UART. `BMS_Fast_1` / `BMS_Fast_2` arrive every 4 ms (±1 ms) also during the 10 Hz bursts. Every node on CAN1 (VCU, inverter, logger, charger) must be set to 1 Mbit/s. |
| 3.6 | Soak | Run 1 hour. No faults, no PEC error trend, max loop time stable. |

## Stage 4: IVT-S on CAN2 (no HV)

| # | Test | Expected |
|---|---|---|
| 4.1 | Power up with the IVT | UART `Configuring IVT`; 0x521 every 4 ms, 0x522 every 4 ms, 0x523 every 4 ms and 0x528 every 100 ms on CAN2, nothing else from the IVT (~760 frames/s). Fault releases (if stages 2-3 conditions are healthy). The datasheet is ambiguous on U2's minimum output time (2 or 3 ms): check the 0x523 interval really is 4 ms. |
| 4.2 | Current direction | Pass a known DC current through the shunt in the **discharge** direction (bench PSU and load). `Pack_Current` positive. If negative, flip `CFG_IVT_CURRENT_DISCHARGE_POSITIVE`. |
| 4.3 | U1 | Apply a known voltage to U1: `Pack_Voltage_IVT` matches the DMM. Confirm U1 is on the **battery side** of the AIRs (otherwise the pack-voltage limits will fault whenever the AIRs are open). |
| 4.4 | Overcurrent | Temporarily set `CFG_MAX_DISCHARGE_CURRENT_A` (now 200 A) to just below your test current: fault < 500 ms, `Fault_Current`. Restore the limit. |
| 4.5 | Unplug the IVT CAN or supply | `Fault_IVT` 450-480 ms after the last IVT frame (it waits out an IVT restart). Plug back in: UART `IVT restarted, configuring` after its alive message, results at the BMS's rates again, fault clears. |
| 4.5a | IVT restart | With the IVT result status 0, send `0x411` `3F 00 00 00 00 00 00 00` (IVT RESTART) from the USB-CAN adapter: the IVT is gone for ~240 ms. The BMS sends nothing until its alive message (~190 ms after the command), then reconfigures it at once, and `Fault_IVT` doesn't appear during the gap. The IVT logs a RESTART as a watchdog reset and comes back with status 8 (system error), latched until its supply is cycled, so `Fault_IVT` follows ~0.2 s after its data resumes: cycle the IVT supply afterwards. |
| 4.5b | Power-on order | Power the BMS first, then the IVT (and, from stage 3, the slaves) seconds later, and the other way round: each time the BMS ends up healthy without a reset (UART `Configuring IVT` / `IVT restarted, configuring`, `--- BMS INIT ---` when the chain appears). |
| 4.5c | IVT bus stress test (optional) | USB-CAN adapter on CAN2: `python test/bench/ivt_bus_stress.py --channel <adapter> --uart <ST-LINK COM port>`. Every test passes: floods, junk frames, spurious alive messages, stray commands, the restart, the stand-in IVT's overcurrent / pack voltage / sensor error cases and the soak (every real fault within 500 ms, no false trips). |
| 4.6 | CAN2 termination | The IVT-S-500-U3-I-**CAN2** has no termination on board. With everything unpowered, measure CANH-CANL on CAN2: **60 Ω** (120 Ω at each end). 120 Ω means one terminator is missing. |
| 4.7 | U2 | Apply a known voltage to U2 (vehicle side of the AIRs): `Status Precharge_Percent` and the IVT U2 frame match. |
| 4.8 | Power and energy | With a known current and U1: `BMS_Energy Power` and `Fast_Power` = I x U1 (2.5 W steps). `Fast_Pack_Current` / `Fast_Pack_Voltage` / `Fast_Vehicle_Voltage` equal the IVT's own 0x521 / 0x522 / 0x523 values to the mA / mV. Run 10 minutes and compare `Energy_Used` with I x U1 x time. |
| 4.9 | Inverter limits | `BMS_Inverter_DC_Limit` = 200 A and `BMS_Inverter_DC_Brake_Limit` = -35 A every 20 ms once the fault output releases, 0 A while it's asserted; `BMS_Limits` shows the same with `Discharge_Power_Limit` = discharge limit x U1. Put 1.53 V on one temperature input (57.5 °C): discharge halfway to 25 A, regen 26.7 A, both derating flags set. With the HV500 connected: it reports the same limits (DTI CAN Tool) and won't draw more. |
| 4.10 | Runtime settings (GUI) | `GUI/bms_gui.py`, connected to CAN1: the AMS settings tab reads the built-in values and "nothing saved in flash". Write a power limit of 50 kW: `BMS_Limits` drops to 50 kW (or less) within 20 ms and the Pack tab's "Limited by" shows *settings*. Write 80001 W: refused, value unchanged. Save, power-cycle the BMS: UART `Settings: loaded from flash`, 50 kW still in force. Save with the TS active: refused. Restore built-in values and Save: back to 80 kW after a power cycle. Change the charge current and voltage: the charger command on CAN3 follows. |

## Stage 5: Accumulator (HV, follow the team's HV procedures)

| # | Test | Expected |
|---|---|---|
| 5.1 | Every cell voltage vs a DMM spot check | Within ±10 mV. `Pack_Voltage_IVT` vs `Pack_Voltage_SumOfCells` within a few volts at rest. |
| 5.2 | All temperatures plausible | Within a few °C of each other at rest. |
| 5.3 | AMS light and latch | Any fault opens the SDC, the AMS light stays on until the manual reset (EV 5.8.12, EV 6.1.6), the TSAC ready relay drops. |
| 5.4 | Inspection demonstration (EV 5.8.13, EV 5.8.14) | For each segment, interrupt one voltage sense and one temperature sense: SDC opens. Have a plan for doing this safely on the closed TSAC. |
| 5.5 | Start-up SoC | Note the cell voltages on CAN and enter them in the Battery Tracker: `BMS_Energy SoC` at start-up matches the sheet's Pack SoC (the BMS doesn't truncate to whole %). |
| 5.6 | Precharge | Send the TS request: precharge relay closes, U2 rises, bypass closes at ≥ 95 % (`Precharge_Percent`), precharge relay opens ~50 ms later. Time it: it must finish well inside `CFG_PRECHARGE_TIMEOUT_MS` (now 5000 ms); set the timeout to ~2x the measured time. Scope the AIR coil vs U2 to confirm ≥ 95 % at closing. |
| 5.7 | Precharge aborts | With TS active: send deactivate, open the SDC, and trigger an AMS fault (e.g. pull a sense wire). Each must open both relays. Repeat the request after a failed precharge: nothing happens until the request goes to 0 and back to 1. |
| 5.8 | Charging | Charger button with SDC closed: charging starts. Above 45 °C or with SDC open: charging stops. (New charger code still to come.) |

---

## Results

| Stage | Test | Date | Commit | Result / measured value | By |
|---|---|---|---|---|---|
| | | | | | |
