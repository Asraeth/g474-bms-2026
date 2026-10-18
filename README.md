# Integrated 2026 STM32G474 BMS

Firmware integration repository for the lead vehicle architecture, verified migration safeguards,
and the separate `bms-gui-2026` GUI fork. Current design: **14 ADBMS6830 × 8 populated positions**
and 8 logical temperature inputs per board. Each ADBMS has 16 physical channels; registers remain 6 bytes.
No ADBMS2950 dependency is present in the active path.

The default is the **single-slave BENCH profile**. Balancing, contactors, charger enable, drive and regen
are locked off. Raw AUX is available; unconfirmed Celsius calibration remains invalid. Profiles 1 (lead)
and 2 (V3 candidate) configure fourteen boards but keep energetic hardware unapproved. Profile 4 is a
host-only digital fixture and is rejected by ARM builds. Select with `BMS_BOARD_PROFILE`.

```sh
python tools/build_arm.py debug --profile 3
python tools/build_arm.py release --profile 3
python tools/run_digital.py
python tools/run_digital.py --sanitize
python tools/run_digital.py --pessimistic
python tools/run_digital.py --wrap
python tools/run_digital.py --profile 3 bench_contract
python tools/run_digital.py --variant test/variants/power_window.h power_window_random
python tools/run_migrated_contracts.py
python tools/generate.py --check
```

Use [tomorrow's staged bench guide](docs/TOMORROW-BENCH-BRINGUP.md). It does not require a fourteen-board
chain or connected energetic actuators. Digital checks do not establish physical safety or competition readiness.

Configuration is split under `Core/Inc/config/` and `Core/Inc/profiles/`. Map TEMP1…TEMP8 and LED_RED/GREEN
in the selected board profile; application code uses logical indices. MCU GPIO aliases are centralized in
`profiles/bms_mcu_routes.h`; peripheral alternate functions remain in the CubeMX board configuration.

The canonical protocol generator is `tools/gen_dbc.py` with the configuration and fault definitions.
`tools/generate.py` emits DBCs, signal lists, model and the physical-bus registry in `can/`.
`python tools/sync_gui_protocol.py PATH_TO_GUI --check` prevents divergence between repositories.
Wire/configuration version 2 and matching transaction tokens protect runtime setting exchanges.

See [integration report](docs/integration-final-report.md), [protocol map](docs/protocol-map.md),
[safety parameter register](docs/safety-parameter-register.md) and [hardware validation](docs/hardware-validation-pending.md).
Historical migration fixtures are retained in `tests/`; final-driver contract tests run through
`tools/run_migrated_contracts.py`, and application contracts through the integrated peripheral simulator.

<!-- BEGIN GENERATED: pins -->
| Signal | Pin | Config | Active level |
|---|---|---|---|
| AMS fault output to the latch | PC1 | `CFG_PIN_FAULT_*` | high = fault |
| TSAC ready relay | PC8 | `CFG_PIN_TSAC_READY_*` | high = on |
| Precharge relay | PC6 | `CFG_PIN_PRECHARGE_*` | high = closed |
| Bypass (AIR) | PC9 | `CFG_PIN_BYPASS_*` | high = closed |
| Shutdown circuit sense | PC2 | `CFG_PIN_SDC_IN_*` | high = SDC closed |
| Charger button | PC0 | `CFG_PIN_CHARGER_BTN_*` | low = pressed |
| isoSPI chip select (ADBMS6822 port A) | PB6 | `CFG_PIN_ISOSPI_CS_*` |  |
| isoSPI chip select port B, unused, held high | PC7 | `CFG_PIN_ISOSPI_CS2_*` |  |
| ADBMS6822 MSTR, held high | PA10 | `CFG_PIN_ISOSPI_MSTR_*` |  |
| ADBMS6822 MSTR2 | PB4 | `CFG_PIN_ISOSPI_MSTR2_*` |  |
| CAN1 TX / RX (to the transceiver) | PA12 / PA11 | `g474-bms.ioc` | |
| CAN2 TX / RX (to the transceiver) | PB13 / PB12 | `g474-bms.ioc` | |
| CAN3 TX / RX (to the transceiver) | PA15 / PA8 | `g474-bms.ioc` | |
<!-- END GENERATED: pins -->

<!-- BEGIN GENERATED: can -->
| Bus | Speed | Use |
|---|---|---|
| CAN1 | 1 Mbit/s | Car / datalogger bus, shared with the inverter (DTI HV500): BMS data out, TS request and settings (GUI) in |
| CAN2 | 500 kbit/s | Inside the TSAC: the IVT-S |
| CAN3 | 500 kbit/s | Charger |

CAN1, **1 Mbit/s** (every node on the car bus must match, the HV500 included), 29-bit IDs, 8 bytes, little-endian. IDs 0x0000-0x3FFF are the inverter's (DTI protocol: packet ID << 8 | node ID); the BMS uses 0xB000 and up, so inverter frames win arbitration.

HV500 setup in the DTI CAN Tool (DTI Inverter CAN Manual v2.4): car bus on the inverter's CAN2 port, CAN2 speed 1 Mbit/s, extended IDs (in standard-ID mode every BMS frame would win arbitration over the inverter), CAN2 map version V24, Send CAN2 Status enabled, Drive enable via CAN2 on (or it ignores the limits). Node ID: `CFG_INVERTER_NODE_ID` = 0xFF (DTI broadcast).

Inverter current limits every 20 ms: discharge <= 200 A and 80 kW with resistance bounds and thermal derating. Regen <= 35 A only inside the 0..45 C charge window, below cell OV. Invalid/stale cells, temperature or IVT, unapproved profiles, or AMS fault force both limits to zero. The BMS sends electrical limits; the VCU owns propulsion requests.

Power ceiling: 80 kW in both limiter modes. The former credit-spending boost is replaced by the conservative instantaneous ceiling. The 500 ms moving average remains diagnostic; above-ceiling boost is not enabled. Resistance bounds and actuation delay require physical validation. The random floor regression is mandatory.

- Measurement burst every 100 ms (10 Hz): 120 frames, pack, status and summary first, 16-19 ms on the bus.
- Fast frames every 4 ms (250 Hz): `BMS_Fast_1`, `BMS_Fast_2`. The burst never holds more than one TX buffer, so they wait for at most one frame.
- BMS share of CAN1: 1850 frames/s, 24-30 % (typical to worst-case bit stuffing; compile-time limit 50 %), plus up to 250 settings responses/s while the GUI reads or changes settings.

| ID | Rate | Content |
|---|---|---|
| `0xB000`-`0xB01B` (`0xB000 + slave*2 + n`) | 10 Hz | 4 x int16 cell voltage mV, cells `4n+1`..`4n+4` of the slave |
| `0xB080`-`0xB09B` (`0xB080 + slave*2 + n`) | 10 Hz | 4 x int16 cell temperature 0.1 C, sensors `4n+1`..`4n+4` (888 C = sensor fault) |
| `0xB100`-`0xB10D` (`0xB100 + slave`) | 10 Hz | int16 slave voltage 0.01 V, int16 die temp 0.1 C, byte 4 balancing bitmask, byte 5 cell fault bitmask, byte 6 open wire bitmask (bit0 = cell 1), byte 7: bit0 comms error, bit1 slave fault |
| `0xB200` | 10 Hz | int16 IVT U1 0.1 V, int16 current 0.1 A (+ = discharge), int16 sum of cells 0.1 V, byte 6: bit0 current timeout, bit1 U1 timeout, bit2 IVT error |
| `0xB201` | 10 Hz | byte 0 active fault flags (`BMS_StatusTypeDef`), byte 1: bit0 fault output, bit1 TSAC ready, bit2 balancing enabled, bit3 charging, bit4 SDC closed, bit5 bench build (`bms_config_local.h`), bit6 inverter limits derated for temperature, bit7 inverter discharge at the power limit, int16 measurement age ms, int16 loop time ms, byte 6 precharge state (0 idle, 1 precharging, 2 closing, 3 TS active, 4 failed), byte 7 U2/U1 % |
| `0xB202` | 10 Hz | int16 min cell mV, int16 max cell mV, int16 min temp 0.1 C, int16 max temp 0.1 C |
| `0xB203` | 10 Hz | int16 power 2.5 W (+ = discharge, saturates at ±81917.5 W, -32768 = not available), uint16 remaining energy Wh (0xFFFF = not estimated yet), uint16 SoC 0.1 % (0xFFFF = not estimated yet), int16 energy used since start-up Wh |
| `0xB204` | 250 Hz | int32 IVT current mA (+ = discharge), int32 IVT U1 (pack) mV. INT32_MIN = not available |
| `0xB205` | 250 Hz | int32 IVT U2 (vehicle side) mV (INT32_MIN = not available), int16 power 2.5 W (+ = discharge, saturates at ±81917.5 W, -32768 = not available), uint16 remaining energy Wh (0xFFFF = unknown) |
| `0xB206` | 50 Hz | int16 discharge power limit 2.5 W (current limit x pack voltage), int16 discharge current limit 0.1 A, int16 regen current limit 0.1 A, byte 6: bit0 discharge derated for temperature, bit1 held to the power limit, bit2 regen derated for temperature, bit3 AMS fault (0 A), bit4 window limiter boosting, bit5 runtime settings differ from the built-in limits, byte 7 500 ms average discharge power 0.5 kW |
| `0xB207` | on request (at most one per fast period) | Settings response: byte 0 command, byte 1 setting, byte 2 result (0 ok, 1 unknown setting, 2 out of range, 3 TS active, 4 flash error, 5 unknown command, 6 busy, 7 incompatible version, 8 malformed), byte 3: bit0 unsaved, bit1 differs from the built-in values, bit2 saved settings in flash, int32 value in force |
| `0x0AFF` | 50 Hz | To the inverter: DTI "Set max. DC current" (0x0A), int16 big-endian A x 10, bytes 2-7 0xFF |
| `0x0BFF` | 50 Hz | To the inverter: DTI "Set max. DC brake current" (0x0B), negative, same format |
| `0xB300` (TBD, extended, received) | from the VCU | TS request: byte 0 = 1 activate, 0 deactivate |
| `0xB301` (extended, received) | from the GUI | Settings request: byte 0 command (1 read, 2 write, 3 save, 4 defaults, 5 read all), byte 1 setting (0 for save, defaults, read all), bytes 4-7 int32 value (write) |

CAN2, 500 kbit/s: IVT-S (factory IDs): `0x521` current, `0x522` U1, `0x523` U2 every 4 ms, `0x528` energy every 100 ms; U3, temperature, power and As off: 760 result messages/s (IVT-S maximum 1000; U1 and U2 share one ADC, 2 ms minimum each). The BMS configures this at start-up.

CAN3, 500 kbit/s: charger (Elcon/TC protocol until the new charger section): `0xFFFFFF` command from the BMS every 100 ms (big-endian), `0x1100000` status from the charger.

Every signal: `can/bms_can1.dbc`, `can/ivt_can2.dbc`, `can/charger_can3.dbc`, or the matching `can/*_signals.csv` in the CAN sheet's columns.

Runtime settings (GUI, `BMS_Config_Request` / `BMS_Config_Response`; values are int32 in the raw unit). A write applies at once, save stores them in flash (refused while the TS is active or charging), and saved settings load at start-up. The inverter limits can only go below the built-in values:

| ID | Setting | Raw unit | Range | Built-in | |
|---|---|---|---|---|---|
| 1 | Power limit | W | 1-80 kW | 80 kW | Discharge power limit sent to the inverter (the window limit with the moving-window limiter). Can only be lowered below the built-in value |
| 2 | Discharge current limit | 0.1 A | 1-200 A | 200 A | Inverter DC current limit before derating. The AMS trip stays at the built-in value |
| 3 | Temperature limit | 0.1 degC | 30-60 °C | 60 °C | Hottest cell at which both inverter limits are fully derated (discharge to 25 A, regen to 25 A). Lowering it moves both derating ramps down by the same amount. The AMS over-temperature trip is not changed |
| 4 | Regen | - | 0-1  | 1  | 0 = off (0 A brake current to the inverter), 1 = on |
| 5 | Regen current limit | 0.1 A | 0-35 A | 35 A | Inverter DC brake current limit before derating |
| 6 | Charge current | 0.1 A | 0-9.9 A | 1 A | Charger current setpoint |
| 7 | Charge voltage | 0.1 V | 280-464.8 V | 450.2 V | Charger voltage setpoint (pack voltage at the end of charge) |
<!-- END GENERATED: can -->
