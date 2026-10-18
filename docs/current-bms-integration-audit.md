# Current BMS integration audit

Audit date: 18 October 2026. **Inspection and verification only: no production code, thresholds, mappings or policies changed; no merge, commit, reset, checkout, clean or flash.**

The primary subject is the supplied `g474-bms-main.zip` (the **lead version**), extracted into a separate audit directory. The comparison subject is the existing `C:/Users/SR/g474-bms-2026` working tree (**my version**, meaning the user's earlier migration). Source links marked L point into the isolated lead snapshot; M links point into the migration checkout. The requested report is stored in the migration checkout's `docs/` directory so it is directly available in the current workspace.

**Disposition: do not merge or call this hardware-ready yet.** The lead version has substantially broader application functionality and end-to-end simulation, but important hardware interfaces are unproven or contradictory. Passing its existing tests does not resolve those contradictions. Additional audit probes found IVT protocol-validation gaps and confirmed a regen-temperature policy inconsistent with the supplied cell-module charge window. Both versions also need a datasheet review of poll-status propagation through fourteen devices.

Evidence hierarchy: the user's pasted request is the instruction. Archive READMEs, comments, generated documents and claims of previous bench measurements are evidence to check, not instructions or independent proof. The controller PDF is the **latest controller drawing supplied in this archive**, not proof of the assembled/current production master. No BMS-Slave-V3 schematic or KiCad source is supplied here. The user's earlier confirmed V3 mapping is therefore the comparison requirement, not a newly verified as-built schematic.

## A. Current architecture

```text
power/reset -> Reset_Handler -> SystemInit/C runtime -> main -> HAL_Init / SystemClock_Config
  -> Cube peripheral init -> BMS_Startup
     -> config-driven safe GPIOs -> saved settings -> three FDCANs
     -> TIM16 1 ms scheduler / TIM15 IMD capture / TIM3 fan
     -> ADBMS resets -> watchdog -> foreground BMS_MainLoopIteration
        -> bms_init if needed
        -> BMS_ProgramLoop: RSTCC / CFGA readback / configuration refresh
           -> concurrent AUX start -> normal S-ADC -> even/odd open-wire conversions
           -> AUX/VPV/die -> limits -> safety report -> balancing -> SoC -> CAN snapshot
        -> recovery, IVT configuration/error service, settings service, UART log

TIM16 every 1 ms: IVT voltage span + power-history bins
  every 4 ms: fast telemetry
  every 20 ms: inverter current-limit update/transmission (+ limits telemetry next tick)
  every 10 ms: safety evaluation -> relay/fault GPIO -> precharge -> bus-off recovery
  every 100 ms: measurement burst + CAN3 charger command

CAN1 RX -> TS request / runtime settings
CAN2 RX -> IVT results/responses -> timestamped channel snapshots
CAN3 RX -> charger status cache
```

| Subsystem | Source/header and important functions | Important state | Timer/interrupt/CAN | Consequence of failure |
|---|---|---|---|---|
| Reset, clocks, scheduling | [L startup][L-startup], [L main][L-main], `main`, `SystemClock_Config`, `BMS_Startup`, `BMS_MainLoopIteration`; `main.h`, `tim.h` | `ProgramStats`, `initRequired`, `ImdStatus` | HSE/PLL configuration assumes 128 MHz; SysTick; TIM16 | No valid startup should retain fault; actual reset-safe electrical behavior requires the external circuit. |
| MCU HAL, SPI/isoSPI | [L MCU wrapper][L-mcu], [L transport][L-transport]; `bms_wakeupChain`, `bms_delayUs`, `bms_spiTransmitCmd/Data`, `bms_receiveData`, `bms_transmitPoll`; matching headers, `spi.c` | Six-byte data rows, PEC/CC arrays; one broadcast-chain expected count | SPI1 PA5/6/7; CS PB6; TIM2 1 us counter; foreground blocking HAL transfers | Missing/incorrect conversions; counter/PEC rejection and measurement-age protection. TIM2 delay itself is unbounded. |
| ADBMS initialization | [L wrapper][L-wrapper], `bms_init`, `bms_findResetSlaves`, `bms_startAdcvCont`, `bms_applyGpo` | `Ic_ad68.cfa_Tx/cfb_Tx`, REFON, `voltageGroupMask`, LED shadow state | Foreground; RSTCC/RDSID/CFGA/ADCV | Wrong GPIO mapping can invalidate temperature measurements; missed reset recovery can leave conversions inappropriate. |
| Cell acquisition | `bms_convertS`, `bms_readSVoltages`, `bms_measureCells`, `bms_calculateStats` in [L wrapper][L-wrapper] | Physical 14x16 normal/even/odd buffers; logical 14x8 `v_cell`, board/pack sums | Foreground ADSV/PLSADC/RDSVA-C | OV/UV/open-wire protection and telemetry depend on these values. |
| AUX/temperature | `bms_startAuxMeasurement`, `bms_readAuxMeasurement`, `convertCellTemp`; same wrapper/header and [L config][L-config] | Temporary 14x10 GPIO volts; published 14x8 Celsius; 14 VPV/die values | ADAX in parallel with S-ADC; PLAUX1, RDAUXA-D, RDSTATA | Wrong pin or bias can yield plausible but wrong Celsius. No independent per-input validity structure. |
| IVT | [L IVT driver][L-ivt], `bms_ivt.h`; `BMS_IVT_HandleFrame`, `GetReading`, `Tick`, `SpanStart/End`, `Service` | Private `IvtValue[4]`, `IvtReading`, configuration/check state, response counters | FDCAN2 RX; TIM16 averaging; foreground configuration | Current/U1 loss eventually faults; U2 is required during precharge; replayed counters can currently keep old measurements fresh. |
| CAN routing/queues | [L CAN][L-can], `bms_can.h`, [L FDCAN][L-fdcan]; `BMS_CAN_Config`, `SendBuffer/Fast/Charger`, `drainRxFifo`, `RecoverBusOff` | Burst queue, three hardware TX buffers/bus, drop/error counters | FDCAN1/2/3 IT0; all application IRQ priorities 0 | CAN2 loss reaches AMS through IVT age. CAN1/3 errors are counted, not generic AMS faults. |
| Safety/watchdog | [L safety][L-safety], `bms_safety.h`; `ReportMeasurement`, `Tick`, `evaluateIvt`, `ForceFault`, `Watchdog_Start/Kick` | `MeasurementState`, per-bit `OverLimitTimer`, IVT timers, active flags, fault-output state, 16-event ring | TIM16 safety tick 10 ms; foreground publication under PRIMASK; IWDG | Drives logical AMS fault and relay outputs. Real latch, output polarity and timing remain unverified. |
| Charging | `BMS_ChargingButtonLogic`, `EnableCharging`, `GetChargerCanMsg` in wrapper; `parse_charger_status` / `GetChargerMsg` in CAN | `chargerConfig`, `chargerStatus`; saved setpoints | PC0 polled at safety ticks; FDCAN3 command every 100 ms | Old/stale charger status and incompatible protocol prevent proving safe charger control. |
| Precharge | [L precharge][L-precharge], `bms_precharge.h`; `HandleRequest`, `Tick`, `enter` | `PrechargeState`, request edge, state/request timestamps, U2/U1 percentage | CAN1 request; TIM16 10 ms; PC6/PC9 | Software opens its outputs on failure, but the supplied PCB has no corresponding drivers. |
| Inverter current limits | [L inverter][L-inverter], `bms_inverter.h`; `ComputeLimits`, `currentForPower`, `Tick`, `Update`, `GetCanFrames` | `InverterLimits`, 500 energy/credit bins, runtime setting snapshot | TIM16 1/20 ms; FDCAN1 TX only | Limits depend on sensor correctness, thermal policy, resistance assumptions and inverter configuration. No inverter feedback validation. |
| Balancing | `bms_startDischarge`, `bms_setPwm`, `bms_stopDischarge`, `bms_isBalancingAllowed`; wrapper/config | PWM A/B, DCTO, discharge bitmask, enable flag | Foreground ADBMS writes | Enabled by default; unapproved resistor/board heat budget can overheat hardware. A lost link can delay stopping until the slave timeout. |
| Shutdown / ready outputs | [L GPIO][L-gpio], `bms_gpio.h`; `WriteFault`, `WriteTsacReady`, `WritePrecharge/Bypass`; safety/error handlers | Logical fault output and relay states | PC1, PC8, PC6, PC9; safety IRQ | Output pins are not proof of drivers or a latch. Master Q1 inverts PC1 to external FAULT_OUT. |
| Telemetry, SoC, settings/debug | `bms_publishCanFrames`, `BMS_GetCanData/FastCanData`; [L SoC][L-soc], `bms_soc.h`; [L settings][L-settings], `bms_settings.h`; `uartDMA.c/h`; GUI, generated DBCs | `measFrames[2]`, `PackSummary[2]`, `SocState`, `BmsSettings`, flash record/CRC, 10 kB UART ring | CAN1, LPUART1 PA2/3 DMA1; last flash page | Stale/mixed samples must be interpreted correctly; model/settings verification is not hardware calibration. |

## B. Git/build/test state

| Subject | Branch / commit | Working state | Remotes |
|---|---|---|---|
| Lead ZIP | `main` / `75ea736c4108cb34aff9e7fd8996d6ff643494d7` | Clean; 200 tracked-file hashes unchanged by all builds/tests | None configured |
| Earlier migration | `develop-2026-bms` / `919a8a8e3f5cf7461b3a6890139e030517976772` | Dirty before audit; all 129 production-source hashes still match the preceding verified state | `origin	https://github.com/Asraeth/g474-bms-2026.git (fetch)<br>origin	https://github.com/Asraeth/g474-bms-2026.git (push)` |

ZIP SHA-256: `16b08edeb28e3949f95fa9597a4da518f801c8ac48a4e49fc1232fe121340323` (366547289 bytes; 13596 entries). A safe separate extraction excluded the bundled GUI virtualenv and bulk charger/LVGL tree; relevant charger reference sources were read separately. No archived executable was used as a test result.

A clean lead Git tree is a useful source baseline, but not a fully pinned/reproducible hardware baseline: no remote is configured, tool/Python dependency versions are not locked, reference schematics/datasheets are ignored, and a team-board hardware revision is missing. Battery Tracker.xlsx is tracked and its correlation test ran. The migration HEAD object is absent from the supplied lead object database, so no common ancestor/merge provenance was assumed or manufactured.

Migration modified files at audit start (all pre-existing):
```text
M Core/Inc/bms_can.h
 M Core/Inc/main.h
 M Core/Inc/stm32g4xx_it.h
 M Core/Src/bms_can.c
 M Core/Src/fdcan.c
 M Core/Src/main.c
 M Core/Src/stm32g4xx_it.c
 M Core/custom_lib/inc/bms_libWrapper.h
 M Core/custom_lib/inc/bms_mcuWrapper.h
 M Core/custom_lib/inc/bms_utility.h
 M Core/custom_lib/src/bms_libWrapper.c
 M Core/custom_lib/src/bms_mcuWrapper.c
 M Core/custom_lib/src/bms_utility.c
 M g474-bms.ioc
```
Migration untracked files at audit start (also pre-existing; this report is subsequently added):
```text
Core/Inc/bms_safety_config.h
Core/Inc/bms_topology.h
Core/custom_lib/inc/bms_command_counter.h
Core/custom_lib/inc/ivt_s.h
Core/custom_lib/src/bms_command_counter.c
Core/custom_lib/src/ivt_s.c
docs/ivt-s-integration.md
docs/p1-warning-fixes-verification.md
docs/safety-comms-bringup.md
docs/software-readiness-level2.md
docs/virtual-bms-bench-verification.md
tests/adbms_counter_test.c
tests/adbms_transport_test.c
tests/bms_p1_regression.c
tests/bms_pack_test.c
tests/ivt_s_test.c
tests/run_host_tests.py
tests/run_p1_regression.py
tests/run_virtual_bench.py
tests/virtual_bms_bench.c
```

| Verification | Result | Warnings/errors | Evidence |
|---|---|---|---|
| arm-debug | PASS; 28.0s wall time | 0 warnings, 0 compiler errors | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/arm-debug/console.log>) |
| arm-release | PASS; 24.2s wall time | 0 warnings, 0 compiler errors | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/arm-release/console.log>) |
| host | 172 passed / 0 failed; 246.2s wall time | 52 warnings, 0 compiler errors; intentional local/variant override #warning only | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/host/console.log>) |
| host-ubsan | 172 passed / 0 failed; 368.8s wall time | 52 warnings, 0 compiler errors; intentional local/variant override #warning only | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/host-ubsan/console.log>) |
| host-pessimistic | 172 passed / 0 failed; 188.7s wall time | 52 warnings, 0 compiler errors; intentional local/variant override #warning only | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/host-pessimistic/console.log>) |
| host-wrap | 172 passed / 0 failed; 229.4s wall time | 52 warnings, 0 compiler errors; intentional local/variant override #warning only | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/host-wrap/console.log>) |
| host-window-excluded (documented excluded case, limiter explicitly enabled) | 2 passed / 1 failed; 16.6s wall time | 26 warnings, 0 compiler errors; intentional local/variant override #warning only | [log](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/runs/host-window-excluded/console.log>) |
| Earlier migration full ARM / Level-2 / virtual bench / numeric regressions | PASS: 41-source ARM build; 37 groups; 20 scenarios in O1/O2/UBSan; exhaustive numeric/ISR/NaN checks at O1/O2/UBSan | ARM 0 warnings/errors; expanded virtual warning inventory remains 189 | [ARM](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/comparison/firmware/build.log>); [37 groups](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/comparison/level2/host-tests.log>); [bench](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/comparison/virtual-report.md>); [numeric](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/comparison/numeric/results.json>) |

Normal-mode soaks: seed1 158 real faults /337 harmless glitches /0 false trips, worst459.0ms; seed2 155/326/0, worst456.2ms; seed3 156/329/0, worst458.8ms. Three600s virtual soaks ran again in UBSan, pessimistic and wrap modes; their per-mode results are preserved in the logs/JSON. These are model results, not independent target measurements.

The explicit enabled-window `power_window_random` run failed: lowest55.4kW against the72kW floor,20ms dip; measured simulated logger500ms peak80.08kW passes that test's tolerance but is not proof of an exact80kW physical ceiling. The default production window limiter remains off. No test or production code was changed to hide this failure.

Tools used: Git Bash; ARM GCC14.3.1 from STM32CubeIDE2.2.0; host GCC15.2.0 (earlier legacy37-group runner uses GCC6.3.0); bundled CPython3.12.14. Isolated audit-only Python dependencies: cantools45.0.0, python-can4.6.1, pyserial3.5, PySide6-Essentials6.12.0, pyqtgraph0.14.0, numpy2.5.4; existing openpyxl for the workbook. GUI tests ran offscreen on a virtual bus. No global Python environment was modified.

Lead UBSan mode uses the repository's `SANITIZE=1` (undefined and float-cast-overflow, trap mode); it passed the full suite. The comparison virtual bench also probes sanitizer support: runtime ASan/UBSan libraries are unavailable in installed MinGW, but UBSan trap instrumentation and its negative controls work. No ASan success is claimed.

Build paths are `tools/build_arm.sh debug|release` and CubeIDE `.cproject`. The CLI script builds all Core/HAL sources with ARM GCC, GNU11, Cortex-M4 hard-float, `-O0 -g3` or `-Os -g0`, section garbage collection and nano/newlib float formatting. Project sources get additional warnings. The flash linker reserves the last 4 kB for settings. The command-line build does not itself run the generator; `tools/generate.py --check` was run separately and passed. The host runner invokes the generator; tracked-file hashes confirmed its outputs were already current.

The host runner compiles the actual startup/main loop and peripheral initialization against simulator HAL/CMSIS implementations. It includes 70 named scenarios plus 16 fault classes, each fault injected at 12 measurement-cycle phases. The full normal run contains 86 primary scenarios, 77 eval-profile scenarios, 5 enabled-power-window scenarios, register layout, DBC, spreadsheet and GUI checks (the GUI group contains 12 Python tests). Register layout was checked with both ARM and host compilers. Existing script timing columns show `?s` because `bc` is unavailable; the audit runner records wall-clock duration independently.

Three soaks run for 600 **simulated** seconds each per complete mode; these are not 30 minutes of real hardware operation. The simulator models bus arbitration, ADC/HAL delays, loss/error cases and watchdog time, but does not assign target execution cycles to ordinary C arithmetic, formatting, interrupt handlers or interrupt-masked copies. Its reaction times therefore do not prove target WCET/interrupt latency.

Additional probes compile/link the unchanged lead IVT/transport/inverter sources outside the repository. They are separate from the original green suite:

| Audit probe | Observed behavior | Disposition |
|---|---|---|
| Replay current counter 0 repeatedly from tick 100 to 660 | `currentFresh=1`, age=0, timestamp=660, although the counter never advanced | Confirmed protocol/freshness gap: counter nibble is discarded. |
| IVT result DLC 7 and 8 | Both update the timestamp and are fresh | Confirmed protocol-validation gap against the specified DLC 6. |
| Poll bytes 0x01 / 0x7F / 0xFF / continuous 0x00 | First three accepted after one byte; continuous zero times out at 16 ms with a 15 ms budget | Acceptance of non-FF alone is not proof of a bad ready decision: the datasheet defines a bit-level signal which can rise mid-byte. The unmodelled initial 2xN status-propagation interval is the important unresolved issue (section E). |
| Die raw code 7150 | Lead expression gives 70.000030518 C and exceeds the 70 C limit; earlier qualified expression gives exactly 70.0 C | Confirmed arithmetic-boundary behavior difference; review before choosing a conversion policy. This is an expression-level reproduction, not an injected full-chain ADC test. |
| Actual inverter calculation at -10 / 50 / 60 C, no AMS fault | Permits 35 / 31.666666 / 25 A regen respectively | Confirmed policy mismatch with the supplied Enepaq 0..45 C charge window; the relevant sensor/hardware and policy must be approved. |

[Probe source][probes-src], [compiler command][probes-command], [observations][probes-results]. The temporary hardware mocks supply time, SPI bytes and settings; no real bus/output is used.

Known omissions are explicit: physical `test/bench/ivt_bus_stress.py` was not executed; no USB/CAN/serial hardware was driven. The standard enabled-power-window variant omits `power_window_random`; this audit also invokes that excluded case explicitly (result above). No failing code/test was changed to obtain a pass.

## C. Hardware/software interface map

The available master reference is **CAN Controller PCB V1**, dated **31 August 2026**, exported from `CAN (2).SchDoc` in [the supplied controller PDF][master-pdf]. It shows:

| Interface | Firmware | Drawing evidence / limitation |
|---|---|---|
| Controller supply | Nucleo/STM32G474 startup; 128 MHz clock assumption | 12 V input through TEC2-1211WI 5 V converter; USB/5 V selector. No physical clock/timing measurement was performed. |
| Vehicle CAN | PA11 RX / PA12 TX | J8 pins 14/12 -> U1 -> J1 CAN1 H/L/GND. Matches the software pins. |
| Second CAN | PB12 RX / PB13 TX | J8 pins 16/30 -> U2 -> J4, labelled **Charger CAN (CAN2)**. Lead firmware assigns those pins to the IVT bus instead. |
| Third CAN | PA8 RX / PA15 TX | No third transceiver/connector shown. Merely enabling FDCAN3 is insufficient. |
| Logical fault | PC1 open-drain, high/released = logical fault | J7 pin 36 -> FAULT_CTRL -> R10/Q1. R9 pulls Q1 gate up to 5 V. Q1 on pulls external J6 FAULT_OUT **low**, against R8/12 V and the zener. Logical fault high is not external connector high. |
| SDC sense | PC2, high = SDC closed | J7 pin 35 is labelled BMS_MASTER_OC1_IN, routed from J3 through 1 k. No SDC source, isolation or accepted voltage-level proof shown. |
| Charger button | PC0 active-low input/pull-up | J7 pin 38 -> J2 CHARGING_BUTTON_IN, switch to GND. Drawing has an external 5 V pull-up; confirm MCU input compatibility. |
| Ready/precharge/bypass | PC8 / PC6 / PC9, active-high, off in reset assumed by comments | These nets have no relay/FET drivers or dedicated output connectors in the supplied drawing. External pull-downs are asserted by code comments, not evidenced here. |
| Fan | TIM3_CH2 PA4 | J7 pin 30 -> R12/Q2 -> J9. Inverted low-side output; schematic says 0% gives full fan speed. |
| IMD PWM | TIM15_CH1 PB14, CH2 indirect capture | J5 resistor divider -> J8 pin 28. Captured only as `imdStatus`; no IMD AMS fault check in this application. At the configured128MHz, prescaler0 and ARR65535 the timer wraps every512us; no overflow extension is present. Actual PWM compatibility and external IMD shutdown ownership are unproven. |
| SPI/HAT | SPI1 PA5/6/7, CS PB6, MSTR PA10; second CS PC7 and MSTR2 PB4 held high | Requires an external ADBMS6822 interface/HAT. Vendor evaluation design files are references, not the team's verified assembled HAT/slave wiring. |

**Additional CAN hardware discrepancy:** U1/U2 are labelled `TCAN1044VDRQ1`, but their pin 5 (`NC/VIO`) is drawn unconnected. For the V variant, TI defines pin 5 as the I/O supply, not NC. Confirm the fitted BOM variant and VIO net before relying on either bus. This is an as-drawn discrepancy, not a claim about an unseen PCB modification. [TI TCAN1044-Q1/V-Q1 datasheet, pin-functions table](https://www.ti.com/lit/ds/symlink/tcan1044-q1.pdf).

No non-programmable latch is shown on this controller sheet. Do not infer that one exists elsewhere from comments. The requested latch reset behavior, 5 V/12 V loss behavior, PC1 high/low/released behavior and external SDC polarity require an approved full-circuit truth table and bench verification.

## D. Three-CAN architecture

All three peripherals use Classical CAN, normal mode, retransmission enabled, BRS off. With the configured 128 MHz PCLK and common divider 2, the nominal bitrate is `64 MHz / (prescaler x (1+13+2))`.

| Peripheral | RX / TX (AF) | Intended bus / speed | Transceiver / connector supplied | Attached devices | RX IDs in BMS | TX IDs from BMS |
|---|---|---|---|---|---|---|
| FDCAN1 | PA11 / PA12, AF9 | TSCAN, 1 Mbit/s; prescaler 4 | U1 TCAN1044VDRQ1 / J1, subject to VIO discrepancy | BMS, inverter, VCU, Badenia/logger per requested architecture; those other nodes' configurations are not supplied | Extended `0xB300` TS request (TBD), `0xB301` settings only | Extended cell `B000-B01B`, temp `B080-B09B`, slave `B100-B10D`, `B200-B207`; DTI `0xAFF` / `0xBFF` with node FF |
| FDCAN2 | PB12 / PB13, AF9 | TSACCAN, 500 kbit/s; prescaler 8 | U2 / J4 is physically labelled **charger CAN2** in the drawing; reassignment not approved | IVT; possible future DC/DC not implemented | Standard `0x521-0x522` to FIFO0; `0x511` and `0x523-0x528` to FIFO1 | Standard `0x411`, DLC8 commands |
| FDCAN3 | PA8 / PA15, AF11 | CHRGCAN, 250 kbit/s; prescaler 16 | **Absent in supplied master drawing** | Intended charger; exact charger/gateway unresolved | Extended `0x18FF50E5` | Extended `0x1806E5F4`, DLC8 every 100 ms |

For CAN2, the response 0x511 is routed by its own filter to FIFO1; I/U1 use FIFO0 and U2..Wh FIFO1. Global filters reject other IDs and remote frames. CAN1 does **not** accept DTI status packets; no inverter feedback/acknowledgement is parsed.

CAN1 frame map: 28 voltage + 28 temperature + 14 board frames + 4 pack/status/summary/energy = 74-frame measurement burst every 100 ms. Fast I/U1 and U2/power/energy frames run every 4 ms; two DTI limit commands every 20 ms; limits telemetry on the next tick; settings responses at most once per fast period. Built-in worst-case load calculation, including settings responses, is 1640 frames/s x 160 bits = 26.24% of 1 Mbit/s. The normal simulation observed about 20.2% with typical stuffing and its traffic pattern. This is not a measured whole-vehicle bus budget.

The enumerated local CAN1 ranges are disjoint: 80 distinct BMS-transmitted IDs including the two DTI command IDs; the two request IDs are separate. The generated DBC/layout tests passed. That proves local layout consistency, **not absence of collisions with an unavailable VCU/Badenia/logger network database**. DTI's low-ID range is reserved by compile-time checks. Existing migration IDs B07E/B07F/B080 and its per-cell/per-board layout are not wire-compatible with the lead's packed schema; B080 is a temperature frame in the lead version.

## E. ADBMS topology

**Software topology is consistently 14 x 8 = 112 monitored series positions.** No ADBMS2950 remains in the production measurement chain. Residual ADBMS2950 text in the README is historical documentation, not an active device offset.

| Concern | Verified implementation / limitation |
|---|---|
| Logical arrays | `Ic_ad68.v_cell[14][8]`, `temp_cell[14][8]`; 14-element sums/min/max/VPV/die/flags. Statistics iterate eight logical positions per board, 112 total. |
| Physical register capacity | `NUM_CHANNELS=16`, six possible S-voltage group commands, physical `vNormal/vOwEven/vOwOdd[14][16]`. These are deliberate register-space arrays. |
| Configured populated channels | `{1,2,3,4,5,6,7,8}`; `voltageGroupMask` selects A/B/C only. Current confirmed logical topology supports this, but the actual V3 cell depopulation/termination wiring is not present to reverify. |
| Parser bounds/payload | Three words per selected group, `ch < 16`; full six-byte rows retained. Group C's C9 word is stored only in the physical buffer and never selected for the eight logical cells. |
| Board indexing | Array index 0 is the first returned device; read order nearest-first. Writes reverse the 14 rows because each device retains the last group reaching it. Board numbering is array+1 for humans. No first-master/2950 offset. |
| PEC | CRC15 command PEC, CRC10 data PEC with six received command-counter bits. Each read checks all 14 devices before its group is used. Full six-byte register payloads and eight-byte data+CC/PEC packets are preserved. |
| Expected counters | One expected count per broadcast chain, comparison against each device's received CC. RSTCC at initialization and each acquisition; normal commands/writes/polls increment, reads do not; 63->1 rollover. PEC is checked before trusting a device's counter. Reset slaves are detected using REFON readback and reconfigured. |
| Counter limitations | Single chain only, no independently reusable per-chain object or latched per-device sequence history. Send helpers count even failed/ambiguous transfers; failed cycles are abandoned and next RSTCC establishes the next comparison baseline. Some reset-command return values are ignored. Existing missed-command/reset tests pass; this does not make reset/send verification as explicit as the earlier driver's state machine. |
| Timeouts | SPI HAL 10 ms; S-ADC poll 15 ms; AUX poll 30 ms. Outer poll uses `>` and can overrun its nominal budget by an in-progress HAL receive. `bms_delayUs` has an unbounded TIM2 busy wait; main-loop watchdog does not start until after early startup wake/reset/delay operations. |
| Poll-status propagation | ADBMS6830B Rev.0 p55 says status becomes valid only after 2xN clocks/data pulses: 28 for 14 devices. Neither implementation explicitly discards that initial interval after issuing the polling command. The lead simulator immediately returns aggregate busy/ready and does not model the invalid prefix. A partial nonzero byte can legitimately contain a late ready bit, so requiring FF is not by itself the datasheet rule. Resolve initial priming and observation timing from the complete transaction/6822 behavior before first chain signoff. |
| Acquisition publication | A complete successful cycle alone refreshes the safety measurement timestamp. **Application values are not fully transactional**: cells are copied before open-wire/AUX completion; AUX updates board values while groups arrive; `bms_publishCanFrames()` runs even after a failed cycle. Double-buffered CAN prevents a torn buffer swap, not mixed acquisition generations. |
| Stale behavior | Safety retains the last reported complete cycle and faults after age >450 ms; it deliberately rides through brief loss. CAN always republishes the current back-buffer contents and carries a separate status-age field. Consumers must not interpret every recently received cell frame as a fresh complete acquisition. Failed AUX/OW cycles can mix new cells with old temperatures in telemetry. |
| Reset codes / numeric validity | Raw signed words are scaled directly; INT16_MIN/MAX are not represented by independent acquisition-valid flags. Reset codes usually become out-of-range voltage/temperature values, rather than explicit invalid measurements. No per-input voltage/Celsius validity state comparable to the earlier migration. |
| Open wire | Enabled each cycle. Normal + even/odd ADSV conversions; compare each populated physical channel's OW value against 75% of normal; logical bitmask and minimum intact ratio reported. Filter/resistor-dependent threshold needs the actual slave and broken-wire tests. C-ADC continuous conversion is maintained, but its cell results/comparison diagnostics are not read into the application. |
| Diagnostics not covered | Only RDSTATA is in the AUX/status list. No complete status-C/D/E internal diagnostic/supply/oscillator/C-vs-S fault evaluation or daisy-chain redundancy/second-port strategy is implemented. Open-wire tests are not a complete device self-diagnostic safety argument. |

Transformer coupling does not require a new register format/parser; correct ADBMS6822 mode, wake timing, polarity, termination, link wiring and full-chain propagation still require the actual HAT/slave hardware. The vendor 6822/6830 evaluation files do not establish those team-board facts.

## F. Temperature/LED mapping

| Logical input | Lead `CFG_SLAVE_TEMP_GPIOS` | Earlier user-confirmed V3 requirement | Status |
|---|---:|---:|---|
| TEMP1 | GPIO1 | GPIO9 | CONFLICT |
| TEMP2 | GPIO2 | GPIO7 | CONFLICT |
| TEMP3 | GPIO3 | GPIO8 | CONFLICT |
| TEMP4 | GPIO4 | GPIO10 | CONFLICT |
| TEMP5 | GPIO5 | GPIO4 | CONFLICT |
| TEMP6 | GPIO6 | GPIO6 | MATCH to reported requirement; not as-built verified |
| TEMP7 | GPIO7 | GPIO5 | CONFLICT |
| TEMP8 | GPIO8 | GPIO3 | CONFLICT |
| Red LED | GPIO9 | LEDs reported on GPIO1/2; color assignment unavailable | BLOCKED |
| Green LED | GPIO10 | LEDs reported on GPIO1/2; color assignment unavailable | BLOCKED |

`bms_applyGpo()` releases all ten GPIO pull-downs except configured active LEDs. With the earlier V3 mapping, lighting red/green would pull actual TEMP1/TEMP4 low. Measuring the actual LED GPIOs as TEMP1/2 can also produce plausible but unrelated temperatures. LED states default off; the public setter is used by tests, and there is no automatic production fault/status LED policy caller. That does not cure the incorrect temperature selection.

The archive-wide inventory contains **no `.kicad_sch`, legacy `.sch`, or BMS-Slave-V3 schematic source/export**. Available schematic PDFs are the controller PCB, Nucleo MB1367 and vendor EVAL-ADBMS6830BMSW; the ADI design directories also include vendor Gerbers/BOM and a `.pdsprj`. None is the V3 slave. The earlier schematic was described as about 90% correct; a signed as-built revision and connector/pin table are still needed. Do not silently choose either mapping or color assignment.

The lead converts to Celsius using the supplied Enepaq table. That table is documented for an Enepaq temperature-dependent shunt reference with the specified bias, **not a generic thermistor calibration**. The table matches the provided datasheet, but actual sensor model, pull-up/bias, reference and harness must be proven. The earlier migration intentionally kept raw input validity separate and Celsius unavailable until that approval. No conversion was changed by this audit.

## G. IVT-S architecture

Official supplied protocol source: [IVT-S datasheet Version 1.01][ivt-pdf], pp18-25 and response descriptions. The lead requires **I, U1, U2 and Wh**. U3, sensor temperature, W and As are explicitly disabled by boot configuration. U1/U2/Wh are not assumed to arrive by factory defaults; the p19/p24 default inconsistency is handled by sending configuration.

| Item | Lead implementation | Assessment |
|---|---|---|
| Bus | FDCAN2, standard IDs, 500 kbit/s Classical CAN | Correct documented default; actual sensor stored bitrate/IDs must match. Supported datasheet bitrates are 250k/500k/1M. No automatic bitrate discovery. |
| Results | I 521/mux00, U1 522/mux01, U2 523/mux02, Wh 528/mux07 | Correct default IDs/mux and signed units: mA, mV, mV, Wh. |
| Byte order/sign | Signed int32 from big-endian DB2-5; optional vehicle sign applied in `GetReading` | Default endian matches the datasheet. Actual negative-current mounting convention remains a hardware test. INT32_MIN negation is saturated. The cast from uint32 bit pattern to int32 relies on selected compiler behavior; earlier driver explicitly avoided that implementation-defined conversion. |
| DLC | Rejects lengths <6, accepts 6/7/8; response likewise accepts >=6 | Result DLC should be exactly 6; response DLC should be 8. Confirmed gap. |
| Result state | DB1 high nibble stored privately; mask 0x0A treats bits1/3 as error, bit0 OCS not used; bit2 deliberately ignored because unused U3 may set it | Bit1 includes range/reduced precision/error; bit3 means sensor functionality not ensured. Bit2 handling is documented but requires team acceptance and diagnostic exposure; it is not a clean status. Raw state/counter are not exposed by the public reading. |
| Counter | DB1 low nibble unused | No duplicate/gap/rollover check or diagnostic. Replayed values can refresh age indefinitely. |
| Freshness | I 100 ms, U1/U2 250 ms, Wh 300 ms | Independent received flags/timestamps; `Fresh` means received recently, not error-free. AMS missing-I/U1 timeout is separately 450 ms. No fresh/error-free snapshot flag per result. |
| Error propagation | I/U1 state OR -> sensor/system error; U2 own error; IVT safety uses I/U1, precharge uses U2 | Numeric values may stay fresh with an error bit. Inverter Update uses freshness, not sensorError, while the AMS debounce is pending. Fast frames likewise encode fresh raw values without their own error field; consumers also need status. U2 loss/error is not a standalone AMS fault outside precharge. Wh's stored state is not represented by an energy-error flag. |
| Startup/restart | Wait configured startup budget, STOP -> eight result configurations -> RUN; 3 ms command spacing, no STORE | Explicitly enables I/U1/U2 each4ms plus Wh100ms: 760 result messages/s, below documented1000/s cap. It does not read back every setting/response. CAN acknowledgement/arrival timing is not configuration verification. |
| Alive | 511/DB0 BF sets pending; service reconfigures when no fresh current, ignores Alive while current is flowing as spurious | Different deliberate policy from earlier driver, which invalidates caches immediately. No Alive serial/counter diagnostic is exposed. Mixed-generation/restart trust policy needs agreement. |
| Error recovery | Persistent bit3 -> wait for clean bus -> query errors/serial; auto-clear only CAN-only mask, max3 times; non-CAN errors retained | New production policy requiring approval, especially because physical manual fault latching is unproven. Commands cannot clear non-CAN errors automatically. |

Required hardware capability: software names `IVT-S-500-U3-I-CAN2-12/24`, but a comment is not a part-number inspection. **U0 variants have no voltage inputs and cannot support this precharge design.** U3 variants have three voltage channels; at least U1/U2 must actually be fitted and connected. The part-number suffix `CAN2` means **no onboard CAN termination**, not STM32 peripheral selection; fit the approved end-of-bus termination accordingly.

Required measurement meaning: U1 must represent accumulator voltage; U2 must represent the vehicle DC-link voltage during the proposed AIR/precharge sequence. Datasheet p16 says both pins of each U connector are internally shorted, not a differential pair; each voltage is referenced to the HV module ground. Do not connect a pack across the two pins of one connector. Confirm shunt installation/reference, HV/LV isolation, positive/negative AIR sequence and whether U2 actually measures the DC link when contactors are open. A common HV reference or externally controlled negative AIR cannot be assumed from software.

For the documented LV connector, X1/X2 pin4=Vcc, pin3=CAN-L, pin2=CAN-H, pin1=GND; verify the actual variant/harness. No physical wiring is approved by this report.

## H. Inverter integration

[L inverter][L-inverter] sends only DTI **maximum DC current** packet0x0A and **maximum DC brake current** packet0x0B, every20ms over TSCAN. With node `0xFF` (explicit TBD/broadcast), extended IDs are **0xAFF / 0xBFF**. Bytes0-1 are signed int16 big-endian, 0.1A/count; remaining bytes FF. Regen is transmitted negative. [DTI manual v2.4][dti-pdf] pp17-22,28-29 supports those command types/scaling.

**The BMS does not request driver torque, AC drive current, speed, position or drive enable.** It does not send DTI control packets0x01-0x06 or0x0C. The VCU must own propulsion/drive-enable commands. No DTI receive/status decoder exists in the BMS; those frames are filtered out, so actual inverter node/configuration and successful application of limits are not verified through feedback.

The default drive ceiling is 200A. Above55C hottest-cell temperature, it ramps to25A at60C. Default regen is35A, ramping from45C to25A at60C. Runtime settings can only lower the built-in current/power/temperature ceilings or disable regen.

The actual power calculation is more than `80000/U1`: `currentForPower()` uses `U = OCV - R*I`, chooses Rmax=0.6ohm when reducing current and Rmin=0.15ohm when increasing it, and solves the smaller quadratic root for the requested power. Drive current is the minimum of that result and the thermally derated current ceiling. Therefore the80kW guarantee depends on an **unverified pack-resistance interval**, IVT timing and inverter response; it is not proven merely by the80kW constant. Without fresh U1 it substitutes470.4V; without fresh current it substitutes0A. Confirmed AMS fault makes both limits zero on the next limit update (up to20ms after fault evaluation); relay shutdown is separate.

The500ms moving-window boost limiter is **off by default**. Its enabled test variant has a documented excluded random-case/floor problem; the explicit audit rerun is recorded in B. Do not enable it as part of integration.

**Thermal/current blocker:** Enepaq specifies charge0..45C. The lead permits regen35A at-10C, approximately31.67A at50C and25A at60C; its general temperature window switches to charge limits only when the external-charger flag is set, not when current is regenerative. Hottest-cell-only derating also cannot prove every cell is above the minimum charge temperature. The200A charge/regen AMS trip is inconsistent with the35A fast-charge rating if the assumed7p module is correct. No supplied pulse-charge exception approves either behavior.

Team must fix/approve node address (broadcast versus specific inverter), HV500 firmware/mapV24, extended IDs, inverter CAN2 port at1Mbit/s, `Drive enable via CAN2`, DC-limit behavior and command timeout. The manual requires control messages faster than half the configured timeout. Specifically test loss of **BMS limit frames while VCU torque messages continue**: a generic inverter control timeout may be refreshed by the VCU and does not prove old BMS limits expire.

## I. Precharge

```text
VCU extended B300 byte0=1, rising edge (ID TBD)
 -> BMS_Precharge_HandleRequest
 -> safety tick consumes edge; require AMS fault output clear + SDC closed
 -> CHARGING: PC6 high, PC9 low
 -> require fresh/error-free plausible U1 (280..470.4V), fresh/error-free U2
 -> U2 >= 95% U1: CLOSING, PC6 and PC9 high
 -> 50ms overlap: ACTIVE, PC6 low / PC9 high
 -> deactivate, AMS fault or SDC open: both low
 -> missing/implausible voltages or >5s while charging: FAILED, both low; new edge required
```

The initial IDLE->CHARGING transition checks AMS/SDC, not U1/U2; voltage validity is evaluated on the following tick. CLOSING/ACTIVE do not independently recheck U2 freshness; I/U1 AMS monitoring remains active. U2 failure matters during CHARGING but is not a global IVT fault. Precharge failure is a state/telemetry result, **not a dedicated AMS fault bit**. The ratio can remain the last computed value when inputs disappear; state/freshness must also be interpreted.

`CFG_TS_REQUEST_TIMEOUT_MS=0` disables request freshness: one activate can persist indefinitely until deactivate/fault/SDC. Repeated active requests do not auto-retry a failed sequence. A TS request while faulted is consumed/ignored; a new0->1 edge is required after recovery.

**Ownership is not established.** The supplied master has neither PC6/PC9 relay drivers nor PC8 ready driver, and there is no complete AIR/precharge circuit, negative-AIR sequence, auxiliary contact feedback or weld detection. Software outputs cannot close actual relays without a verified external board/harness. Do not integrate this state machine with real contactors until the team explicitly assigns ownership and supplies the circuit.

## J. Charging

The BMS uses Elcon/TC-style extended IDs1806E5F4(command) /18FF50E5(status), 250kbit/s on FDCAN3. Actual packing is **big-endian** 16-bit voltage/current in0.1V/0.1A and byte4 `1=disable,0=enable`; comments incorrectly describe little-endian/enable polarity and byte5 status. The implemented parser reads flags from byte4.

Default charge setpoints:1A and112x4.02=450.24V. GUI limits allow35A and112x(4.2-0.05)=464.8V. The PC0 button toggles charging only after SDC closed, no AMS fault and cached charger output >=300V. Charger hardware/temperature/input fault checks in the enable decision are commented out. There is **no charger-status timestamp, timeout or reliable acknowledgement of disable**. The RX parser does not require a minimum DLC before reading bytes0-4, so short frames can use uninitialized/old stack contents. These are software blockers for charging, independent of the bus hardware blocker.

The archive's **new charger references are incompatible** with that interface:

| Reference | Implemented communication |
|---|---|
| `Charger Docs/g474-psfb-main/...` HV stage | UART interface to the LV/display controller; no FDCAN interface found in its Core sources. |
| [LV charger/display main][charger-main] + [FDCAN config][charger-fdcan] | 500kbit/s (170MHz /17/20, also confirmed in its archived .ioc). RX extended00FFFFFF(CC/CV),001FFFFF(BMS fault heartbeat),00000001(discharge). TX01000000,02000000,01100000 telemetry/status. |
| LV/display data decoder | CC/CV unsigned24-bit little-endian fields; voltage factor0.0001V, current implementation0.000001A despite an inconsistent nearby comment. Its BMS-fault heartbeat drives its CAN-health counter. |

Do not assume this is the assembled charger; identify the actual charger, LV gateway firmware and authoritative protocol. A BMS still sending Elcon frames will not control the supplied display/gateway protocol. The charger has its own power-stage precharge state; that does not establish BMS ownership of vehicle precharge.

## K. Balancing

Default `CFG_BALANCING_ENABLED=1`, not restricted to charging; balance threshold lowest cell+30mV, hysteresis10mV. No balance while the complete cycle reports any limit fault or the AMS output is active. Temperature interlock uses the same conversion/mapping and mode-dependent limits discussed above; it is not independently hardware-qualified.

For each board, the code sums `V^2 /15ohm` of selected cells, chooses PWM duty0..15, and floors duty proportionally to keep the configured board budget <=4W. A single selected cell can receive **15/15=100%**. At4.2V, one resistor dissipates1.176W; eight would total9.408W, reducing the calculated common duty to6/15 (~3.76W total). This arithmetic is not a validated thermal rating.

Only configured C1-C8 are selected for discharge. PWM A maps channels1-12, B13-16, low cell in low nibble; stop explicitly zeros all16 PWM positions and DCTO. Unused PWM entries remain zero through reachable normal code (static initialization/stop, no public setter for unused positions). CFB's DCC field starts at zero and is not enabled. There is no independent hardware readback/mask enforcement on every possible error path.

`bms_startDischarge()` ignores its three register-write return values and sets active=true. `bms_stopDischarge()` does check all writes and retains active=true on failure for a later retry. A failed measurement cycle skips the balancing decision; communication recovery resets after three failures, but a physically broken link may not carry a stop/reset. The slave discharge timeout is **one minute**, not the500ms AMS reaction budget. An MCU reset/error also does not prove an immediate stop command reached every board.

This conflicts with the earlier migration's bring-up lock (enabled=false, PWM cap/budget0 until confirmation). **Do not silently inherit enabled balancing,15ohm or4W as approved.** Resolve resistor BOM, per-resistor pulse/continuous rating, board heat spreading, cell/sensor placement, timeout policy and maximum allowed duty experimentally before accumulator use.

## L. Safety/shutdown

Flags: COMMS01, VOLTAGE02, TEMP04, CURRENT08, OPEN_WIRE10, IVT20, STARTUP40. There is no generic CAN, precharge, IMD or internal-software-error bit. Logical fault output starts asserted; complete healthy data and a1000ms clear interval are required before release.

| Source -> detector | Persistence / fault state | Charging / balancing / inverter / physical output |
|---|---|---|
| Cell OV/UV -> `bms_evaluateLimits` | VOLTAGE; per-bit over-limit accumulator180ms, drains at half rate under limit; deadline470ms since last clear sample | Common confirmed-fault action below. |
| Board VPV, IVT U1 pack OV/UV | VOLTAGE; ADBMS sample timer / separate IVT timer respectively | Same. Board uses8, pack112 positions. |
| Temperature out of window or sentinel888; die out of range | TEMP; same accumulator/deadline (also held to500ms policy) | Same. External-charge flag selects0..45 instead of-20..60; regen does not select charge window. |
| I >200A or I <-200A -> `evaluateIvt` | CURRENT; same persistence/deadline | Same; threshold needs correction/approval relative to cell rating. |
| No complete ADBMS sample since startup | STARTUP immediately | Fault output held from startup. |
| Missing cells/temperatures, SPI error, bad PEC, bad CC, missing board -> unsuccessful complete cycle | Does not refresh safety sample; COMMS when age>450ms. Three failures request reset/reinit. Failed-cycle telemetry may be partial. | Initial ride-through is intentional; balancing stop is not guaranteed while link is broken. |
| IVT I or U1 absent | IVT when channel age>450ms; control/telemetry freshness thresholds100/250ms are separate | Common action after fault; inverter uses substitutes during the freshness-to-AMS interval. |
| IVT bit1/3 error | IVT after debounce/deadline; numeric data can remain fresh | Same; CAN-only sensor-error auto-clear policy can subsequently clear software fault. |
| IVT U2 missing/error | Precharge fails during CHARGING; no standalone AMS flag | Opens its precharge/bypass outputs; no generic AMS fault; after ACTIVE, U2 loss alone does not open the state machine. |
| Open cell wire -> even/odd ratio | OPEN_WIRE; debounced/deadline as above | Common action; threshold/filter behavior hardware-dependent. |
| U1-versus-cell-sum difference >20V | VOLTAGE, only compared when averaged U1 is usable and current range <=20A over the conversion | If a comparison cannot be made, prior plausibility result is held, not cleared; prolonged dynamic-load exemption needs review. |
| CAN1/CAN3 no-ack, dropped TX, bus-off | Counters/recovery, **no generic AMS flag** | Reliance on VCU/inverter/charger timeout policy; those configurations are not proven. CAN2 loss eventually becomes IVT fault. |
| Precharge timeout/bad input | FAILED state, no AMS bit | Its relay outputs off; new activation edge needed. Does not inherently command a generic AMS fault or zero inverter limits. |
| `Error_Handler`, HardFault/MemManage/BusFault/UsageFault/NMI paths | Force logical fault/relay outputs; Error_Handler starts IWDG, but the other trap handlers rely on it already running | An early CPU fault before watchdog startup can hang without automatic reset. No explicit ADBMS stop in force-fault; physical discharge depends on prior slave state/timeout. Simulation replaces Error_Handler and does not execute the real fatal reset path. |
| Main-loop hang | IWDG nominal200ms, actual LSI-dependent; safety IRQ can separately detect stale sample if interrupts run | IWDG reset relies on actual reset-safe outputs and external hardware. Early startup delays occur before watchdog activation. |

Common confirmed-fault sequence: safety tick first drives PC9/PC6/PC8 off, then asserts logical PC1 fault. Precharge tick follows in the same IRQ. Inverter limits become zero at the next20ms update; CAN delivery is not guaranteed merely because the frame was queued. Charging is disabled by the foreground loop; balancing is stopped by a subsequent successful acquisition/policy pass or reset, subject to communication success. These paths have different latency and failure guarantees.

The software fault releases automatically after clear conditions for1000ms. **Manual/non-programmable latching is external to this code and absent from the supplied sheet.** The controller's Q1 inverts the signal. Verify external latch circuitry and actual SDC opening; do not equate a simulated PC1 write with proven AIR opening. Force-fault writes during very early initialization also do not prove that GPIO clocks/modes have already been configured; reset-safe behavior must come from the approved external circuit.

## M. Remaining TBD parameters

Classification meanings: VERIFIED = checked software arithmetic/layout or user-confirmed logical topology, **not hardware approval**; DATASHEET-DERIVED = traceable nominal value/protocol in the supplied source, still conditional on fitted hardware; RULE-DERIVED = sourced from the rule set named by the code, applicability unapproved; TEAM DECISION REQUIRED / HARDWARE TEST REQUIRED / LEGACY / UNKNOWN remain unresolved.

Key decisions include the physical TEMP/LED mapping, sensor model/bias, 7p module assumption, charge/regen trip and temperature window, enabled balancing/thermal budget, three physical CAN buses and charger protocol, IVT U3 variant/wiring/reference, output/SDC/latch truth table, precharge owner/negative AIR, inverter node/configuration/timeout, resistance bounds and control latency. Full per-constant classification is in the appendix below; no legacy value was silently approved.

The repository cites **FS Rules2027 v1.0**, while the project/request is2026 and the archive supplies only an FSUK2026 supplementary rules PDF alongside2027 base rules. The applicable event/year/base/supplementary rule set must be frozen by the team. The500ms/95%/80kW checks in code are not a complete2026 compliance assessment.

## N. Hardware blockers

1. Missing authoritative V3/as-built slave schematic; seven temperature mappings conflict with the user's confirmed requirement, and LED outputs overlap the reported temperature pins.
2. Supplied master has two transceivers, not three; its second connector is labelled charger, while firmware assigns it to IVT. FDCAN3 has no shown transceiver. V-variant VIO wiring is also unresolved.
3. No ready/precharge/AIR drivers or approved ownership/negative-AIR sequence. No complete hardware latch/SDC truth table or safe supply-loss proof; PC2 is drawn as an old OC input.
4. Actual IVT part/voltage capability, HV references, connector wiring, termination and polarity unverified.
5. Charger/display reference is500kbit/s with non-Elcon IDs/protocol; actual installed charger/gateway must be identified.
6. Module/parallel count, charge/regen current and temperature rating, balancing resistance/thermal budget and power-model resistance interval require approval/measurement.
7. Inverter node, CAN2 map/bitrate/drive-enable/timeout settings and behavior on loss of BMS-only limits require confirmation.

## O. Software blockers

1. IVT exact-DLC validation, per-channel counter sequence/replay handling and exposed status/validity/configuration trust are weaker than the earlier migration; audit probes demonstrate the first two gaps.
2. Acquisition/publication is not fully transactional, and raw/Celsius validity is not represented separately. Failed late groups can publish mixed measurement generations.
3. Initial poll-status propagation for14 devices is not explicitly handled/modelled; bounded timer delay/context/timeout contracts are incomplete. A non-FF result alone is not sufficient evidence of a polling defect.
4. Regen is allowed outside the supplied charge temperature window; external-charge-only protection mode and200A charge trip require a resolved safety requirement, not just a warning cleanup.
5. Balancing is enabled without the prior confirmation lock; start-write errors are ignored; stop/error behavior is not guaranteed with a lost link. Do not merge these policy changes silently.
6. Charger parser/protocol documentation disagree; short-DLC parsing, status freshness, commented error checks and acknowledged disable behavior need explicit contracts after the real charger interface is selected.
7. Precharge timeout/ownership/request-loss/U2-active-state behavior and lack of dedicated fault must be accepted or corrected against a hardware-level specification.
8. Target IRQ/WCET, early watchdog coverage, actual fatal-handler reset behavior, TIM15 IMD capture range/role and the70C die precision boundary remain unqualified. The simulator does not measure CPU execution time.
9. Enabled moving-window limiter has an excluded known random/floor case. Keep it disabled until the measured failure is resolved and the variant suite includes it.

## P. Recommended next coding tasks and comparison

**No merge is recommended solely because either suite is green.** Choose contracts area by area after resolving the hardware interfaces.

| Major area | Classification | Reason |
|---|---|---|
| 14x8 logical topology, six-byte register payloads, no2950 offset | EQUIVALENT | Both represent112 populated positions correctly and distinguish physical register capacity. |
| Temperature pin assignment | CONFLICT / NEEDS HARDWARE DECISION | Earlier migration matches the user's confirmed V3 order; lead uses1..8. No authoritative V3 file is available here. |
| Celsius conversion and sensor approval | NEEDS HARDWARE DECISION | Lead activates an Enepaq-specific table; earlier migration retains separate raw validity and disabled Celsius. Correct table provenance is not proof of fitted sensor/bias. |
| Measurement validity/publication | MY VERSION BETTER | Explicit independent freshness/validity, complete pending snapshot publication and failed-acquisition invalidation. Lead CAN double buffering does not provide the same transactional acquisition semantics. |
| ADC numerical boundary qualification | MY VERSION BETTER | Earlier P1 work exhaustively compared all raw codes; lead70C calculation produces a different threshold decision. New thresholds themselves remain team decisions. |
| PEC and expected ADBMS CC | EQUIVALENT for present broadcast chain; MY VERSION BETTER for explicit state/error contracts | Lead checks every read/per-device and resets each cycle; earlier module explicitly classifies operations, handles ambiguous sends and supports independent chain instances with latched diagnostics. |
| Full-chain poll-status priming | CONFLICT / NEEDS HARDWARE DECISION | Both need a p55 transaction-level review; neither byte predicate alone proves initial status validity. |
| Open wire and slave-reset recovery | LEAD VERSION BETTER | Implements normal/even/odd OW and REFON reset detection/restart, with simulation; earlier migration lacks equivalent end-to-end OW functionality. Physical ratio still unapproved. |
| IVT strict receive validation/counters | MY VERSION BETTER | Exact DLC, counter/replay diagnostics, raw status, independent validity and explicit transport/restart invalidation. |
| IVT U2, Wh, explicit configuration and managed recovery | LEAD VERSION BETTER in functionality / NEEDS HARDWARE DECISION in policy | Supports new precharge/energy needs and startup-default inconsistency; does not replace strict receive validation or readback verification. |
| CAN bus assignment, schema and vehicle rate | CONFLICT / NEEDS HARDWARE DECISION | Lead3 buses1M/500k/250k and packed messages versus migration2 buses250k/500k and130 frames. Master drawing only shows2 transceivers. |
| Safety scheduling/watchdog/persistence | LEAD VERSION BETTER in implemented supervision / CONFLICT in failure policy | Independent1/10ms scheduler, decay/deadline checks and watchdog are useful; intentional450ms ride-through versus earlier immediate invalidation must be explicitly specified. Earlier1000ms freshness and layered legacy delays are not approved for competition. |
| Initial balancing inhibition | MY VERSION BETTER for unapproved bring-up | Earlier cap/budget0 and confirmation lock prevent accidental heat. Lead active balancing needs thermal approval before acceptance. |
| Balancing algorithm/heat calculation | LEAD VERSION BETTER in functionality / NEEDS HARDWARE DECISION for limits | Hysteresis and V-squared heat budget exist, but15ohm/4W/100%-possible duty are not approved. |
| Inverter/precharge | LEAD VERSION BETTER in implemented functionality / NEEDS HARDWARE DECISION | Earlier version does not implement these subsystems; lead has current-limit-only commands, but policy/ownership/wiring must be corrected/confirmed. |
| Charging | CONFLICT / NEEDS HARDWARE DECISION | Both carry inherited Elcon-style assumptions; supplied new charger references differ. No version is approved for the actual charger. |
| SoC/settings/GUI/generated DBCs | LEAD VERSION BETTER in functionality | Spreadsheet correlation, saved settings, GUI and code-generated databases exist; they do not validate physical capacity or the safety policy. |
| Tests | LEAD VERSION BETTER in breadth; MY VERSION BETTER for some strict contracts | Lead drives real startup/main with soaks/variants; earlier suites target invalidity, replay and transactional behavior more explicitly. Retain complementary tests and add the uncovered contracts. |

Recommended coding order after decisions: freeze interfaces/configuration -> correct chosen TEMP/LED and bus/output mappings -> strict IVT receive/state contracts -> transactional ADBMS publication and poll propagation model -> explicit balancing/charging stop/error contracts -> regen/charge safety policy and dedicated tests -> precharge ownership/U2/request/failure contracts -> target timing/error-path qualification -> combined full regression. Do not broaden this into an unreviewed rewrite.

## Q. Questions that must be answered by the team lead

1. Which exact master, slave V3 and HAT schematic/BOM revisions are the assembled hardware? Supply the authoritative pin/net exports, not only a90%-complete draft.
2. Is the user-confirmed TEMP GPIO order still authoritative? Which physical LEDs/colors use GPIO1/2? Which sensor belongs to each logical cell-group position?
3. Are there actually three physical CAN transceivers? Where is CAN3, and what are its connector, RX/TX nets, VIO supply and termination? Is the existing second connector repurposed from charger to IVT?
4. Which TCAN1044 variant is fitted, and is pin5 supplied on the V variant?
5. Which exact IVT part is fitted? Does it have U3 capability, U1/U2 wired to the required references, the expected IDs/500kbit/s configuration and correct end termination? Which mounting direction means discharge?
6. Who owns positive/negative AIR and precharge? Where are PC6/PC8/PC9 drivers, pull-downs, auxiliary feedback and weld detection? Can U2 measure the actual DC link throughout the contactor sequence?
7. Where is the non-programmable AMS latch? What levels on PC1/FAULT_OUT mean fault, and what happens on MCU reset,5V loss,12V loss, broken wiring and stuck firmware? What exactly drives PC2?
8. Is the pack actually112s7p/Enepaq Li1x7pVTC6? Who approves cell UV, maximum charge/regen current, discharge current and the charge/discharge/regen thermal windows?
9. Is regen outside0..45C intended, and what manufacturer evidence permits it? Why is the AMS charge trip200A when the cited7p fast-charge rating is35A?
10. May balancing be enabled for first bring-up? What validates15ohm,4W/board, maximum per-resistor duty and the one-minute loss-of-link discharge timeout?
11. Which charger and LV gateway firmware/protocol is installed: Elcon/TC250k or the supplied custom500k display/UART gateway? What are the authoritative IDs, units, heartbeat/disable/fault semantics?
12. What are the approved VCU TS-request ID, timeout and retry/edge semantics? Must U2 loss after ACTIVE open the AIRs? Is precharge failure an AMS fault or a separate vehicle fault?
13. Which HV500 node(s), CAN2 map/bitrate/extended-ID settings and drive-enable/timeout configuration are approved? What happens if BMS limits stop but VCU commands continue?
14. Who approves the0.15..0.6ohm power-model bounds, inverter lag and80kW margin to the actual logger? Must boost remain disabled?
15. Is450ms measurement-loss ride-through and180ms accumulated violation policy accepted, including automatic software clearing and CAN-only IVT error clearing? How does that combine with the physical latch?
16. Which event/year rule set applies to this2026 fork? Are2027 references intentional, and where is the authoritative full2026 base/supplement?
17. What physical timing/EMC/open-wire/thermal/fault-latch evidence already exists, with exact firmware/configuration hashes? Which README bench claims can be independently reproduced?
18. Which commit/repository is the agreed integration baseline? The lead ZIP has no remote; the migration contains substantial uncommitted work. No merge should obscure the provenance or discard either implementation.

## Appendix: every configuration value, test inventory and source evidence

184 unique central definitions plus internal protocol/timing/buffer values and hard-coded operating values are listed (221 rows). Conditional local-override definitions are consolidated to the normal0/override1 behavior. Fixed command opcodes/bitfields are protocol definitions covered by the register/command tests, not unlisted pack protection setpoints.

[CSV inventory](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/configuration-inventory.csv>) / [JSON](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/configuration-inventory.json>).

| Parameter | Value / expression | Classification | Basis / required approval | Source |
|---|---|---|---|---|
| `CFG_NUM_SLAVES` | `14` | VERIFIED | 14x8/112 logical topology; physical population still needs as-built evidence. | [30](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:30>) |
| `CFG_CELLS_PER_SLAVE` | `8` | VERIFIED | 14x8/112 logical topology; physical population still needs as-built evidence. | [31](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:31>) |
| `CFG_TEMPS_PER_SLAVE` | `8` | VERIFIED | 14x8/112 logical topology; physical population still needs as-built evidence. | [32](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:32>) |
| `CFG_TOTAL_CELLS` | `(CFG_NUM_SLAVES * CFG_CELLS_PER_SLAVE)` | VERIFIED | 14x8/112 logical topology; physical population still needs as-built evidence. | [34](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:34>) |
| `TOTAL_IC` | `CFG_NUM_SLAVES` | VERIFIED | 14x8/112 logical topology; physical population still needs as-built evidence. | [35](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:35>) |
| `CFG_SLAVE_CELL_CHANNELS` | `{ 1, 2, 3, 4, 5, 6, 7, 8 }` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [40](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:40>) |
| `CFG_SLAVE_TEMP_GPIOS` | `{ 1, 2, 3, 4, 5, 6, 7, 8 }` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [48](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:48>) |
| `CFG_SLAVE_LED_RED_GPIO` | `9` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [49](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:49>) |
| `CFG_SLAVE_LED_GREEN_GPIO` | `10` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [50](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:50>) |
| `CFG_SLAVE_LED_ACTIVE_LOW` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [54](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:54>) |
| `CFG_TEMP_TABLE_C` | `{ -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5,   10,   15,   20,   25,   30,   35,   40,  45,   50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100,  105,  110,  115,  120 }` | DATASHEET-DERIVED | Enepaq Table5; only valid for the fitted specified sensor/bias, not generic thermistors. | [63](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:63>) |
| `CFG_TEMP_TABLE_V` | `{ 2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68,  1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30 }` | DATASHEET-DERIVED | Enepaq Table5; only valid for the fitted specified sensor/bias, not generic thermistors. | [65](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:65>) |
| `CFG_TEMP_INVALID_C` | `888.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [68](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:68>) |
| `CFG_CELL_UV_V` | `2.50f` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [75](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:75>) |
| `CFG_CELL_OV_V` | `4.20f` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [76](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:76>) |
| `CFG_CELL_TEMP_MIN_C` | `(-20.0f)` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [79](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:79>) |
| `CFG_CELL_TEMP_MAX_C` | `60.0f` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [80](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:80>) |
| `CFG_CELL_CHARGE_TEMP_MIN_C` | `0.0f` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [83](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:83>) |
| `CFG_CELL_CHARGE_TEMP_MAX_C` | `45.0f` | DATASHEET-DERIVED | Enepaq Tables1/2; operating selection and actual module still need approval. | [84](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:84>) |
| `CFG_SLAVE_DIE_TEMP_MAX_C` | `70.0f` | LEGACY / UNKNOWN | Retained/chosen value; no signed engineering approval or physical validation supplied. | [87](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:87>) |
| `CFG_SLAVE_DIE_TEMP_MIN_C` | `(-40.0f)` | LEGACY / UNKNOWN | Retained/chosen value; no signed engineering approval or physical validation supplied. | [88](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:88>) |
| `CFG_SLAVE_V_MAX` | `(CFG_CELL_OV_V * CFG_CELLS_PER_SLAVE)` | DATASHEET-DERIVED | Derived from selected cell limits x8/x112: board20..33.6V, pack280..470.4V. | [89](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:89>) |
| `CFG_SLAVE_V_MIN` | `(CFG_CELL_UV_V * CFG_CELLS_PER_SLAVE)` | DATASHEET-DERIVED | Derived from selected cell limits x8/x112: board20..33.6V, pack280..470.4V. | [90](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:90>) |
| `CFG_MAX_DISCHARGE_CURRENT_A` | `200.0f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [97](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:97>) |
| `CFG_MAX_CHARGE_CURRENT_A` | `200.0f` | LEGACY / UNKNOWN | No final approved engineering basis; 200A charge trip conflicts with assumed7p/35A rating. | [99](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:99>) |
| `CFG_PACK_V_MAX` | `(CFG_CELL_OV_V * CFG_TOTAL_CELLS)` | DATASHEET-DERIVED | Derived from selected cell limits x8/x112: board20..33.6V, pack280..470.4V. | [101](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:101>) |
| `CFG_PACK_V_MIN` | `(CFG_CELL_UV_V * CFG_TOTAL_CELLS)` | DATASHEET-DERIVED | Derived from selected cell limits x8/x112: board20..33.6V, pack280..470.4V. | [102](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:102>) |
| `CFG_PACK_V_PLAUSIBILITY_V` | `20.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [108](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:108>) |
| `CFG_PACK_V_PLAUSIBILITY_MAX_DI_A` | `20.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [109](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:109>) |
| `CFG_FS_MAX_REACTION_MS` | `500` | RULE-DERIVED | Code cites FS2027; event/year applicability and physical margins need team approval. | [148](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:148>) |
| `CFG_FS_MARGIN_MS` | `20` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [149](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:149>) |
| `CFG_TIMER_TICK_MS` | `1` | VERIFIED | TIM16 register/config arithmetic1ms at configured128MHz; physical clock/WCET still unmeasured. | [151](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:151>) |
| `CFG_SAFETY_TICK_MS` | `10` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [152](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:152>) |
| `CFG_FAULT_DEADLINE_MS` | `(CFG_FS_MAX_REACTION_MS - CFG_FS_MARGIN_MS - CFG_SAFETY_TICK_MS)` | RULE-DERIVED | Code cites FS2027; event/year applicability and physical margins need team approval. | [153](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:153>) |
| `CFG_MEAS_PERIOD_MS` | `100` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [154](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:154>) |
| `CFG_MEAS_CYCLE_MAX_MS` | `70` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [155](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:155>) |
| `CFG_MEAS_STALE_TIMEOUT_MS` | `450` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [156](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:156>) |
| `CFG_FAULT_DEBOUNCE_MS` | `180` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [157](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:157>) |
| `CFG_FAULT_DECAY_DIVISOR` | `2` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [158](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:158>) |
| `CFG_FAULT_CLEAR_DELAY_MS` | `1000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [159](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:159>) |
| `CFG_COMMS_RETRIES` | `3` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [160](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:160>) |
| `CFG_COMMS_RETRY_DELAY_MS` | `10` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [161](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:161>) |
| `CFG_CHECK_COMMAND_COUNTER` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [162](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:162>) |
| `CFG_WATCHDOG_TIMEOUT_MS` | `200` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [164](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:164>) |
| `CFG_ISOSPI_WAKE_PULSE_US` | `20` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [171](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:171>) |
| `CFG_ISOSPI_WAKE_DELAY_US` | `600` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [172](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:172>) |
| `CFG_SPI_TIMEOUT_MS` | `10` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [173](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:173>) |
| `CFG_SADC_POLL_TIMEOUT_MS` | `15` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [174](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:174>) |
| `CFG_AUX_POLL_TIMEOUT_MS` | `30` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [175](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:175>) |
| `CFG_OPEN_WIRE_CHECK_ENABLED` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [182](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:182>) |
| `CFG_OPEN_WIRE_MIN_RATIO` | `0.75f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [187](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:187>) |
| `CFG_ADSV_OW_EVEN` | `0b01` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [189](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:189>) |
| `CFG_ADSV_OW_ODD` | `0b10` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [190](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:190>) |
| `CFG_BALANCING_ENABLED` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [197](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:197>) |
| `CFG_BALANCE_ONLY_WHILE_CHARGING` | `0` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [198](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:198>) |
| `CFG_BALANCE_THRESHOLD_V` | `0.030f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [199](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:199>) |
| `CFG_BALANCE_HYSTERESIS_V` | `0.010f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [200](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:200>) |
| `CFG_BALANCE_RESISTOR_OHM` | `15.0f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [201](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:201>) |
| `CFG_BALANCE_MAX_POWER_PER_SLAVE_W` | `4.0f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [204](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:204>) |
| `CFG_BALANCE_DCTO_MIN` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [205](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:205>) |
| `CFG_IVT_ID_COMMAND` | `0x411` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [216](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:216>) |
| `CFG_IVT_ID_RESPONSE` | `0x511` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [217](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:217>) |
| `CFG_IVT_ID_RESULT_I` | `0x521` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [218](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:218>) |
| `CFG_IVT_ID_RESULT_U1` | `0x522` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [219](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:219>) |
| `CFG_IVT_ID_RESULT_U2` | `0x523` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [220](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:220>) |
| `CFG_IVT_ID_RESULT_WH` | `0x528` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [221](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:221>) |
| `CFG_IVT_CURRENT_DISCHARGE_POSITIVE` | `1` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [225](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:225>) |
| `CFG_IVT_CONFIGURE_ON_BOOT` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [230](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:230>) |
| `CFG_IVT_CURRENT_PERIOD_MS` | `4` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [239](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:239>) |
| `CFG_IVT_U1_PERIOD_MS` | `4` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [240](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:240>) |
| `CFG_IVT_U2_PERIOD_MS` | `4` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [241](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:241>) |
| `CFG_IVT_ENERGY_PERIOD_MS` | `100` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [242](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:242>) |
| `CFG_IVT_MAX_MESSAGES_PER_S` | `1000` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [243](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:243>) |
| `CFG_IVT_STARTUP_MS` | `400` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [244](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:244>) |
| `CFG_IVT_RESTART_WAIT_MS` | `300` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [248](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:248>) |
| `CFG_IVT_RETRY_INTERVAL_MS` | `500` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [249](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:249>) |
| `CFG_IVT_AUTO_CLEAR_MAX` | `3` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [252](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:252>) |
| `CFG_IVT_RECONFIG_INTERVAL_MS` | `2000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [253](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:253>) |
| `CFG_IVT_CURRENT_TIMEOUT_MS` | `100` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [257](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:257>) |
| `CFG_IVT_VOLTAGE_TIMEOUT_MS` | `250` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [258](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:258>) |
| `CFG_IVT_FAULT_TIMEOUT_MS` | `450` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [260](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:260>) |
| `CFG_IVT_STATUS_FAULT_MASK` | `0x0A` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [265](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:265>) |
| `CFG_CELL_ENERGY_WH` | `75.6f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [282](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:282>) |
| `CFG_PACK_ENERGY_WH` | `(CFG_CELL_ENERGY_WH * CFG_TOTAL_CELLS)` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [283](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:283>) |
| `CFG_SOC_VMIN_V` | `2.8f` | VERIFIED | Matches tracked Battery Tracker fit/model and test; not physical capacity/SoC validation. | [285](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:285>) |
| `CFG_SOC_VMAX_V` | `4.09959016393443f` | VERIFIED | Matches tracked Battery Tracker fit/model and test; not physical capacity/SoC validation. | [286](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:286>) |
| `CFG_SOC_COEFFS` | `{  0.423004261244546f,     0.49948859673559f,    0.120958931657348f,   -0.0112915923908232f,  -0.0135757684176135f,   -0.00150121928350042f, -0.00413216758222435f, -0.0104979259889331f,   -0.00635341354977716f,  -0.000801816990438799f, -0.00226824735787767f, -6.36450868561571e-06f,  0.00450625289432844f,   0.00602363597183047f,  0.00231370791940568f, -0.000704525643415416f,  -0.00224505625907345f,  -0.00204305060329757f, -0.00158222115306415f, -0.000992284085613443f,  0.000420170472073563f,  0.000711803275347776f, 0.000678935305282581f }` | VERIFIED | Matches tracked Battery Tracker fit/model and test; not physical capacity/SoC validation. | [287](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:287>) |
| `CFG_SOC_CELL_MIN_V` | `2.8f` | VERIFIED | Matches tracked Battery Tracker fit/model and test; not physical capacity/SoC validation. | [293](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:293>) |
| `CFG_SOC_CELL_MAX_V` | `4.1f` | VERIFIED | Matches tracked Battery Tracker fit/model and test; not physical capacity/SoC validation. | [294](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:294>) |
| `CFG_SOC_NO_LOAD_CURRENT_A` | `5.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [297](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:297>) |
| `CFG_ENERGY_MAX_GAP_MS` | `1000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [299](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:299>) |
| `CFG_PRECHARGE_TARGET_PERCENT` | `95` | RULE-DERIVED | Code cites FS2027; event/year applicability and physical margins need team approval. | [311](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:311>) |
| `CFG_PRECHARGE_TIMEOUT_MS` | `5000` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [312](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:312>) |
| `CFG_PRECHARGE_OVERLAP_MS` | `50` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [313](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:313>) |
| `CFG_CAN_ID_TS_REQUEST` | `0xB300` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [318](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:318>) |
| `CFG_CAN_TS_REQUEST_EXTENDED` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [319](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:319>) |
| `CFG_CAN_ID_CONFIG_REQUEST` | `0xB301` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [321](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:321>) |
| `CFG_TS_REQUEST_TIMEOUT_MS` | `0` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [323](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:323>) |
| `CFG_INVERTER_LIMITS_ENABLED` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [344](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:344>) |
| `CFG_INVERTER_NODE_ID` | `0xFF` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [345](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:345>) |
| `CFG_INVERTER_LIMIT_PERIOD_MS` | `20` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [346](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:346>) |
| `CFG_INVERTER_POWER_LIMIT_W` | `80000.0f` | RULE-DERIVED | Code cites FS2027; event/year applicability and physical margins need team approval. | [347](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:347>) |
| `CFG_INVERTER_DERATE_START_C` | `55.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [348](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:348>) |
| `CFG_INVERTER_DERATE_END_C` | `60.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [349](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:349>) |
| `CFG_INVERTER_DERATE_CURRENT_A` | `25.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [350](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:350>) |
| `CFG_INVERTER_REGEN_CURRENT_A` | `35.0f` | DATASHEET-DERIVED | 35A assumes Li1x7pVTC6; parallel count, charge window and transient allowance unapproved. | [354](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:354>) |
| `CFG_INVERTER_REGEN_DERATE_START_C` | `45.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [355](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:355>) |
| `CFG_INVERTER_REGEN_DERATE_END_C` | `60.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [356](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:356>) |
| `CFG_INVERTER_REGEN_DERATE_CURRENT_A` | `25.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [357](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:357>) |
| `CFG_POWER_WINDOW_LIMITER_ENABLED` | `0` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [373](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:373>) |
| `CFG_POWER_WINDOW_MS` | `500` | RULE-DERIVED | Code cites FS2027; event/year applicability and physical margins need team approval. | [374](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:374>) |
| `CFG_POWER_WINDOW_LIMIT_W` | `CFG_INVERTER_POWER_LIMIT_W` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [375](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:375>) |
| `CFG_POWER_WINDOW_FLOOR_W` | `72000.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [376](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:376>) |
| `CFG_POWER_WINDOW_LAG_MS` | `10` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [377](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:377>) |
| `CFG_POWER_WINDOW_SPEND_MS` | `200` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [378](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:378>) |
| `CFG_PACK_RESISTANCE_MIN_OHM` | `0.15f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [385](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:385>) |
| `CFG_PACK_RESISTANCE_MAX_OHM` | `0.6f` | HARDWARE TEST REQUIRED | Cell/BOM, thermal, R/C, latency, polarity or actual clock-dependent value; do not approve from simulation. | [386](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:386>) |
| `CFG_SETTING_POWER_MIN_W` | `1000.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [412](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:412>) |
| `CFG_SETTING_CURRENT_MIN_A` | `1.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [413](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:413>) |
| `CFG_SETTING_TEMP_LIMIT_MIN_C` | `30.0f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [414](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:414>) |
| `CFG_CHARGER_CURRENT_A` | `1.0f` | LEGACY / UNKNOWN | Retained/chosen value; no signed engineering approval or physical validation supplied. | [416](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:416>) |
| `CFG_CHARGER_CURRENT_MAX_A` | `35.0f` | DATASHEET-DERIVED | 35A assumes Li1x7pVTC6; parallel count, charge window and transient allowance unapproved. | [417](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:417>) |
| `CFG_CHARGER_VOLTAGE_V` | `(CFG_TOTAL_CELLS * 4.02f)` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [418](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:418>) |
| `CFG_CHARGER_VOLTAGE_MAX_V` | `(CFG_TOTAL_CELLS * (CFG_CELL_OV_V - 0.05f))` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [419](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:419>) |
| `CFG_SETTINGS_FLASH_ADDR` | `0x0807F800` | VERIFIED | Within linker-reserved final4kB; device/bank/erase timing still requires physical validation. | [423](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:423>) |
| `CFG_SETTINGS_SAVE_INTERVAL_MS` | `2000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [424](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:424>) |
| `CFG_CAN1_BITRATE` | `1000000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [438](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:438>) |
| `CFG_CAN2_BITRATE` | `500000` | DATASHEET-DERIVED | IVT v1.01 or ADBMS Rev0 protocol/default; verify fitted configuration and hardware. | [439](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:439>) |
| `CFG_CAN3_BITRATE` | `250000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [440](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:440>) |
| `CFG_CAN_TX_PERIOD_MS` | `100` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [452](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:452>) |
| `CFG_CAN_FAST_PERIOD_MS` | `4` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [454](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:454>) |
| `CFG_CAN_POWER_W_PER_BIT` | `2.5f` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [455](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:455>) |
| `CFG_CAN1_MAX_LOAD_PERCENT` | `50` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [456](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:456>) |
| `CFG_CAN_V_FRAMES_PER_SLAVE` | `((CFG_CELLS_PER_SLAVE + 3) / 4)` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [458](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:458>) |
| `CFG_CAN_T_FRAMES_PER_SLAVE` | `((CFG_TEMPS_PER_SLAVE + 3) / 4)` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [459](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:459>) |
| `CFG_CAN_MEAS_FRAMES` | `(CFG_NUM_SLAVES * (CFG_CAN_V_FRAMES_PER_SLAVE + CFG_CAN_T_FRAMES_PER_SLAVE + 1))` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [460](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:460>) |
| `CFG_CAN_BURST_FRAMES` | `(CFG_CAN_MEAS_FRAMES + 4)` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [461](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:461>) |
| `CFG_CAN_FAST_FRAMES` | `2` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [462](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:462>) |
| `CFG_CAN_FRAME_BITS_MAX` | `160` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [463](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:463>) |
| `CFG_CAN1_INVERTER_ID_MAX` | `0x3FFF` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [470](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:470>) |
| `CFG_CAN_ID_INVERTER_DC_LIMIT` | `((0x0A << 8) \| CFG_INVERTER_NODE_ID)` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [473](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:473>) |
| `CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT` | `((0x0B << 8) \| CFG_INVERTER_NODE_ID)` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [474](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:474>) |
| `CFG_CAN_ID_CELL_V_BASE` | `0xB000` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [476](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:476>) |
| `CFG_CAN_ID_CELL_T_BASE` | `0xB080` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [477](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:477>) |
| `CFG_CAN_ID_SLAVE_BASE` | `0xB100` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [478](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:478>) |
| `CFG_CAN_ID_PACK` | `0xB200` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [479](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:479>) |
| `CFG_CAN_ID_STATUS` | `0xB201` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [480](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:480>) |
| `CFG_CAN_ID_SUMMARY` | `0xB202` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [481](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:481>) |
| `CFG_CAN_ID_ENERGY` | `0xB203` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [482](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:482>) |
| `CFG_CAN_ID_FAST_1` | `0xB204` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [483](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:483>) |
| `CFG_CAN_ID_FAST_2` | `0xB205` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [484](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:484>) |
| `CFG_CAN_ID_LIMITS` | `0xB206` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [485](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:485>) |
| `CFG_CAN_ID_CONFIG_RESPONSE` | `0xB207` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [486](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:486>) |
| `CFG_PIN_FAULT_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [508](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:508>) |
| `CFG_PIN_FAULT_PIN` | `GPIO_PIN_1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [509](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:509>) |
| `CFG_FAULT_ACTIVE_HIGH` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [510](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:510>) |
| `CFG_FAULT_OPEN_DRAIN` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [511](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:511>) |
| `CFG_PIN_TSAC_READY_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [514](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:514>) |
| `CFG_PIN_TSAC_READY_PIN` | `GPIO_PIN_8` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [515](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:515>) |
| `CFG_TSAC_READY_ACTIVE_HIGH` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [516](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:516>) |
| `CFG_PIN_PRECHARGE_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [519](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:519>) |
| `CFG_PIN_PRECHARGE_PIN` | `GPIO_PIN_6` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [520](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:520>) |
| `CFG_PIN_BYPASS_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [521](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:521>) |
| `CFG_PIN_BYPASS_PIN` | `GPIO_PIN_9` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [522](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:522>) |
| `CFG_PRECHARGE_ACTIVE_HIGH` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [523](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:523>) |
| `CFG_BYPASS_ACTIVE_HIGH` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [524](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:524>) |
| `CFG_PIN_SDC_IN_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [527](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:527>) |
| `CFG_PIN_SDC_IN_PIN` | `GPIO_PIN_2` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [528](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:528>) |
| `CFG_SDC_IN_ACTIVE_HIGH` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [529](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:529>) |
| `CFG_PIN_CHARGER_BTN_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [532](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:532>) |
| `CFG_PIN_CHARGER_BTN_PIN` | `GPIO_PIN_0` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [533](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:533>) |
| `CFG_CHARGER_BTN_ACTIVE_LOW` | `1` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [534](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:534>) |
| `CFG_PIN_ISOSPI_CS_PORT` | `GPIOB` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [537](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:537>) |
| `CFG_PIN_ISOSPI_CS_PIN` | `GPIO_PIN_6` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [538](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:538>) |
| `CFG_PIN_ISOSPI_CS2_PORT` | `GPIOC` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [539](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:539>) |
| `CFG_PIN_ISOSPI_CS2_PIN` | `GPIO_PIN_7` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [540](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:540>) |
| `CFG_PIN_ISOSPI_MSTR_PORT` | `GPIOA` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [541](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:541>) |
| `CFG_PIN_ISOSPI_MSTR_PIN` | `GPIO_PIN_10` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [542](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:542>) |
| `CFG_PIN_ISOSPI_MSTR2_PORT` | `GPIOB` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [543](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:543>) |
| `CFG_PIN_ISOSPI_MSTR2_PIN` | `GPIO_PIN_4` | HARDWARE TEST REQUIRED | Match approved schematic/BOM/harness; mapping/output blockers in C/F/I/N. | [544](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:544>) |
| `CFG_DEBUG_PRINT_LOOP` | `1` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [550](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:550>) |
| `CFG_DEBUG_PRINT_CELLS` | `0` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [551](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:551>) |
| `CFG_DEBUG_PRINT_TEMPS` | `0` | TEAM DECISION REQUIRED | Deliberate software policy/budget/network allocation; tests show implementation, not engineering approval. | [552](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:552>) |
| `CFG_LOCAL_OVERRIDES` | `0 normally; 1 for an explicitly selected local/variant override` | VERIFIED | Software-derived layout/range or normal-build state; tests passed, no whole-vehicle approval. | [576](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h:576>) |
| `CHARGER_CONFIG_CAN_ID` | `0x1806E5F4` | LEGACY / UNKNOWN | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [32](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_can.h:32>) |
| `CHARGER_STATUS_CAN_ID` | `0x18FF50E5` | LEGACY / UNKNOWN | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [33](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_can.h:33>) |
| `IVT_COMMAND_GAP_MS` | `3` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [39](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:39>) |
| `IVT_SLOW_CURRENT_FRAMES` | `5` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [96](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:96>) |
| `IVT_SETTLE_MS` | `50` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [97](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:97>) |
| `IVT_GAP_MS` | `25` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [103](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:103>) |
| `IVT_SYS_ERROR_PERSIST_MS` | `(CFG_FAULT_DEBOUNCE_MS + 2 * CFG_SAFETY_TICK_MS)` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [407](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:407>) |
| `IVT_CLEAN_BUS_MS` | `2000` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [408](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:408>) |
| `IVT_GET_TIMEOUT_MS` | `100` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [409](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:409>) |
| `IVT_RESET_TIMEOUT_MS` | `2000` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [410](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:410>) |
| `IVT_CHECK_TRIES` | `3` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [420](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c:420>) |
| `NUM_CHANNELS` | `16` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [44](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:44>) |
| `NUM_GPIOS` | `10` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [45](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:45>) |
| `NUM_VOLTAGE_GROUPS` | `6` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [46](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:46>) |
| `NUM_AUX_GROUPS` | `5` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [47](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:47>) |
| `ADC_LSB_V` | `0.00015f` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [53](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:53>) |
| `ADC_OFFSET_V` | `1.5f` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [54](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:54>) |
| `PWM_DUTY_MAX` | `0b1111` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [55](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:55>) |
| `DATA_LEN` | `(6)` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [16](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/inc/bms_utility.h:16>) |
| `DATAPKT_LEN` | `(6 + 2)` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [17](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/inc/bms_utility.h:17>) |
| `CMD_LEN` | `(2)` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [18](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/inc/bms_utility.h:18>) |
| `CMDPKT_LEN` | `(2 + 2)` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [19](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/inc/bms_utility.h:19>) |
| `NUM_FLAG_BITS` | `8` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [29](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_safety.c:29>) |
| `IWDG_PRESCALER_32` | `0x3U` | DATASHEET-DERIVED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [34](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_safety.c:34>) |
| `EVENT_QUEUE_LEN` | `16` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [115](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_safety.c:115>) |
| `BUFFER_SIZE` | `10000` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [16](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/uartDMA.c:16>) |
| `MESSAGE_MAX` | `256` | TEAM DECISION REQUIRED | Fixed register/protocol encoding or internal timer/buffer policy; see subsystem audit. Charger IDs require real protocol selection. | [17](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/uartDMA.c:17>) |
| `startup delay` | `500ms` | TEAM DECISION REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [237](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c:237>) |
| `initial C-ADC fill` | `12ms` | HARDWARE TEST REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [327](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:327>) |
| `charger enable prerequisite` | `cached output >=300V` | LEGACY / UNKNOWN | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [1379](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c:1379>) |
| `fan initial duty` | `0.7` | LEGACY / UNKNOWN | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [227](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c:227>) |
| `charger button debounce` | `50ms` | TEAM DECISION REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [532](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c:532>) |
| `blue button debounce` | `500ms` | TEAM DECISION REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [511](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c:511>) |
| `debug UART` | `1000000baud` | TEAM DECISION REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [43](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/usart.c:43>) |
| `TIM15 IMD capture` | `128MHz/no prescale/ARR65535` | HARDWARE TEST REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [143](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/tim.c:143>) |
| `TIM3 fan PWM` | `128MHz/20/256 =25kHz` | HARDWARE TEST REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [88](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/tim.c:88>) |
| `TIM2 delay counter` | `128MHz/128 =1MHz` | HARDWARE TEST REQUIRED | Hard-coded value outside central configuration; preserve pending explicit approval/validation. | [47](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/tim.c:47>) |

[Complete per-mode group CSV](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/test-results.csv>) and [full test summary/soak evidence](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/test-summary.json>). The following compact table lists each normal-run group and its selected variants; all four full modes passed each selected group. The separately enabled/excluded random case failed and is not included as a pass.

| Group | Normal car configuration | Eval variant | Enabled-window variant |
|---|---|---|---|
| `boot` | PASS | PASS | PASS |
| `boot_u1_disabled` | PASS | PASS | not selected |
| `can_layout` | PASS | PASS | not selected |
| `glitch_immunity` | PASS | PASS | not selected |
| `current_spike_immunity` | PASS | PASS | not selected |
| `pec_noise` | PASS | PASS | not selected |
| `balancing` | PASS | not selected | not selected |
| `balancing_hysteresis` | PASS | PASS | not selected |
| `leds` | PASS | not selected | not selected |
| `charging` | PASS | PASS | not selected |
| `can_no_ack` | PASS | PASS | not selected |
| `can_bus_off` | PASS | PASS | not selected |
| `ivt_power_cycle` | PASS | PASS | not selected |
| `ivt_power_cycle_u1_off` | PASS | PASS | not selected |
| `power_on_slaves_late` | PASS | PASS | not selected |
| `power_on_ivt_late` | PASS | PASS | not selected |
| `power_on_slaves_then_ivt` | PASS | PASS | not selected |
| `power_on_ivt_then_slaves` | PASS | PASS | not selected |
| `dropout_ivt_brief` | PASS | PASS | not selected |
| `dropout_ivt_long` | PASS | PASS | not selected |
| `dropout_slave_brief` | PASS | PASS | not selected |
| `dropout_slave_long` | PASS | PASS | not selected |
| `dropout_chain_brief` | PASS | PASS | not selected |
| `isospi_chaos` | PASS | PASS | not selected |
| `slave_reset_restored` | PASS | PASS | not selected |
| `isospi_lost_command` | PASS | PASS | not selected |
| `intermittent_overcurrent` | PASS | PASS | not selected |
| `overcurrent_pulses` | PASS | PASS | not selected |
| `current_glitches_periodic` | PASS | PASS | not selected |
| `intermittent_cell_overvoltage` | PASS | PASS | not selected |
| `can2_flood_low_priority` | PASS | PASS | not selected |
| `can2_flood_high_priority` | PASS | PASS | not selected |
| `can2_junk_frames` | PASS | PASS | not selected |
| `ivt_energy_extremes` | PASS | PASS | not selected |
| `ivt_spurious_alive` | PASS | PASS | not selected |
| `ivt_stray_stop` | PASS | PASS | not selected |
| `ivt_restart_command` | PASS | PASS | not selected |
| `ivt_config_interrupted` | PASS | PASS | not selected |
| `ivt_can_errors_cleared` | PASS | PASS | not selected |
| `ivt_system_error_kept` | PASS | PASS | not selected |
| `ivt_can_error_clear_limit` | PASS | PASS | not selected |
| `ivt_can_errors_during_disturbance` | PASS | PASS | not selected |
| `uart_flood` | PASS | PASS | not selected |
| `temperature_table_edges` | PASS | PASS | not selected |
| `soc_model` | PASS | PASS | not selected |
| `soc_startup` | PASS | PASS | not selected |
| `energy_tracking` | PASS | PASS | not selected |
| `fast_frames` | PASS | PASS | PASS |
| `inverter_limits` | PASS | PASS | not selected |
| `power_window_sustained` | PASS | not selected | PASS |
| `power_window_bursts` | PASS | not selected | PASS |
| `power_window_random` | PASS | not selected | not selected |
| `load_swings` | PASS | not selected | PASS |
| `settings` | PASS | PASS | not selected |
| `settings_save_wear` | PASS | PASS | not selected |
| `can_long_dlc` | PASS | PASS | not selected |
| `uart_ring_wrap` | PASS | PASS | not selected |
| `settings_fuzz` | PASS | PASS | not selected |
| `settings_foreign_record` | PASS | PASS | not selected |
| `precharge_nominal` | PASS | PASS | not selected |
| `precharge_timeout_retry` | PASS | PASS | not selected |
| `precharge_fault_abort` | PASS | PASS | not selected |
| `precharge_sdc_open` | PASS | PASS | not selected |
| `precharge_while_faulted` | PASS | PASS | not selected |
| `precharge_bad_battery_voltage` | PASS | PASS | not selected |
| `ts_request_abuse` | PASS | PASS | not selected |
| `ivt_value_extremes` | PASS | PASS | not selected |
| `soak_1` | PASS | not selected | not selected |
| `soak_2` | PASS | not selected | not selected |
| `soak_3` | PASS | not selected | not selected |
| `fault_cell_overvoltage` | PASS | PASS | not selected |
| `fault_cell_undervoltage` | PASS | PASS | not selected |
| `fault_cell_overtemp` | PASS | PASS | not selected |
| `fault_cell_undertemp` | PASS | PASS | not selected |
| `fault_temp_sensor_open` | PASS | PASS | not selected |
| `fault_slave_die_overtemp` | PASS | PASS | not selected |
| `fault_open_sense_wire` | PASS | PASS | not selected |
| `fault_overcurrent_discharge` | PASS | PASS | not selected |
| `fault_overcurrent_charge` | PASS | PASS | not selected |
| `fault_overcurrent_ivt_range` | PASS | PASS | not selected |
| `fault_pack_overvoltage` | PASS | PASS | not selected |
| `fault_pack_implausible` | PASS | PASS | not selected |
| `fault_ivt_lost` | PASS | PASS | not selected |
| `fault_ivt_sensor_error` | PASS | PASS | not selected |
| `fault_isospi_chain_broken` | PASS | PASS | not selected |
| `fault_spi_timeout` | PASS | PASS | not selected |
| Register layout ARM+host | PASS | shared | shared |
| DBC decoding | PASS | not applicable | not applicable |
| SoC/workbook | PASS | shared | shared |
| GUI12 tests, virtual CAN/offscreen | PASS | not applicable | not applicable |

Source index (all paths are local audit inputs; no remote provenance inferred):

- [L-startup](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Startup/startup_stm32g474retx.s>)
- [L-main](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c>)
- [L-config](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h>)
- [L-mcu](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_mcuWrapper.c>)
- [L-transport](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_utility.c>)
- [L-wrapper](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c>)
- [L-ivt](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c>)
- [L-can](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_can.c>)
- [L-fdcan](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/fdcan.c>)
- [L-safety](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_safety.c>)
- [L-precharge](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_precharge.c>)
- [L-inverter](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_inverter.c>)
- [L-gpio](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_gpio.c>)
- [L-soc](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_soc.c>)
- [L-settings](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_settings.c>)
- [master-pdf](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Schematic PDF_[No Variations].pdf>)
- [ivt-pdf](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Datasheet_IVT-S.pdf>)
- [dti-pdf](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Can_Manual_V2.4.pdf>)
- [charger-main](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/charger-display-reference/Core/Src/main.c>)
- [charger-fdcan](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/charger-display-reference/Core/Src/fdcan.c>)
- [probes-src](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/audit_contract_probe.c>)
- [probes-command](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/contract-probe-command.json>)
- [probes-results](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/contract-probe-results.txt>)

Additional supplied documents used:
- [adbms6830b.pdf](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/adbms6830b.pdf>)
- [Datasheet_with_VTC6_rev_D2024-10-28.pdf](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Datasheet_with_VTC6_rev_D2024-10-28.pdf>)

[Archive inventory](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/archive-inventory.json>); [Git states](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/git-state.json>); [unchanged tracked hashes](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/lead-before.json>); [master drawing rendering](<C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/master-schematic.png>).

[L-startup]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Startup/startup_stm32g474retx.s>
[L-main]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/main.c>
[L-config]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Inc/bms_config.h>
[L-mcu]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_mcuWrapper.c>
[L-transport]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_utility.c>
[L-wrapper]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/custom_lib/src/bms_libWrapper.c>
[L-ivt]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_ivt.c>
[L-can]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_can.c>
[L-fdcan]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/fdcan.c>
[L-safety]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_safety.c>
[L-precharge]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_precharge.c>
[L-inverter]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_inverter.c>
[L-gpio]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_gpio.c>
[L-soc]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_soc.c>
[L-settings]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Core/Src/bms_settings.c>
[master-pdf]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Schematic PDF_[No Variations].pdf>
[ivt-pdf]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Datasheet_IVT-S.pdf>
[dti-pdf]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/source/g474-bms-main/Supporting Docs/Can_Manual_V2.4.pdf>
[charger-main]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/charger-display-reference/Core/Src/main.c>
[charger-fdcan]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/charger-display-reference/Core/Src/fdcan.c>
[probes-src]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/audit_contract_probe.c>
[probes-command]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/contract-probe-command.json>
[probes-results]: <C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/integration-audit/contract-probe-results.txt>

## Prioritized action list

**P0 — before further firmware integration**

1. Freeze authoritative master/slave/HAT/harness revisions and resolve TEMP/LED, three-CAN/VIO, SDC/latch and relay/precharge ownership discrepancies.
2. Identify the fitted IVT and actual charger/gateway; approve their wiring, references, termination and bus/protocol assignments.
3. Freeze module/parallel count, charge/regen current and temperature requirements; resolve the200A trip and out-of-window regen behavior. Do not approve active balancing or its thermal values implicitly.
4. Select event/rule set and a provenance-preserving integration baseline. Record explicit acceptance criteria for validity, ride-through, shutdown and external latching.

**P1 — before the first bench test**

1. Build an explicitly approved bench configuration matching the available hardware; inhibit/physically isolate unapproved relay, inverter and balancing functions.
2. Resolve strict IVT DLC/counter/freshness contracts and transactional ADBMS publication; add regression cases reproducing the audit findings.
3. Resolve the14-device poll-status priming requirement in both driver and simulation; bound/check TIM2 waits and verify early watchdog/fatal paths.
4. Correct/qualify the chosen temperature mapping/calibration and70C arithmetic boundary; implement the approved charge/regen safety window and current limits.
5. Establish charger DLC/status-age/disable contracts and precharge request/U2/failure contracts before enabling those functions. Verify inverter node/settings and BMS-only limit-loss behavior.
6. Prove GPIO polarity, reset/power-loss states and real latch/SDC behavior using safe low-voltage fixtures; measure ISR/critical-section/loop timing. Passing software tests is not this proof.

**P2 — before accumulator testing**

1. Validate all14 boards/112 positions with calibrated inputs, missing-board/PEC/counter/power-dip/open-wire injection, correct sensor bias and the complete physical harness.
2. Validate resistor temperatures, board heat, PWM cap and stop-on-link-loss/reset behavior; enable balancing only after signed thermal approval.
3. Validate cell/current/temperature limits and the complete charging/precharge/shutdown chain with actual sensor, charger and contactors; include negative AIR/reference, welded/missing-contact feedback and stale U2 scenarios as required by the approved design.
4. Measure pack resistance, inverter response and actual logger power; validate conservative80kW behavior and current/regen limits under load. Keep the window limiter off.
5. Run hardware CAN arbitration/flood/dropout/EMC/termination tests and prove missing BMS limits or charger disable cannot leave unsafe control active.

**P3 — before competition**

1. Close the event-specific rules/compliance evidence, complete non-programmable latch/IMD/shutdown validation and obtain engineering approval for every unresolved parameter.
2. Lock tool/dependency versions, archive firmware/configuration/build/test/hardware hashes, commit an agreed clean baseline and maintain a release/rollback procedure.
3. Complete target-duration soak, power/thermal/EMC tests, combined vehicle regression, VCU/logger/charger interoperability and documented failure/recovery drills.
4. Resolve or formally exclude incomplete optional functionality (including the enabled-window random/floor case); retain strict contract and end-to-end suites together. Schedule unrelated warning/style cleanup separately.
