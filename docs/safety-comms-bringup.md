This cleanup prepares software for controlled bring-up; it does not approve the electrical design, safety limits or vehicle operation. No hardware was flashed. Topology is centralized in `Core/Inc/bms_topology.h`: fourteen boards, eight populated cell positions and eight direct temperature inputs per board, 112 pack positions. The ADBMS register capacity remains sixteen; that is explicitly a hardware capacity, not a populated topology assumption.

All configurable limits and communication budgets are in `Core/Inc/bms_safety_config.h`. Values are explicitly unresolved. Inherited values were retained where possible, with board/pack derivations corrected; new finite deadlines are provisional software budgets.

| Parameter | Current bring-up setting | Unresolved decision |
|---|---|---|
| Cell UV / OV | 3300 / 4200 mV | Chemistry and operating window. |
| Aggregate UV basis | 3000 mV per populated position | Does not equal cell UV; retain this distinction until the intended policy is approved. |
| Segment UV / OV | 8 x basis: 24.0 / 33.6 V | VPV wiring/scaling and aggregate protection policy. |
| Pack UV / OV | 112 x basis: 336.0 / 470.4 V | Confirm pack operating window. |
| Pack current | -10000 to +10000 mA | Legacy limits; not an approved vehicle current rating or polarity convention. |
| External temperature | 0-60 C | Sensor model, conversion, location and operating limits. Celsius stays invalid. |
| Die temperature | 0-70 C | Confirm the intended chip/board thermal policy. |
| IVT current / U1 maximum age | 100 / 300 ms | Verify sensor output mode and vehicle safety budgets; U1 might be disabled. |
| ADBMS cell / AUX maximum age | 1000 / 1000 ms | New provisional watchdogs; confirm against actual loop and bus timing. |
| SPI transaction / ADC poll budget | 10 / 50 ms | New finite software deadlines; validate against all fourteen boards and fault cases. |
| Balancing threshold | 30 mV | Inherited, not approved for the new resistance/thermal design. |
| Maximum PWM / aggregate PWM budget | 0 / 0 | Bring-up inhibition, not final operating values. |
| Balancing configuration confirmed | false | Requires an explicit source configuration change after physical review. |
| Discharge timeout | 1 minute, inactive during bring-up | Inherited fallback; not an approved thermal shutdown time. |
| Voltage / temperature fault delay | 490 / 990 ms | Inherited, evaluated by the existing 100 ms timer. |
| Acquisition delay / startup delay | 500 / 500 ms | Inherited cadence/wait; validate actual safety response times. |
| Communication retries | 2 attempts, 250 ms apart | Inherited retry budget, now finite per handler invocation. |
| Wake pulse / initial cell-fill wait | 1 / 12 ms | Inherited timings; verify on hardware. |
| Charger target / maximum current / minimum enable voltage | 450 V / 1 A / 300 V | Inherited settings, pending charger/pack approval. |
| ADBMS hardware VUV / VOV codes | 0x800 / 0x7FF | Existing reset-register codes, not thresholds matched to the software limits. |
| C-versus-S comparison / filter codes | 1 / 1 | Existing register selections. Redundant diagnostic coverage is not newly claimed. |

Voltage configuration uses integer millivolts, with conversion to volts after deriving the eight-/112-position limits. This avoids silently moving an inclusive threshold because of float multiplication. All comparisons reject unavailable/nonfinite measurements. Host fixtures exercise exact thresholds and values just outside them. The inherited 3.0 V aggregate UV basis and 3.3 V individual-cell UV setting remain visibly separate.

IVT U1 and the summed ADBMS cell voltages remain separate. The diagnostic difference is U1 minus cell sum. Neither source is averaged into or substituted for the other. No disagreement threshold was chosen; a difference is not by itself an invented protection decision. Samples are asynchronous, so a future tolerance must account for measurement intervals, wiring and calibration.

ADBMS communication now returns checked status from command, data, receive and poll operations. Every blocking HAL SPI call has a finite 10 ms timeout. ADC polling requires 0xFF, rejects partial/nonzero busy responses, and ends within the 50 ms poll budget using wrap-safe HAL tick subtraction. A single register read/write uses at most two SPI call budgets. Receive failure poisons the destination with reset-code words and clears received PEC/counter fields, so a caller cannot accidentally decode an older payload. Chip select is deasserted on all started-transaction exits. Blocking SPI is rejected from ISR or interrupt-masked contexts where the HAL deadline might not advance. The microsecond timer delay also has a HAL-tick fallback deadline; a stopped timer cannot trap initialization forever.

Cell and AUX reads use staging storage. Interrupt consumers retain the preceding complete snapshot while a new acquisition is in progress. Publication is a short interrupt-masked operation after the complete read/PEC pass. Cell reset codes are rejected. GPIO/die/VPV reset codes retain separate per-value validity. External Celsius remains NaN/invalid; the legacy thermistor table and function body are untouched and unused.

Acquisition failure invalidates **all** ADBMS cell/AUX values and statistics, clears balancing requests, disables charging and asserts the fault output. Successful unrelated SID/config reads cannot make those measurements fresh. The periodic timer independently checks ADBMS and IVT age, even if the main acquisition loop stalls. Stale cell voltages/differences use invalid telemetry sentinels and clear validity bits; stale cell sums suppress the cross-check. Snapshot timestamps are host acquisition-completion times, not hardware sample timestamps or proof of sensor authenticity.

Live IVT overcurrent is included in the shutdown hold/release condition. Previously, a current fault could disable charging while the timer cleared the shutdown output. `BMS_CheckCurrent` now checks fresh integer mA data against centralized limits as well as the last computed fault flag. Charging-enable requests are refused while any computed software fault or live pack/ADBMS validity/current check is unhealthy, so button/API requests cannot bypass these checks. The timer's existing 100 ms cadence still bounds when it observes the result; this is not an approved hardware overcurrent reaction-time budget. Voltage/temperature fault timers retain their configured delays.

Balancing defaults off and cannot be enabled by a button/API call while configuration is unconfirmed or the maximum PWM is zero. Direct setters clamp populated channels to the configured cap and force indices 8-15 (PWM9-16) to zero. Every register write sanitizes unused PWM fields, clears the entire PWMB payload including reserved bytes, masks discharge telemetry to 0x00FF and clears all direct-DCC overrides. Reset/init/stop/failure paths clear all sixteen PWM positions, all DCC bits, discharge timers/modes and software discharge flags. A future enabled balancing start requires fresh measurements and healthy status, writes the sanitized controls, and only then sends UNMUTE.

Stop sends MUTE and attempts CFGB, PWMA and PWMB zero writes even if one write fails. If those transfers succeed, it reads back all three register groups with PEC and confirms zero timer/DCC/PWM fields on every device. This is bounded and nonrecursive. `BMS_AdbmsDataStatus` exposes snapshot timestamps/freshness, the primary acquisition transport fault, PEC failure and whether discharge stop/readback failed. Stop transactions do not overwrite the primary acquisition diagnosis. For a healthy link, readback confirms the reported register values, not physical switching behavior. For a broken link, physical-off state cannot be proven: the fault output stays asserted. Calls from timer/button context queue a foreground stop; no blocking SPI is attempted in that ISR. Autonomous hardware discharge timeout and shutdown-path effectiveness still require physical verification.

The main fault handler now limits each communication-recovery burst to two attempts instead of looping indefinitely. Initialization is called once per attempt; the previous duplicate initialization call is removed. Combined fault bitmasks containing a communication error enter recovery. Subsequent foreground passes may retry initialization; the normal firmware loop remains continuous, but each poll/transaction/recovery burst is bounded.

PEC10 verification is preserved, including its received six-bit command-counter contribution. Expected sequence checking is now implemented in `bms_command_counter.c` and the existing transport/wrapper insertion points. The model follows ADBMS6830B Rev. 0 p53 and Table 50 p57-59: INC commands (including polling) advance once, data writes advance after successful complete transmission and CS rising, reads do not advance, resets establish zero, and rollover is 63 to 1. Each device's PEC-verified counter must match the host prediction before parsing/publication. A mismatch latches until explicit reset; a failed transmit makes acceptance uncertain and requires resynchronization. Initialization/recovery sends RSTCC and verifies every device through RDSID before proceeding. Command-counter faults use the same complete invalidation, charging/balancing inhibition and fault-output handling as PEC/transport faults; primary per-device expected/received diagnostics survive stop attempts. Matching SID/config reads never refresh measurement timestamps. See `docs/software-readiness-level2.md` for the final verification evidence, protocol coverage limits and exact gated bench procedure. Counter matching is communication evidence, not proof of ADC correctness, sample age inside the IC, authenticity or completed safety certification.

The vehicle telemetry map is exact and uses extended identifiers, DLC 8:

| ID/range | Count | Meaning |
|---|---|---|
| 0xB000-0xB06F | 112 | Board-major cell/TEMP slots: index = board x 8 + position. TEMPn is a connector index, not proof of physical placement on cell n. |
| 0xB070-0xB07D | 14 | Per-board VPV, die temperature, fault/communication flags and validity. |
| 0xB07E | 1 | Signed IVT pack current in mA, with raw status/counter/validity. |
| 0xB07F | 1 | Signed IVT U1 pack voltage in mV, with raw status/counter/validity. |
| 0xB080 | 1 | Independent ADBMS cell sum and U1-minus-sum difference, both mV. |
| 0x1806E5F4 | 1 | Existing charger configuration. |
| 0x18FF50E5 | RX | Existing charger status; not part of the transmit buffer. |

The transmit count/capacity is exactly 112+14+3+1=130, including when ADBMS measurements are invalid. Compile-time assertions prove disjoint cell/board/pack ranges and separation from charger IDs. Host tests inspect every emitted identifier, verify all 129 contiguous BMS slots exactly once, verify all 130 messages pairwise, and check extended-frame type. No internal collision exists. This proof covers firmware-defined identifiers only; the complete external vehicle DBC/other ECU allocation was not supplied.

All telemetry multibyte values are little-endian, independently of IVT receive big-endian encoding:

| Frame | Bytes | Encoding |
|---|---|---|
| Cell/TEMP | 0-1 / 2-3 / 4-5 | Signed int16 cell mV / difference from pack minimum mV / external centidegrees C. INT16_MIN means invalid. |
| Cell/TEMP | 6 | Bit 0 discharging; bit 1 combined slot fault; upper bits zero. |
| Cell/TEMP | 7 | Bit 0 fresh temperature-input voltage; bit 1 valid Celsius; bit 2 fresh cell voltage. Upper bits zero. |
| Board | 0-3 / 4-5 | Signed int32 VPV mV / int16 die centidegrees C; corresponding minimum-integer sentinels when invalid. |
| Board | 6 | Bit 0 communications error or stale cell/AUX snapshot; bit 1 board fault flag. |
| Board | 7 | Bit 0 VPV valid/fresh; bit 1 die temperature valid/fresh; bit 2 selected cell-voltage snapshot fresh. |
| IVT current / U1 | 0-3 | Signed int32 mA / mV; INT32_MIN when invalid. Inspect the validity bit because signed protocol data can also represent INT32_MIN. |
| IVT current / U1 | 4 / 5 / 6 / 7 | Raw IVT DB1 / low-nibble counter / flags / zero. Flags: bit 0 valid, bit 1 stale, bit 2 received, bit 3 configuration verified. |
| Cross-check | 0-3 / 4-7 | Signed int32 fresh cell-sum mV / U1-minus-cell-sum mV. INT32_MIN if unavailable or not representable. |

Run `python tests/run_host_tests.py --cc /path/to/gcc`. It runs the IVT protocol suite, the application/safety suite with the firmware's disabled balancing configuration, the same suite with host-only simulated PWM cap/budget, and the actual SPI/PEC/poll implementation with HAL mocks. The nonzero PWM fixture is not a firmware value or an approval. Coverage includes inclusive cell/board/pack and current boundaries, 8/112 derivation, NaN rejection, stale cell/AUX telemetry, tick wrap, transactional publication, every cell/AUX PEC failure group, HAL/poll failures, reset words, independent voltage disagreement, zero/active balancing masks, failed stop/readback and exhaustive telemetry uniqueness. Physical CAN/SPI timing, interrupt-masked publication latency, the microsecond timer and output transistor behavior remain bench tests.

Before energizing hardware, resolve chemistry/limits and response budgets; confirm the final thermistor and conversion; measure balancing resistance/current/thermal capacity; verify all upper-channel controls remain off on the actual chain; test loss/short/open wiring and shutdown effectiveness; verify IVT part/U1 wiring, persisted output configuration and separate CAN transceiver routing/termination; reconcile the complete vehicle DBC; physically validate command-counter diagnostics and complete the planned open-wire/redundancy diagnostics. Existing charger-status fault bits are decoded but some button-enable checks remain commented in the legacy application; their required vehicle behavior also needs review. None of those decisions is inferred from the preliminary schematic.
