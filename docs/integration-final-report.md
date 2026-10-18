# Final firmware and GUI integration

Integration date: 19 October 2026. Physical hardware validation remains pending.

This implements the existing `final-integration-plan.md` in the read-only lead checkout and this repository's `gui-firmware-protocol-gap.md`. It is a deliberate source integration, not a Git branch merge. The firmware and GUI retain separate histories and are published only to the user's Asraeth forks, on `integration/bms-final`. No team upstream or main/master branch is updated.

## Architecture and provenance

Lead donor: `g474-bms-seb/g474-bms-main`, revision `75ea736c4108cb34aff9e7fd8996d6ff643494d7`. Migration baseline: `919a8a8`, preserved with its verified safeguards/audit in `b1fd57c`. GUI fork baseline: `dbb91c6`. The lead donor is unchanged. The local `codex/final-bms-integration` branches are pushed explicitly to the fork integration branches.

The current supplied hardware drawings take priority over current lead documentation, then manufacturer specifications, applicable rules, implementations, tests and historical documentation. Historical seven-board/ADBMS2950 architecture is not active. Supplied rules are design inputs, not a claim of current event approval.

| Subsystem | Plan decision | Implemented result |
|---|---|---|
| Topology | EQUIVALENT | 14 devices × 8 cells × 8 temperatures; physical 16-channel and six-byte register formats retained. BENCH explicitly has one device. |
| Board routing | HARDWARE-CONFIGURABLE | Selected lead, V3, BENCH and host-only DIGITAL profiles. TEMP1–TEMP8 and LED_RED/GREEN map in one profile; compile assertions reject illegal, duplicate or overlapping GPIOs. MCU aliases are in `bms_mcu_routes.h`; alternate functions remain in CubeMX. |
| Transport | COMBINE BOTH | Lead ordering/wake flow plus finite HAL/SPI/poll budgets, context guards, daisy-chain polling priming, CS cleanup and primary diagnostics. |
| Register parsing | COMBINE BOTH | Lead channel placement and layout probes plus explicit signed little-endian ADC decode, reset sentinel rejection and full-range numeric checks. |
| PEC | EQUIVALENT | Production PEC15/PEC10 retained; independent bitwise reference and corruption vectors. PEC validates data before counter trust. |
| Command counter | MY VERSION BETTER | Per-device expected sequence, 63→1 rollover, opcode classification, duplicate/gap detection and desynchronization after ambiguous operations. Explicit reset/resynchronization required. |
| Wake/recovery | COMBINE BOTH | Lead reference/reset awareness and retry scheduling; finite waits, checked HAL starts and recovery that never refreshes data by itself. |
| Cells | COMBINE BOTH | Normal/even/odd acquisition stages complete before coherent snapshot publication; failed cycles preserve previous values but invalidate control use. |
| AUX/temperature | COMBINE BOTH | Direct AUX acquisition, logical profile routing, raw-voltage validity separate from Celsius. Documented lead conversion table retained behind calibration confirmation. |
| Open wire | LEAD BETTER | Normal/even/odd S-ADC checks retained and included transactionally; ratio remains a hardware qualification item. |
| Validity/freshness | MY VERSION BETTER | Separate timestamps/generation/trust metadata, sentinels and immediate control inhibition; independent bounded AMS ride-through retained. |
| IVT-S | COMBINE BOTH | Strict pure decoder extended to I/U1/U2/Wh, exact DLC and metadata, independent counters/age, global-error inhibition, restart epochs and verified configuration responses. |
| CAN architecture | COMBINE BOTH | Three lead logical buses and scheduler; strict receive validation and transport invalidation. Current charger gateway bitrate/protocol replace inherited Elcon code. |
| Inverter | LEAD BETTER | DTI DC discharge/brake limits; VCU continues to own propulsion. Added validity, profile and finite-value gates. |
| 80 kW | LEAD BETTER / defect fixed | Resistance-aware conservative instantaneous ceiling in both modes. Former credit-spending boost removed; 500 ms average remains diagnostic. Previously excluded random floor regression is mandatory. |
| Regen | NEEDS FIX | Charge temperature window, coldest/hottest sensors, cell OV permission, fresh IVT/acquisition and bounded 35 A ceiling. Drive evaluated independently. |
| Precharge | NEEDS FIX | Rising request, finite timeout, overlap and U1/U2 convergence retained; invalid/stale/implausible inputs abort in CHARGING/CLOSING/ACTIVE. Failure enters shared fault flow; explicit deactivate clears its cause, then normal fault-clear delay applies. Held activate cannot restart. |
| Charger | NEEDS FIX | Current documented gateway BCD setpoint plus separate inhibit heartbeat; status/fault/measurement freshness and charging interlocks. Installed gateway revision still requires validation. |
| Balancing | COMBINE BOTH | Lead hysteresis/thermal budget plus populated-channel masks, profile permission, invalid-data inhibition and verified MUTE/zero configuration/PWM/readback stop. No SPI in ISR. |
| Shutdown | COMBINE BOTH | Lead timed supervisor/events/watchdog plus immediate control inhibit and health causes; early fatal path configures fail-safe GPIO before normal initialization. |
| SoC | LEAD BETTER | Spreadsheet model retained; actual valid no-load current required in firmware and GUI. Model estimate is not a protection permission. |
| Energy | LEAD BETTER | Signed Wh accounting with independent validity, counters, restart epochs and gap/jump rejection. |
| Runtime settings | COMBINE BOTH | Seven bounded scalar settings, CRC flash and idle-only SAVE; protocol v2, exact packets, tokens, response matching and GUI identity/topology gates. |
| Telemetry | COMBINE BOTH | Lead compact frames plus raw AUX, per-board PEC/counter/stop diagnostics, full 32-bit faults, profile/version and data-quality frames. |
| GUI | COMBINE BOTH | Lead Qt architecture integrated into separate fork; old launchers forward to one application. Generated models/DBCs, 112-position display, raw diagnostics, logging, settings and virtual adapter. |
| Testbench | COMBINE BOTH | Lead real-main-loop peripheral simulator plus migrated independent driver contracts, numerical, charger, configuration, transaction, BENCH and end-to-end GUI cases. |

## Configuration and deliberate limits

`Core/Inc/config/` separates topology, buses, vehicle parameters, safety, sensor calibration, IVT and wire protocol. `profiles/` contains physical approval and routing. Default profile 3 is BENCH: one ADBMS6830, eight populated cells, eight raw AUX inputs. Balancing, AIR/bypass/precharge/ready actuation, charger enable, drive and regen are disabled. The fault output remains functional. Each energetic capability requires both profile confirmation and a deliberate build enable. DIGITAL profile 4 is rejected on ARM.

Celsius remains invalid on unconfirmed hardware. The one calibration TODO is to identify and measure the complete sensor/bias/series-resistor transfer function for each board input, then approve its profile. Raw AUX still reaches the GUI. No thermistor-to-cell physical attachment is inferred from a shared index.

Charge/regen protection is 35 A from the supplied 7p × 5 A cell basis. The gateway's six-digit BCD current representation is limited to 9.99999 A, so the runtime charger setting uses a conservative representable 9.9 A maximum at 0.1 A resolution. This is a codec ceiling, not approval of charger current or thermal design. Hardware profiles leave charging disabled.

## Protocol and GUI delivery

See [protocol map](protocol-map.md). `tools/gen_dbc.py`, configuration headers and `BMS_StatusTypeDef` are the canonical definition. `tools/generate.py` produces DBCs, signal CSVs, `registry.json` and vehicle/BENCH GUI models. `tools/sync_gui_protocol.py` copies/checks the bundle in the separate fork. Strict decoding and actual production CAN trace replay verify the wire layout.

The historical 28 overlaps were incompatible schemas for the same telemetry region, not 28 simultaneous arbitration collisions in the final architecture. Retiring the old GUI database resolves their interpretation. Registry uniqueness is checked per physical bus and frame type; equal numeric IDs on separate buses are legal.

GUI additions include raw AUX and board health, PEC/counter counts, full fault names, U1/U2, independent voltage sum/disagreement, acquisition validity, precharge/charger state and limits. `--model bms_model_bench.json` selects eight positions. Settings writes require fresh matching protocol/configuration versions and topology. Logging remains local candump, with DBC CSV export. GUI failure/disconnection cannot grant firmware permissions.

## Defects corrected and regression integrity

Production fixes cover transactional publication, ignored/reset ADC values, command-counter ambiguity, first daisy-chain poll propagation, unchecked/bounded transport and timer starts, IVT duplicate/gap/status/restart/configuration trust, U2 loss in later precharge states, precharge fault reporting, charger incompatibility, charge-temperature regen, charge-flow temperature protection, stale-control use, balancing stop/readback and early fatal outputs. Numerical regressions cover all 65,536 ADC codes and full-width IMD capture arithmetic; division/publication is deferred out of the capture ISR. Generator numeric parsing preserves hexadecimal digits and C integer division.

Simulator corrections preserve monotonic time when a callback consumes time and deliver actual transmitted frames to listeners after bounded log storage fills. Healthy soak drift now respects the documented charge-current/window limits. The soak VCU issues explicit deactivation after FAILED and polls more promptly; all real-fault reaction assertions and the >10 completed-precharge assertion remain. A transient global IVT error during active precharge is now an intentional precharge fault, independently tracked in a bounded input-event window. Historical wrapper fixtures remain as provenance; their entrypoints invoke final-production runners. No safety regression is intentionally excluded.

## Verification evidence

Final machine-readable results and build logs are under ignored `test/build/` and `build/`. [Verification results](verification-results.json) records final totals. All local gates below passed. Independent fork CI additionally runs the same checks; its Linux Qt job explicitly installs EGL/OpenGL runtime libraries so missing GUI dependencies cannot be mistaken for skipped passing tests. Windows Application Control rejected the full native simulator (4551); the same production sources are tested using the existing Ubuntu WSL GCC toolchain. No application-control policy was changed. ARM builds use installed STM32 GCC with `-Wall -Werror` and additional production warnings.

Required checks: all digital scenarios in normal/UBSan/pessimistic/tick-wrap modes; mandatory power-window variant; independent migrated IVT/counter/transport contracts normal and UBSan; BENCH contract; ARM Debug/Release for BENCH and lead profiles; ARM/host register layout; strict DBC actual-traffic decoding; spreadsheet SoC comparison; generation/synchronization; GUI synthetic/virtual/production-trace tests.

## Implementation sequence and remaining work

The software sequence follows the plan: preserve baselines → split configuration/profiles → port transport/counter trust → transactionally stage acquisition/open-wire/AUX → strengthen IVT → unify fault/control inhibition → reconcile registry/charger → harden inverter/regen/power → harden precharge → merge balancing → version settings → integrate GUI/model/logging → add digital/BENCH contracts → verify, document and push separate fork branches.

Physical execution remains prioritized: staged single-board communication first; raw-input and IVT comparison second; calibration and shutdown timing third; charger/inverter protocol and timeout confirmation fourth; thermal/output-driver/latch/full-chain qualification last. See [hardware validation](hardware-validation-pending.md) and [bench guide](TOMORROW-BENCH-BRINGUP.md). Physical actuation requires an approved profile and separate deliberate enables; the software integration does not provide that approval.

## Changed-file inventory

[integration-changed-files.json](integration-changed-files.json) lists every changed/added/deleted path in both independent repositories relative to their original baselines, excluding ignored build outputs. It also records the donor revision. Firmware modules, generators, tests, generated protocol and documentation are in this repository; GUI application/library/tests and its generated bundle are in the GUI fork.

## Final verification totals

| Gate | Result |
|---|---|
| Real-firmware digital scenarios | 91/91 normal, 91/91 UBSan, 91/91 pessimistic, 91/91 tick-wrap = **364/364** |
| Mandatory power-window variant | 7/7 in each of four modes = **28/28** |
| Migrated independent contracts | 3 executables normal + 3 UBSan = **6/6** (15 named contract groups per mode) |
| Single-board BENCH | **1/1** |
| Profile compile acceptance/rejection | **9/9** |
| Datasheet register/command layout | 7 ARM + 7 host vectors = **14/14** |
| GUI | **18/18**, no skips; virtual CAN and actual production telemetry replay included |
| ARM Debug/Release | BENCH profile3 and lead profile1 = **4/4**, zero production warnings/errors |
| Actual-traffic strict DBC decode | **4/4** modes; each includes 120 burst frames, 50 fast frames and 8 settings responses |
| Spreadsheet SoC reference | **4/4** modes, 301 points each; worst error 0.00005 percentage points |
| Canonical generation / GUI bundle | **PASS**, byte-identical release artifacts |

The digital matrix also includes all 16 lead fault-injection sweeps, three 600-second soak seeds per mode, counter/PEC/stale/IVT/charging/precharge/balancing regressions, and exhaustive signed ADC conversion coverage. Former excluded floor regression passes normally. Machine-readable names/results and production source hashes are in `verification-results.json`. Ignored build folders retain full UART/results/build logs. Git's Windows line-ending notices are unrelated to compiler diagnostics; release bundle line endings are pinned in `.gitattributes`.

## Delivery status

- SOFTWARE INTEGRATION: COMPLETE
- DIGITAL VERIFICATION: PASS
- GUI INTEGRATION: PASS
- SINGLE-SLAVE BENCH PROFILE: READY
- PHYSICAL HARDWARE VALIDATION: PENDING

Here READY means the conservative one-board software profile, artifacts and staged procedure are available. It does not grant calibration/output approval. No competition-ready or hardware-validated claim is made. Remaining physical approval tasks are in the linked hardware register.
