# Staged single-slave BENCH bring-up

Use firmware profile **3 (BENCH)** and GUI `bms_model_bench.json`. The intended bench is STM32 → SPI → ADBMS6822 → **one** ADBMS6830 with eight populated cell inputs and eight raw AUX inputs. No fourteen-board chain is required. This procedure validates communication and measurement incrementally; it does not approve a vehicle or energetic outputs.

## Before power

Keep actual AIR/bypass/precharge/ready relay coils, charger power stage, inverter power stage and balancing test loads disconnected. All energetic firmware permissions remain off. Do not set physical confirmation flags or `BMS_ENABLE_*` flags merely to make AMS go healthy. BENCH can correctly report an asserted AMS fault because Celsius is unconfirmed or IVT is absent while cell/raw-AUX communication works.

Identify the actual board revisions and label every supply, return and sense connection from the matching schematic before connection. Use a current-limited isolated low-voltage cell simulator/resistor ladder for cell tests. Do not use the full vehicle pack for this sequence. The numerical 20..33.6 V BENCH plausibility window is software configuration, not approval of an arbitrary supply or evaluation board power connection.

Sources: current supplied MB1367-G474RE-C04 schematic; EVAL-ADBMS6822 revision-B design files; EVAL-ADBMS6830BMSW supplied schematic/design support files; ADBMS6822/6830B datasheets; IVT-S datasheet; current lead connection design. Pin locations below are known NUCLEO routing. Confirm the physically installed header orientation/revision. Connector pin numbers absent from the supplied design are intentionally not invented.

## Commands and files

From firmware root:

```powershell
python tools/generate.py --check
python tools/build_arm.py debug --profile 3
python tools/build_arm.py release --profile 3
python tools/run_digital.py --profile 3 bench_contract
python tools/sync_gui_protocol.py C:\Users\SR\BMS-Integration\bms-gui-2026 --check
```

The digital simulator may need Ubuntu/CI where Windows Application Control blocks its generated executable. On this machine:

```powershell
wsl -d Ubuntu -- sh -lc 'cd /mnt/c/Users/SR/g474-bms-2026 && python3 tools/run_digital.py --profile 3 bench_contract'
```

ARM ELF: `build/debug-profile3/g474-bms.elf`; Release: `build/release-profile3/g474-bms.elf`. Program the Debug ELF through the installed STM32CubeIDE/ST-LINK workflow after confirming profile 3. Do not program a profile-4 digital fixture; ARM compilation rejects it. Preserve the build log and exact Git revisions with the bench log. Build commands do not themselves flash anything.

From GUI root:

```powershell
python -m pip install -r requirements.txt
python bms_gui.py --model bms_model_bench.json --demo
python bms_gui.py --model bms_model_bench.json --interface slcan --channel COM5 --bitrate 1000000 --connect
```

Replace `COM5`/backend with the actual enumerated adapter. `slcan` is one supported backend, not an assumption about the installed adapter. Raw logs are in GUI `logs/`; `python log-reader.py FILE --output decoded.csv` exports with the shipped DBC. Close demo before connecting real hardware.

## Known MCU routing

| Logical signal | STM32 net | NUCLEO Morpho where documented | Bench destination |
|---|---|---|---|
| TSCAN TX/RX | PA12 / PA11 | CN10-12 / CN10-14 | Appropriate CAN transceiver TXD/RXD, not directly CANH/CANL |
| TSACCAN TX/RX | PB13 / PB12 | CN10-30 / CN10-16 | Separate IVT CAN transceiver |
| CHRGCAN TX/RX | PA15 / PA8 | CN7-17 / CN10-23 | Separate charger CAN transceiver; leave power stage disconnected |
| AMS fault | PC1 | CN7-36 | Scope/latch input; open-drain, external 10 kΩ pull-up in lead bench design; high means fault |
| TSAC ready | PC8 | CN10-2 | Scope only, no relay |
| Precharge | PC6 | CN10-4 | Scope only, no relay |
| Bypass | PC9 | CN10-1 | Scope only, no contactor |
| SDC sense | PC2 | CN7-35 | Defined logic through approved low-voltage test fixture; do not float |
| Charger button | PC0 | CN7-38 | Defined inactive level; no charging test here |
| SPI1 SCK/MISO/MOSI | PA5 / PA6 / PA7 | Verify these nets on the installed header/schematic | Same-labelled ADBMS6822 SPI interface |
| isoSPI CS / MSTR | PB6 / PA10 | Verify installed header | Matching ADBMS6822 control nets |
| Unused port CS2 / MSTR2 | PC7 / PB4 | Verify installed header | Maintain documented safe levels; not a second active chain |
| Logic reference | GND / 3V3 | CN7-20 / CN7-16 | Only compatible logic/power interfaces; respect transceiver and isoSPI isolation design |

The exact ADBMS6822 connector pin index must come from its installed board labels/drawing. Do not infer it from an Internet module pinout. CAN requires compatible transceivers and one 120 Ω terminator at each physical end (approximately 60 Ω CANH–CANL unpowered with two terminators). IVT has no internal termination. Verify transceiver power/standby and isolation separately from MCU pins.

## 0. PC and GUI only

**Connections:** None. Start vehicle demo for all 112 positions, then BENCH demo for eight:

```powershell
python bms_gui.py --demo
python bms_gui.py --model bms_model_bench.json --demo
python -m unittest discover -s tests -v
```

**Expected:** Live cell/temperature/IVT/SoC/energy/limits and precharge/charger states, raw-AUX/health diagnostics, readable synthetic faults. Vehicle demo has 112 cells/112 temperatures; BENCH has eight/eight. Settings read/write uses version 2 and tokens. Synthetic confirmed Celsius is a demo fixture only.

**Pass:** Both layouts update; tests pass; local log decodes; mismatch/version tests inhibit writes. **Diagnosis:** Missing Qt/python-can/cantools → install requirements in the interpreter actually used. Wrong layout → select generated model, then synchronize bundle. **Stop:** Do not treat demo values as measurements or connect a demo session to actuators.

## 1. STM32 only

**Connections:** ST-LINK/USB, documented logic power/reference and scope on fault/relay-output nets. No isoSPI/cells/IVT/energetic loads. Define SDC/button input levels using the board's approved logic fixture. Program BENCH Debug ELF.

**Expected:** Boot and bounded retries, watchdog/main-loop activity. Missing acquisition/IVT and unconfirmed Celsius assert fault; ready/precharge/bypass remain inactive. Fault should be high/asserted through reset/startup as supported by the external pull-up/latch design.

**Pass:** Stable boot/retries with all actuation outputs inactive; reset/fatal/watchdog observations recorded. **Diagnosis:** Repeated resets → inspect UART/debugger fault cause, clocks/power and watchdog kick gap. A floating or low fault signal → inspect pull-up/open-drain configuration before proceeding. **Stop:** Unexpected output assertion, supply heating/overcurrent or failure to assert fault.

## 2. CAN and GUI

**Connections:** TSCAN transceiver and adapter, CANH to CANH, CANL to CANL, reference/isolation per adapter design, correctly terminated bus. Adapter must acknowledge real frames. TSCAN is 1 Mbit/s; IVT and charger are separate 500 kbit/s buses.

**Software:** Run the real BENCH GUI command above. **Expected:** Identity B208 reports protocol2/config2/profile3/slaves1/cells8/temps8/physical16/BENCH. Status/quality show expected startup/unavailable inputs. Fast frames may contain unavailable sentinels. GUI reports faults and unavailable values, not plausible zeros. Charger remains inhibited and inverter limits zero.

**Pass:** Received identity matches build; strict DBC decode, setting READ_ALL and local logging work; no outputs become enabled. **Diagnosis:** No frames → check TXD/RXD versus CANH/CANL, transceiver standby/supply, 1 Mbit/s setting and termination. Bus-off/no ACK → inspect adapter and ground/isolation. Inhibited settings with 14-board model → use BENCH model. **Stop:** Unexpected physical ID owner or any energetic permission/output.

## 3. STM32 → SPI → ADBMS6822

**Connections:** Verified compatible SPI/control power domains, PA5 SCK, PA6 MISO, PA7 MOSI, PB6 CS and PA10 MSTR to same-labelled ADBMS6822 nets. Confirm other port control safe states from selected profile and actual drawing. Exact connector indexes remain a drawing check.

**Software:** Same BENCH ELF; scope SCK/CS/MOSI/MISO during bounded retries. **Expected:** Finite chip-select transactions/wake pulses and timeout handling. Without a powered/connected ADBMS6830, no valid cells are expected; a transport fault is correct.

**Pass:** Correct idle/active CS, finite retry waveform and no hang/watchdog starvation; failed communication never marks values fresh. **Diagnosis:** Constant MISO, wrong phase/polarity or long CS → compare CubeMX SPI mode, control straps and scope against ADBMS6822 datasheet. **Stop:** Incompatible logic voltages, bus contention, uncontrolled CS or unbounded communication.

## 4. ADBMS6822 → one ADBMS6830

**Connections:** Documented ADBMS6822 port A isoSPI to EVAL-ADBMS6830BMSW J3 (isoSPI A); J4 is downstream B, unused for a one-board chain. Verify actual board revision/transformer/polarity/termination and power via its supplied schematic. Do not power the analogue board through a guessed connector. Firmware expects exactly one device.

**Expected:** Successful configuration readback, valid PEC and expected command counters, finite conversion/poll cycles. Initially unknown/unsupplied cell inputs are not a passing voltage test. Normal/even/odd plus AUX acquisition must finish coherently before publication.

**Pass:** Repeatable one-board register communication without PEC/CC failures, safe balancing configuration readback zero, and no claimed validity from failed cycles. **Diagnosis:** Reversed isoSPI ports/chain count or incorrect MSTR/termination commonly cause no reply. Increasing PEC/CC counts requires fixing physical communication; do not disable checking. **Stop:** Unexpected balancing state, unexplained register layout or repeated integrity failures.

## 5. Known low-voltage cell inputs

**Connections:** Use the actual EVAL schematic and current-limited isolated cell simulator. CELL0 is the lower reference and CELL1..CELL8 are successive taps. For the supplied eight-position depopulation, CELL9..CELL16 are tied to the CELL8 top node; confirm the actual board's V+ supply path via CELL16/R55/R63 before power. Connector indexes/orientation must be read from the board drawing, not guessed. A 3.70 V-per-position fixture is approximately 29.6 V total.

Keep a resistor ladder intact. **Never open a ladder resistor to simulate a sense-wire break**: voltage redistribution can exceed a cell input's absolute maximum (the ADBMS cell input absolute maximum is not the 4.2 V software trip). For later open-wire injection disconnect only the separate measurement sense lead under the approved fixture procedure.

**Expected:** Eight cell values near independent meter/simulator readings, summed voltage near their sum, raw physical channels 9–16 unused in logical displays. GUI raw data appears only after a completed valid cycle; Celsius can remain invalid and AMS asserted. Verify logical cell 1 and cell 8 order using small distinct safe input values.

**Pass:** Every channel/order/scale agrees within the instrument+datasheet+fixture error budget recorded for this bench; no unexplained PEC/counter errors. No invented universal millivolt tolerance is imposed before fixture accuracy is known. **Diagnosis:** Offset/wrong index → check CELL0 reference, channel map, sequential taps and input filtering. **Stop:** Misordered taps, any input over board/datasheet limits, unexplained total voltage, heating or balancing activity.

## 6. AUX/temperature inputs

**Connections:** BENCH maps TEMP1..TEMP8 to GPIO1..GPIO8. The supplied EVAL J5 exposes GPIO-numbered inputs; verify pin n/opposite V− row against the actual connector orientation. Known resistor/voltage sources must respect input ratings and board bias. GPIO1–3/6–8 use the supplied VREF2 pull-up network; GPIO4/5 are EEPROM I²C lines biased toward VREG with different loading. Do not treat these as identical thermistor circuits.

**Expected:** GUI diagnostics show eight distinct raw AUX voltages and raw-valid mask. Celsius-valid mask is zero; displayed Celsius is unavailable. This is expected until the single calibration approval task is complete. LED outputs remain disabled and cannot overwrite these analogue inputs.

**Pass:** Raw AUX order and voltage agree with independent measurements, board die/VPV diagnostics are plausible, no invalid-to-valid conversion from absent calibration. **Diagnosis:** Unexpected GPIO4/5 voltage → inspect EEPROM pull-up/series network. A known resistor does not directly prove the lead temperature table on EVAL hardware. **Stop:** Input overvoltage or shorting a driven bias. Open/short plausibility thresholds require the approved actual circuit/calibration; do not invent them.

## 7. IVT-S

**Connections:** Verify the exact IVT-S 12/24 V supply/variant and connector drawing. Use TSACCAN's separate 500 kbit/s transceiver/bus with external end termination. Current conductor/shunt direction and negative-side placement follow the supplied IVT installation diagram; U1 is battery-side voltage, U2 vehicle-side voltage. Exact connector pin indexes are absent from the integration evidence and must be obtained from the installed variant drawing. Use only the low-voltage test fixture; no precharge/AIR actuator connection.

**Expected:** 411 commands and exact 511 response echoes; 521/522/523 at 4 ms and 528 at 100 ms after configuration. GUI shows verified configuration and independent I/U1/U2/Wh validity. Compare zero and a known safe current with a meter, confirm discharge-positive sign, and compare U1/U2 with independent measurements. U2 may be a separately defined low-voltage test point; do not leave an assumed input floating.

**Pass:** No status fault, sequential counters, independent values/ages agree with the fixture and instruments. Alive invalidates the epoch and triggers verified reconfiguration. **Diagnosis:** Global bit2 from an unused U3/input still inhibits the sensor: resolve the documented installation rather than masking the bit. CAN ACK without an exact SET response is not configured. **Stop:** Wrong supply/connector, unverified current direction, voltage-input reference or global error semantics.

## 8. Communication fault injection

**Connections:** Keep all energetic loads disconnected. Save a healthy communications log first. Remove one communication path at a time; restore it before the next test. Do not remove powered cell taps casually.

**Tests and expected results:**

| Injection | Expected firmware/GUI result | Pass and diagnosis |
|---|---|---|
| Remove downstream isoSPI connection | Failed cycle invalidates cell/AUX control use, old values cannot become fresh, diagnostics increase, bounded retries; AMS communication/stale causes by configured deadline. | Scope/log reaction and verify PWM/DCC remain off. No hang. Restore requires a new complete acquisition, not just reset. |
| Stop IVT current while voltage continues using an approved CAN fixture | I invalid at 100 ms; U1/U2 keep independent ages; all current-dependent control inhibited. Missing-required-measurement AMS bounded by 450 ms plus supervisor scheduling. | Peer frames must not extend current age. Inspect DLC/mux/counter if restoration fails. |
| Stop U1 or U2 independently | Respective validity expires at 250 ms; invalid U1/U2 aborts any allowed simulated precharge state. BENCH stays off throughout. | No cached voltage use or held-request automatic restart. |
| Restart/power-cycle IVT | Alive clears trusted caches/configuration; exact reconfiguration and fresh independent results required. | GUI reflects epoch/status/trust; no stale immediate recovery. |
| Remove GUI/adapter | Firmware measurement/protection continues. CAN errors/bus-off follow recoverable handling; GUI shows disconnection/stale data. | No GUI-dependent safety permission and no energization. |
| Inject malformed/version-mismatched setting packet in a software CAN fixture | Malformed/incompatible response, no setting change. | Use the regression commands below; do not inject arbitrary vehicle frames on a connected actuator bus. |

Software fault-injection reference:

```powershell
python tools/run_digital.py transaction_contract versioned_config can_long_dlc can_bus_off ivt_spurious_alive ivt_value_extremes precharge_timeout_retry
python tools/run_migrated_contracts.py
```

**Stop:** Any stale value marked healthy, unexpected energized output, integrity checking bypass, unbounded timeout, or measured fault/SDC reaction outside its approved budget. Record the failing trace before changing a profile or threshold. The model's timing pass is not a substitute for this physical measurement.

## Bench record and next approvals

For each stage save hardware/schematic revision, firmware/GUI commits, profile3 build logs, power/fixture values, instrument readings, raw CAN log, expected/actual result and scope captures. Stop at the first failed stage. Keep [hardware-validation-pending.md](hardware-validation-pending.md) as the sign-off list for calibration, actual shutdown timing, charger/inverter compatibility, relay drivers, balancing heat and the full 14-board chain. None is automatically approved by completing this single-slave procedure.
