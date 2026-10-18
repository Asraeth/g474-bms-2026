# GUI / firmware protocol gap

Inspection date: 18 October 2026. **Documentation only; no production code, DBC or Git-history changes.**

This retains the originally requested migration-versus-GUI field comparison and incorporates the subsequently identified lead source. The final subsystem decisions, hardware conflicts, board profiles and implementation order are in [final-integration-plan.md][final-plan].

## Sources and scope

- **M:** current working files in `C:/Users/SR/g474-bms-2026`, HEAD `919a8a8e3f5cf7461b3a6890139e030517976772` plus existing uncommitted improvements. Inspecting HEAD alone would miss the migration's current behavior.
- **G:** `C:/Users/SR/BMS-Integration/bms-gui-2026`, HEAD `dbb91c662bbedbecc0cd9a0e2aecd6da63d386be`; README selects `main6.py` and `databases/bms_can_database.dbc`.
- **L:** actual lead source `C:/Users/SR/Downloads/g474-bms-seb/g474-bms-main`, HEAD `75ea736c4108cb34aff9e7fd8996d6ff643494d7`, current config/specification/DBCs/GUI. It is the `lead-bms` of the request.

Current schematic/PCB, current lead docs, manufacturer data, current rules, lead implementation, migration, tests and historical docs are prioritized in that order. This report does not use the old seven-slave/ADBMS2950 README as the final architecture. The lead controller schematic and charger references contain material conflicts with implementation; see the final plan.

Classification labels are the requested **MATCH, GUI OBSOLETE, GUI MISSING, FIRMWARE MISSING, ID MISMATCH, SCALING MISMATCH, ENDIAN MISMATCH, TOPOLOGY MISMATCH**. Multiple labels identify independent issues. MATCH is limited to the specified field/format, not a claim that the whole system interoperates. FIRMWARE MISSING identifies an absent capability in the named implementation, including missing validity/metadata fields needed for the final integration.

All CAN identifiers and fault bitmask values below are hexadecimal; signal sizes and scaling factors are decimal.

## Receive path, adapter, databases and topology

| Item | Current migration M | GUI fork G | Classification / lead reconciliation |
|---|---|---|---|
| CAN hardware/library | Classical CAN via STM32 FDCAN1 vehicle/charger, FDCAN2 IVT | python-can `can.Bus(interface="slcan", channel=usb_can_path, bitrate=bitrate)`; tested-adapter path names WeAct Studio USB2CAN | **MATCH** in classical CAN capability. Adapter and channel are hard-coded Linux defaults in `main6.py`; choose COM/device/backend in final UI. [G-ui] |
| Vehicle bitrate | 250 kbit/s | `main6.py`:250 kbit/s | **MATCH** M/G. **GUI OBSOLETE** for L CAN1=1 Mbit/s. All physical participants must match. |
| Sensor/charger buses | M IVT isolated 500 kbit/s; charger shares vehicle bus | One adapter on one bus | **GUI MISSING** direct separate-bus diagnostics. L has logical vehicle 1 Mbit/s / IVT 500 kbit/s / charger 250 kbit/s; supplied PCB only shows two transceivers, so physical architecture remains unresolved. |
| Thread handoff | HAL RX callbacks; TX buffer/timer | `can.Notifier` -> `CANListener` -> thread-safe queue -> Tk `after(100)` drains/decode/widget updates | **MATCH** as a suitable receive/display pattern; no frame-level trust validation added by this queue. |
| DBC usage | Hand-packed telemetry; no migration DBC supplied | `cantools.database.load_file`, `decode_message(msg.arbitration_id,msg.data)` | **GUI OBSOLETE**. Selected DBC is for old schema; old decoding keys only by numeric ID rather than explicitly checking extended/data/FD metadata. |
| DBC copies | M source is the actual wire contract | Root DBC: 121 messages / 592 signals; selected database: 137 / 636, including 16 inverter messages; generator uses 7x16 | **GUI OBSOLETE**. The two copies differ; L generated CAN1 DBC: 82 messages / 691 signals; IVT: 6 / 20; charger: 2 / 9 should seed the final versioned bundle. |
| Cells per board / boards | 14 boards x8 logical cells | 7 segments x16 rows | **TOPOLOGY MISMATCH**. Both totals are 112; equal totals do not preserve board/cell identity. L is also 14x8. |
| Cell count | 112 measurement positions | 112 displayed cells | **MATCH** total only. Parallel cells per position are not individual CAN channels. |
| Temperature count | 8 logical inputs per board, 112 total, plus 14 die temps | One cell-temperature field for each of 112 cells, plus 7 die temps | **TOPOLOGY MISMATCH** despite 112 external values in both. Temperature sensors and cells require independently labelled indices; L has 14x8 external and 14 die. |
| Temperature physical order | TEMP1..8 -> GPIO9, 7, 8, 10, 4, 6, 5, 3 | No GPIO model; inherits signal names/cell labels | **GUI MISSING** profile/connector metadata. L candidate instead GPIO1..8, LEDs GPIO9/10. Neither candidate becomes permanent; final map belongs in one board profile. |
| Board display voltage range | Eight-cell board; provisional 24..33.6 V limits | Widget shade assumes 16 cells at 48..67.2 V | **TOPOLOGY MISMATCH**, **GUI OBSOLETE**. Final display ranges come from selected policy/model, not grid dimensions. |

## Migration cell / temperature frame: every field

M sends extended classical DLC 8 **`0xB000 + 8*b + c`**, `b=0..13`, `c=0..7`; range B000..B06F. Numeric fields are signed little-endian. G expects **`0xB000 + 16*s + c`**, `s=0..6`, `c=0..15`, same numeric ID range. G labels are `CELL_{s+1}x{c+1}_...`. [M-acq], [G-dbc]

| Field | M encoding and meaning | G expected decode | Classification |
|---|---|---|---|
| CAN ID / logical cell identity | Board=floor((ID-B000)/8)+1; cell=(ID-B000)%8+1 | Segment=floor(offset/16)+1; cell=offset%16+1 | **TOPOLOGY MISMATCH**. For example, B008 is M board 2 cell 1, G segment 1 cell 9. No numeric ID-range mismatch in this particular M/G comparison. |
| Bytes 0..1 | int16 cell voltage, 1 mV/count ->0.001 V | Signed16 LE, 0.001 V | **MATCH** format/scale/endian. M invalid=8000h, G displays -32.768 V as a number. |
| Bytes 2..3 | int16 voltage minus **pack minimum**, 1 mV/count | Signed16 LE, 1 mV/count | **MATCH** scale. Reference is pack minimum in M; L GUI computes delta from mean, so preserve an explicit label rather than treating these as the same statistic. |
| Bytes 4..5 | int16 TEMP(c+1), 0.01 C/count; current M uses INT16_MIN because conversion unconfirmed | Signed16 LE, 0.01 C/count | **MATCH** encoding; **GUI MISSING** invalid interpretation. Raw 8000h becomes -327.68 C in old GUI. Raw input acquisition does not imply calibrated Celsius. |
| Byte 6 bit 0 | This logical cell's commanded balancing bit | `isDischarging` | **MATCH** bit position; neither bit proves physical resistor current or successful readback. M balancing interlock keeps it off in current build. |
| Byte 6 bit 1 | Cell fault summary | `isFaultDetected` | **MATCH** position, **GUI MISSING** causes. No OV/UV/temperature/open-wire cause decomposition in this old frame. |
| Byte 6 bits 2..7 | No additional signals defined | Not decoded | **MATCH** as unused; do not allocate silently. |
| Byte 7 bit 0 | TEMP input voltage acquired/valid | Ignored | **GUI MISSING**. Raw sensor volts are retained internally but not sent numerically in this frame. |
| Byte 7 bit 1 | Celsius conversion valid | Ignored | **GUI MISSING**. Always false with the unconfirmed current M conversion. |
| Byte 7 bit 2 | Cell voltage valid/fresh | Ignored | **GUI MISSING**. Faulted/stale numeric sentinels cannot be treated as current values. |
| Byte 7 bits 3..7 | No defined extra state | Ignored | **MATCH** as unused. |
| Raw temperature input volts | Internal separate AUX input array | No signal/widget | **GUI MISSING**, **FIRMWARE MISSING** M on-wire numeric field. Retain explicit diagnostic access in final schema. |

## Migration board frame: every field

M sends **B070..B07D**, fourteen boards. G expects **B070..B076**, seven segments, same field format for those first seven IDs. [M-can], [G-dbc]

| Field | M | G | Classification |
|---|---|---|---|
| Board ID/name | B070+b, b0..13 | B070+s, s0..6 | **TOPOLOGY MISMATCH**, **GUI MISSING** boards 8..14. B077/B078 additionally collide with G pack/status meanings. |
| Bytes 0..3 | int32 VPV, 1 mV/count | int32 LE 0.001 V | **MATCH** scale/endian for recognized board frames; G lacks INT32_MIN handling. VPV is a measured board supply voltage, not a 16-cell sum. |
| Bytes 4..5 | int16 die temperature, 0.01 C/count | int16 LE 0.01 C | **MATCH**, **GUI MISSING** invalid sentinel. |
| Byte 6 bit 0 | Comms error or stale cell/AUX state | `SEG_n_isCommsError` | **MATCH** bit for decoded boards; no PEC/counter/timeout cause detail. |
| Byte 6 bit 1 | Board fault summary | `SEG_n_isFaultDetected` | **MATCH** bit for decoded boards; no complete fault definitions. |
| Byte 7 bit 0 | VPV valid | Ignored | **GUI MISSING**. |
| Byte 7 bit 1 | Die temp valid | Ignored | **GUI MISSING**. |
| Byte 7 bit 2 | Cell voltage valid | Ignored | **GUI MISSING**. |
| Byte 6 other bits / byte 7 other bits | No extra signals defined | Ignored | **MATCH** as unused. |

## IVT / pack / diagnostic / control capabilities

| Item | M actual behavior | G expectation | Classification and L disposition |
|---|---|---|---|
| IVT result IDs | Standard 0x521 current, 0x522 U1; response 0x511; isolated bus 500 kbit/s | No raw IVT DBC/support | **GUI MISSING**. L also consumes 0x523 U2 and 0x528 Wh; these are **FIRMWARE MISSING** in M. |
| Raw IVT payload | DLC 6, DB0 mux 0/1, DB1 high-nibble status / low-nibble 4-bit counter, signed 32-bit BE DB2..5 in mA/mV | No decoder | **GUI MISSING**. M strict checks/counters/status better than L receive path; preserve when extending to U2/Wh. Raw sensor BE and vehicle telemetry LE are separate contracts. |
| M vehicle current B07E | Bytes 0..3 signed 32-bit LE mA; byte 4 raw DB1; byte 5 counter; byte 6 flags; byte 7 zero | Old pack current is B077 bytes 2..3 signed 16-bit LE, 0.01 A | **ID MISMATCH**, **SCALING MISMATCH**, **GUI MISSING**. M is 0.001 A/count and 32-bit. |
| M vehicle U1 B07F | Same layout, signed 32-bit LE mV | Old pack voltage B077 bytes 0..1 signed 16-bit LE, 0.1 V | **ID MISMATCH**, **SCALING MISMATCH**, **GUI MISSING**. M is 0.001 V/count and 32-bit. |
| B07E/B07F byte 6 bit 0 | Result valid | No equivalent | **GUI MISSING**. |
| B07E/B07F byte 6 bit 1 | Stale, including never measured/reset | No equivalent | **GUI MISSING**. |
| B07E/B07F byte 6 bit 2 | Result ID has been received | No equivalent | **GUI MISSING**. Receipt is not validity. |
| B07E/B07F byte 6 bit 3 | Configuration verified | No equivalent | **GUI MISSING**. M never verifies sensor config; flag remains false. L sends configuration but also needs verified-state separation. |
| IVT status semantics | OCS retained; status bits 1/2/3 invalidate according to driver policy; bit 2/3 inhibit cached peer; duplicate doesn't extend freshness | No fault definitions or counter display | **GUI MISSING**. Preserve raw status and independent channel quality in final model; do not blindly import L 0x0A mask. |
| B080 bytes 0..3 | Signed 32-bit LE cell-voltage sum mV, independent of IVT | No message | **GUI MISSING**. This ID is L TEMP1..4 and cannot keep its M meaning in final protocol. |
| B080 bytes 4..7 | Signed 32-bit LE U1-minus-cell-sum mV; sentinel when unavailable | No message | **GUI MISSING**. M has no approved discrepancy fault threshold; L span-based plausibility is a useful contribution. |
| Old PACK B077 | Actually M board 8 frame | `BMS_Pack_Voltage` int16 0.1 V/current int16 0.01 A | **ID MISMATCH**, **GUI OBSOLETE**. Decodes board supply bytes as unrelated pack values. |
| Old PACK_STATUS B078 | Actually M board 9 frame | byte 0 bits 0/1 master comms/fault | **ID MISMATCH**, **GUI OBSOLETE**. Can show arbitrary board-voltage bits as master status. |
| Full fault/status word | M enum 01 comms/02 voltage/04 temp/08 current/10 IVT; not broadcast as full enum in legacy telemetry | Per-cell/board two booleans, obsolete master flags | **GUI MISSING**, **FIRMWARE MISSING** M full on-wire fault word. L B201 includes open-wire 10 / IVT 20 / startup 40, so raw M bit values cannot be reused. |
| Precharge state | No M state machine | None | **FIRMWARE MISSING**, **GUI MISSING** relative to final system. L B201 state 0 idle / 1 charging / 2 closing / 3 active / 4 failed, plus percentage; resolve hardware ownership/state defects. |
| Inverter current/power/regen limits | No M inverter module | Inverter DBC decodes other-node traffic only; no dedicated BMS-limit widgets/control | **FIRMWARE MISSING**, **GUI MISSING**. L B206 and 0AFF/0BFF supply intended functionality. |
| Inverter IDs/format | M sends none | Selected old inverter DBC node 14 (hex), e.g. 0A14/0B14 DLC 2 signed 16-bit BE 0.1 A | **ID MISMATCH** with L default node FF: 0AFF/0BFF DLC 8. Actual node/DLC/device setup must be reconciled. |
| SoC | Not implemented | None | **FIRMWARE MISSING**, **GUI MISSING**. Retain L spreadsheet-derived qualified rest estimate and B203 at 0.1%. |
| Energy/instant power | Not implemented | None | **FIRMWARE MISSING**, **GUI MISSING**. L IVT Wh accounting / B203/B205 adds function; fix Wh validity first. |
| Charger command | Extended 1806E5F4, unsigned 16-bit BE 0.1 V/0.1 A, byte 4 disable = 1; other bytes zero | No charger decoder/control in active BMS grid/DBC | **GUI MISSING**. Comments claiming LE / enable = 1 are inconsistent with actual shifts. |
| Charger status | Extended 18FF50E5, unsigned 16-bit BE voltage/current at 0.1 V/A, byte 4 bits 0 hardware / 1 overtemperature / 2 input voltage / 3 charging state; exact DLC/frame gate | Unknown raw frames; filter utility selects these IDs from ASC logs | **GUI MISSING**. Status age/fault interlocks incomplete in both M/L. Current charger refs use a different protocol; inherited Elcon IDs are not final authority. |
| Configuration upload | No M BMS settings receiver, transaction/flash service or response protocol | No settings uploader | **FIRMWARE MISSING**, **GUI MISSING**. L B301 WRITE / B207 reply with seven bounded settings is starting point. |
| Configuration download | No runtime readback | No READ/READ_ALL support | **FIRMWARE MISSING**, **GUI MISSING**. Port L settings model/client after exact DLC/unique-response fixes. |
| Firmware update / bulk profile upload | No application CAN bootloader/profile-upload protocol found | None | **FIRMWARE MISSING**, **GUI MISSING** if later required; separate from seven scalar runtime settings. No implementation is proposed implicitly. |

## Lead protocol versus old GUI: field reconciliation

| Final baseline field group | Lead specification | G classification |
|---|---|---|
| Packed voltage frames | B000..B01B, four signed 16-bit LE mV values per frame; cell index `4*k+j+1` | **GUI OBSOLETE**, **TOPOLOGY MISMATCH**. Voltage resolution matches, but byte offsets 2/4/6 contain other cells, rather than delta/temperature/flags. |
| Packed TEMP frames | B080..B09B, four signed 16-bit LE values at 0.1 C/count | **ID MISMATCH**, **SCALING MISMATCH**, **GUI MISSING**. Old 0.01 C layout is ten times finer and in different messages. |
| Slave voltage/die | B100..B10D, signed 16-bit LE at 0.01 V / 0.1 C | **ID MISMATCH**, **SCALING MISMATCH**, **TOPOLOGY MISMATCH** versus seven old board frames with 32-bit mV and centidegrees. |
| Slave masks | Byte 4 balancing, 5 cell faults, 6 open wire; byte 7 bits 0/1 comms/board fault | **GUI MISSING**. Old cell flags occupy byte 6 bits 0/1; positions cannot be preserved. |
| Pack U1/current/cell sum | B200 signed 16-bit LE at 0.1 V / 0.1 A / 0.1 V; byte 6 timeout/error flags | **ID MISMATCH**, **SCALING MISMATCH**, **GUI MISSING**. Old current scale 0.01 A would decode ten times too small if copied mechanically. |
| Status/fault/output/ready | B201 byte 0 fault bits; byte 1 output/ready/balancing/charging/SDC/bench/derating/power-limit flags | **GUI MISSING**. A Boolean master fault is insufficient. |
| Measurement age/cycle | B201 bytes 2..3 / 4..5, signed 16-bit ms | **GUI MISSING**. Combine measurement age with frame-receipt age. |
| Precharge state/ratio | B201 byte 6 state, byte 7 percentage | **GUI MISSING**. |
| Min/max voltages/temperatures | B202 signed 16-bit mV / mV / 0.1 C / 0.1 C | **GUI MISSING**. |
| Power/remaining/SoC/used | B203 signed 16-bit at 2.5 W; uint16 Wh; uint16 at 0.1%; signed 16-bit Wh | **GUI MISSING**; unsigned FFFF means unknown. |
| Fast I/U1/U2/power/remaining | B204 signed 32-bit mA and mV; B205 signed 32-bit mV, signed 16-bit at 2.5 W and uint16 Wh | **GUI MISSING**. Full quality diagnostics are also **FIRMWARE MISSING** in current L frames. |
| Limits | B206 power at 2.5 W, discharge/regen magnitude at 0.1 A, flags and average at 500 W | **GUI MISSING**. |
| Settings request/reply | B301/B207, DLC 8, byte 0 command / 1 setting; signed 32-bit LE value in bytes 4..7; response byte 2 result / 3 flags | **GUI MISSING**. Current L code/client need malformed/duplicate-response fixes. |
| Endian at correct modern field | L BMS telemetry LE; G BMS DBC LE | **MATCH** endian only. Byte-swapping the modern BMS payload is not a general fix. |
| Historical `main.py` decoder | Uses `struct.unpack('>h', data[:2])`, placeholder base ID 069, manual format | **ENDIAN MISMATCH**, **ID MISMATCH**, **GUI OBSOLETE** against M/L voltage fields. This differs from README-selected `main6.py`. |

Concrete verified collision: lead B000 `A6 0E B0 0E BA 0E C4 0E` means 3.750/3.760/3.770/3.780 V. G's actual DBC decoder successfully produces 3.75 V, 3760 mV difference, 37.7 C and two flags from the fourth voltage. Matching ID and successful decode are insufficient evidence of compatibility. The audit found 28 such lead/G ID collisions. [protocol-evidence]

## Logging and historical entry points

| Capability | Finding | Classification |
|---|---|---|
| Active raw disk logging | `main6.py` notifier includes `can.CanutilsLogWriter`; timestamped `logs/can_log_*.log`, all received frames, independent of decode | **MATCH** as a useful raw CAN record. Preserve with protocol/profile/build metadata. |
| CAN list | Main6 keeps one updated row per numeric ID, maximum 500 IDs; known signals decoded, unknown payloads in hex | **MATCH** basic diagnostic function; **GUI MISSING** separate frame-format/bus keys and measurement trust. This list is not the full disk log. |
| Plot history | Every decoded value appended without a bound; only the last 500 points are sliced when plotting | **GUI OBSOLETE**. L store uses bounded 1200-point deques and a minimum 0.05 s sampling interval. |
| Stale readouts | Old numbers stay displayed after traffic stops; no firmware-validity interpretation | **GUI MISSING**. Retain untrusted frames in raw logs while marking unavailable data in the UI. |
| Offline analysis | `log-reader.py` uses python-can LogReader, cantools and Matplotlib with hard-coded files, time filters and old signal names | **GUI OBSOLETE** schemas; retain workflow with version-selected decoders. |
| ASC filter utility | `log_lines_filter.py` selects 1806E5F4x/18FF50E5x from ASC text | **MATCH** for those historical IDs; **GUI OBSOLETE** for the custom new charger unless a gateway retains Elcon. It is not a configuration client. |
| Earlier apps | `main2.py`: basic 500k ASC logger; `main3.py`: 500k DBC viewer; `main4.py`: 250k cell grid; `main5.py`: 250k 7x16 segment grid; `main6.py`: recommended | **GUI OBSOLETE** as competing launchers. Historical main.py's big-endian 069 format differs from main6's DBC. |
| Test send utility | `can-test.py` sends IDs 1/2 on 500k; not a BMS configuration tool | **GUI OBSOLETE** for the final protocol. Inspected; not run against hardware. |

## Reconciliation outcome

Use the lead's current packed CAN architecture and generated model as the baseline, subject to hardware/current-charger corrections. Port the migration's trust/validity/diagnostic improvements into that schema. Deliver the GUI in its existing separate repository using the richer lead GUI/model/link/settings implementation as a starting point, retaining useful fork logging/plot workflows. Add protocol/profile/version negotiation and captured-frame tests before enabling settings writes; retain legacy decoding only for explicitly selected historical logs.

Read-only checks completed: six DBCs loaded; 28 lead/G collisions confirmed; 14x8x8 model validated; 50 kW settings encoding matched the DBC; duplicate READ_ALL client defect reproduced; 12 lead GUI offscreen/virtual tests passed; lead generator `--check` passed. No complete firmware rebuild, CAN transmission to hardware or flash was performed. [protocol-evidence], [gui-test-log]

<!-- SOURCE REFERENCES -->
[G-dbc]: <C:/Users/SR/BMS-Integration/bms-gui-2026/databases/db-maker.py:9>
[G-ui]: <C:/Users/SR/BMS-Integration/bms-gui-2026/main6.py:301>
[M-acq]: <C:/Users/SR/g474-bms-2026/Core/custom_lib/src/bms_libWrapper.c:540>
[M-can]: <C:/Users/SR/g474-bms-2026/Core/Inc/bms_can.h:17>
[final-plan]: <C:/Users/SR/Downloads/g474-bms-seb/g474-bms-main/docs/final-integration-plan.md>
[gui-test-log]: <C:/Users/SR/.codex/visualizations/2026/10/18/01a15042-bb06-73f1-8df1-7cf3c83766f3/bms-audit/lead-gui-tests.log>
[protocol-evidence]: <C:/Users/SR/.codex/visualizations/2026/10/18/01a15042-bb06-73f1-8df1-7cf3c83766f3/bms-audit/protocol-checks.json>
