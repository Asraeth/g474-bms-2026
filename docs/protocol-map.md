# Final BMS protocol map

Wire version 2; configuration version 2. Authoritative machine-readable output: [`can/registry.json`](../can/registry.json), including every signal's start bit, width, endian, signedness, scale, unit, period and ownership. Source: `tools/gen_dbc.py`, `Core/Inc/config/`, `Core/Inc/bms_can.h` and the fault enum in `main.h`. Generate with `python tools/generate.py`; verify with `--check`. Copy/check the GUI bundle with `python tools/sync_gui_protocol.py PATH_TO_GUI [--check]`.

## Physical buses and ownership

| Bus | STM32 | Bitrate | Participants | Interpretation |
|---|---|---|---|---|
| TSCAN | FDCAN1 | 1 Mbit/s | BMS, VCU, GUI/logger, DTI inverter | BMS telemetry/settings plus inverter electrical limits; VCU owns activation/propulsion requests. |
| TSACCAN | FDCAN2 | 500 kbit/s | BMS, IVT-S | Standard IVT command/response/results. |
| CHRGCAN | FDCAN3 | 500 kbit/s | BMS, documented charger gateway | Extended gateway frames; inherited Elcon/TC protocol retired. |

All are classical CAN. No RTR or FD application payloads are accepted. Topology and payload length are checked. Numeric ID reuse on separate buses is legal. The 28 old GUI/firmware overlaps were misleading incompatible interpretations; the old database is removed from active use. Generator and GUI tests reject duplicate `(bus, ID, standard/extended)` registry keys. Vehicle assignments/node/bus physical wiring still require confirmation.

## TSCAN telemetry

All BMS telemetry is extended ID, DLC8, little-endian. Device/slave `s` and frame `f` below are zero-based; GUI segment/cell/sensor labels are one-based. Default vehicle layout is 14×8; BENCH emits only slave 0. ADBMS register channels remain physical 1–16, six bytes per register. Logical populated cells map to physical 1–8 in the selected profile.

| ID | Period | Content |
|---|---|---|
| B000 + 2s + f, f=0..1 | 100 ms | Four signed int16 cell mV; logical cells 4f+1..4f+4. |
| B080 + 2s + f | 100 ms | Four signed int16 logical Celsius ×10. Unconfirmed Celsius remains invalid. |
| B0C0 + 2s + f | 100 ms | Four signed int16 raw AUX mV in TEMP1–TEMP8 logical order. |
| B100 + s | 100 ms | Slave voltage 0.01 V, die 0.1 C; balancing/fault/open-wire masks, comms/slave flags. Bits 0..7 refer only to populated logical cells. |
| B180 + s | 100 ms | PEC count u16, counter-error count u16, raw/Celsius valid masks, transport fault, balancing-stop failure. |
| B200 | 100 ms | IVT U1 0.1 V, current 0.1 A, summed cell voltage 0.1 V, IVT trust flags. Positive current is discharge in the selected installation convention. |
| B201 | 100 ms | Legacy low eight fault bits, AMS/ready/balance/charging/SDC/BENCH bits, measurement age, loop duration, precharge state and U2/U1 percent. Full faults are in B209. |
| B202 | 100 ms | Min/max cell mV and min/max temperature 0.1 C. |
| B203 | 100 ms | Signed power 2.5 W, remaining Wh, SoC 0.1%, signed used Wh. |
| B204 | 4 ms | Signed int32 current mA, signed int32 U1 mV. |
| B205 | 4 ms | Signed int32 U2 mV, signed int16 power 2.5 W, unsigned remaining Wh. |
| B206 | 20 ms | Power ceiling 2.5 W, drive/regen current limits 0.1 A, limiting/AMS/settings flags, average power 0.5 kW. |
| B207 | On request, max one per 4 ms | Versioned configuration response below. |
| B208 | 100 ms | Protocol/config version, board profile, slave count, cells/slave, temps/slave, physical channels, BENCH bit. |
| B209 | 100 ms | Full u32 fault mask, cell/AUX/Celsius/IVT/config/current/U1/U2/Wh validity bits, acquisition age u16, generation u8. See DBC for exact bit allocation. |
| B20A | 100 ms | Independent IVT status/counter bytes, valid bits, restart epoch and gap count. |
| B20B | 100 ms | Charger state, fault mask, healthy/enabled bits, output voltage/current 0.1. |
| 0AFF / 0BFF | 20 ms | DTI max DC drive/brake current. Signed **big-endian** int16 ×0.1 A; brake negative; unused six bytes FF. Node FF is the unconfirmed broadcast candidate, not a torque request. |
| B300 RX | VCU event | Byte0 0=deactivate, 1=activate. Exact DLC8; only a rising request can start precharge. Timeout default 0 (event semantics); vehicle owner must confirm final lifetime. |
| B301 RX | GUI request | Configuration v2 below. |

Signed unavailable values use minimum signed integer where specified; SoC/remaining energy use FFFF; temperature sentinel is 888 C. Raw/Celsius/quality masks must be checked independently. DBC scaling alone does not grant validity. Failure may retain the last completed values for diagnosis while clearing permission to consume them. GUI display ages out at 450 ms.

## IVT-S on TSACCAN

Standard IDs: 411 command (8 bytes), 511 response/Alive (8 bytes), 521 I, 522 U1, 523 U2, 528 Wh (each 6 bytes). Result byte0 is the mux, byte1 status high nibble/counter low nibble, bytes2–5 signed **big-endian** int32. I is mA, U1/U2 mV, Wh is signed Wh. Counter rollover is 15→0. Duplicate/gap/status errors cannot refresh a healthy timestamp. Global status bits 2/3 invalidate peer caches until each independently receives clean data. Status mask 0E is not silently narrowed for an unused voltage input.

I/U1/U2 periods are 4 ms; Wh 100 ms. Control ages: I <100 ms, U1/U2 <250 ms, Wh uses its independent decoder age. Missing required IVT measurements/configuration reach AMS at the bounded fault deadline; control permissions are removed earlier. Every Alive invalidates the epoch and verified configuration. Configuration is volatile: stop, configure I/U1/U2/Wh cyclic and unused muxes off, then run. Each SET response must exactly echo the eight-byte command with the response opcode; TX/CAN ACK is insufficient.

## Charger gateway on CHRGCAN

Extended DLC8 frames from current supplied gateway documentation:

| Direction | ID | Fields |
|---|---|---|
| BMS TX | 00FFFFFF | Bytes0–2 six decimal BCD digits, MSB-first, V×1000; bytes3–5 six BCD digits, A×100000; bytes6–7 zero. |
| BMS TX | 001FFFFF | AA 00 00 00 00 00 00 00 inhibits. All zero permits only deliberate healthy charging. |
| Gateway RX | 01100000 | State byte0: 0 idle,1 battery,2 precharge,3 active,4 discharge,5 error; fault u16 little-endian bytes1–2; remaining bytes reserved. |
| Gateway RX | 01000000 | Signed LE16 PFC voltage, output voltage, output current, battery voltage, each ×0.1. |
| Gateway RX | 02000000 | Documented power/thermal diagnostics, decoded by generated charger DBC. |

Golden encode: `48 27 35 68 12 34 00 00` = 482.735 V and 6.81234 A. Production encoding rejects non-finite, negative and unrepresentable values. Inhibited setpoint current is zero and the independent fault heartbeat is AA. Status and electrical measurements must both be fresh (<450 ms), state <=3 and fault mask zero. Fault bits are carried verbatim; the DBC names the documented gateway definitions. The bundled binary C gateway example contradicts its current BCD documentation; BCD follows the user's source priority. Charging remains physically disabled until the installed gateway revision and enable semantics are verified.

## Configuration upload/download

This is bounded scalar configuration, not firmware flashing or arbitrary topology/safety upload. Request B301: byte0 command, byte1 setting ID, byte2 config version=2, byte3 token 0..31, bytes4–7 signed LE32 raw value. Response B207: command, setting, result, byte3 low three state flags and upper five token bits, signed LE32 effective value. Commands: 1 Read,2 Write,3 Save,4 Defaults,5 Read_All. Results: 0 OK,1 Unknown_Setting,2 Out_Of_Range,3 TS_Active,4 Flash_Error,5 Unknown_Command,6 Busy,7 Incompatible_Version,8 Malformed. See generated registry for names/bounds and DBC multiplexing.

Settings: power W, drive current 0.1 A, thermal end 0.1 C, regen switch, regen current 0.1 A, charger current 0.1 A, charger voltage 0.1 V. Writes cannot raise built-in electrical ceilings; no runtime safety threshold, GPIO or calibration changes exist. SAVE is refused with TS active or charging, rate-limited and CRC/version checked. GUI permits one request in flight, bounded queue/retries, matching command/setting/token responses and fresh matching identity/topology before writes/save/defaults. Disconnect does not change firmware protection.

## Fault flow

Faults cover COMMS, VOLTAGE, TEMP, CURRENT, OPEN_WIRE, IVT, STARTUP, CAN, CHARGER, PRECHARGE, SENSOR, INTERNAL, PEC, COUNTER, CELL_STALE, AUX_STALE, BALANCE_STOP, CELL_UV, CELL_OV and CHARGE_TEMP. Their exact bit positions are generated in `registry.json`; B209 carries all 32 bits and the GUI uses those names. Immediate control inhibit is separate from persistence/debounce and the physical latch. Precharge failure requires explicit deactivate before its cause clears. Fatal errors assert safe GPIO independently of the normal initialization sequence.
