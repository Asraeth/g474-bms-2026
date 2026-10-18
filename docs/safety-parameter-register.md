# Safety parameter register

Values below are implemented design inputs from the supplied lead documentation/manufacturer material, plus explicitly identified conservative software policy. They are not measured hardware qualification. Exact macros and compile checks are authoritative; runtime settings may lower bounded electrical limits but cannot change protection thresholds, routing or calibration.

| Parameter | Implemented value | Basis / limitation | Location |
|---|---|---|---|
| Vehicle topology | 14×8 populated cells, 8 logical temperatures/device | Current lead architecture; 16 physical cell channels remain represented. | `bms_topology_config.h`, selected profile |
| BENCH topology | 1×8 cells, 8 raw temperature inputs | Explicit single-slave fixture; no complete chain required. | `board_bench.h` |
| Cell UV / OV | 2.50 / 4.20 V | Supplied Enepaq/VTC6 data and lead protection values; exact installed module must be confirmed. | `bms_safety_limits.h` |
| Pack voltage | Vehicle 280..470.4 V; BENCH 20..33.6 V | Derived from UV/OV × actual logical cell count. | Same |
| Discharge temperature | -20..60 C | Supplied cell/lead design and rules ceiling. Sensor attachment and accuracy pending. | Same |
| Charge/regen temperature | 0..45 C inclusive | Supplied cell charge window. Drive and regen evaluated independently; charging current below zero is protected too. | Same, charger/inverter/safety modules |
| Slave die temperature | -40..70 C | Lead board design; validate actual board environment. | Same |
| Celsius conversion | Lead documented voltage table, disabled on unconfirmed physical profiles | One remaining calibration approval: actual sensor + complete bias/series circuit for each board input. Raw AUX remains independent. No invented thermistor equation. | `bms_sensor_calibration.h`, board profile |
| ADC conversion | 1.5 V + signed raw ×150 microvolts; INT16_MIN invalid | ADBMS6830B format. Exhaustive 65,536-code test; full six-byte register payloads. | `bms_adc.h` |
| Discharge / charge trip | 200 / 35 A | Lead discharge ceiling; supplied VTC6 5 A ×7p charge basis. Requires installed pack/fuse/charger approval. | `bms_safety_limits.h` |
| Power ceiling | 80,000 W | Supplied lead vehicle/rules architecture; conservative instantaneous ceiling in both modes. No above-ceiling credit boost. | `bms_vehicle_config.h`, inverter |
| Power averaging / floor | 500 ms diagnostic, 72 kW regression floor | Former optional boost defect fixed by conservative policy; floor is evaluated only where current/thermal/headroom permit. | Same, mandatory power-window tests |
| Pack resistance bounds | 0.15..0.6 ohm | Lead documented conservative bounds; replace only with measured justified range. | Same |
| Inverter update / assumed lag | 20 / 10 ms | Lead architecture; physical DTI application and timeout validation pending. | Same |
| Drive thermal derating | 55..60 C, down to 25 A | Lead electrical limit policy; runtime thermal end can only be lowered in documented range. | Same |
| Regen ceiling | 35 A | 7p charge basis, further inhibited outside 0..45 C or at cell OV. | Same |
| Precharge convergence | >=95% U2/U1 | Supplied lead/rules design. Both voltages must be validated, finite and plausible in every active state. | Same |
| Precharge timeout / overlap | 5000 / 50 ms | Lead design values awaiting measured resistor/capacitance/driver timing. BENCH actuation disabled. | Same |
| TS request timeout | 0 = event semantics | Retained lead behavior; rising edge required, held request cannot restart failure. Vehicle VCU owner must approve lifetime. | Same |
| Limit debounce / decay | 180 ms / 2 | Lead accumulating violation policy; missing data cannot clear a pending violation. | `bms_safety_limits.h`, safety module |
| Safety tick / FS design budget / margin | 10 / 500 / 20 ms | Supplied design budgets, not a physical compliance certificate. Derived latest-good deadline 470 ms. | Same |
| Cell/AUX stale deadline | 450 ms | Retained bounded lead timing, not the migration's provisional 1000 ms. Failed acquisition never refreshes age. Control data is invalidated immediately on failure. | Same |
| Fault clear delay | 1000 ms | All causes must be clear. Physical latch state is separate. Precharge cause requires deliberate deactivate. | Same |
| Measurement target / cycle warning | 100 / 70 ms | Lead schedule, overlapped cell/open-wire/AUX conversion. Qualify full-chain time on hardware. | Same |
| Retry delay / reinit threshold | 10 ms / 3 failures | Lead bounded recovery; reset alone never creates valid data. | Same |
| SPI / S-ADC / AUX deadlines | 10 / 15 / 30 ms | Configurable bounded transport/polling budgets; chain priming included. | Same |
| Wake pulse / inter-device delay | 20 / 600 microseconds | Supplied ADBMS wake design; verify actual chain and clock. | Same |
| IWDG target | 200 ms | Lead watchdog, hardware clock tolerance pending. Simulator models minimum timeout separately. | Same |
| Open-wire ratio | 0.75 | Provisional lead filter-dependent threshold; normal/even/odd checks retained. | Same |
| Balancing threshold / hysteresis | 30 / 10 mV | Lead selection algorithm; permission requires valid cells/AUX/Celsius/IVT and profile approval. | Same |
| Balancing heat / duty / timeout | 4 W per board / PWM<=15 / 1 minute hardware timeout | Lead configurable heat budget and migrated stop discipline; physical thermal/stop validation pending. | Same |
| Balance resistor | Lead/V3 15 ohm candidate, EVAL BENCH 30 ohm | Select by actual PCB/BOM, never infer from software. Hardware approval remains off. | Board profiles |
| CAN1 / CAN2 / CAN3 | 1 Mbit/s / 500 kbit/s / 500 kbit/s | Current lead vehicle/IVT and current charger gateway documentation. CubeMX/code checked by generator. | `bms_bus_config.h` |
| Telemetry burst / fast | 100 / 4 ms | Lead scheduler; generated worst-case BMS share constrained to 50%. | Same |
| IVT I/U1/U2 / Wh periods | 4 / 100 ms | Supplied IVT timing/message budget, total <=1000 messages/s. | `bms_sensor_config.h` |
| IVT control age I / U1/U2 | 100 / 250 ms | Independent trust ages. A valid peer does not refresh a missing channel. | Same, `ivt_s.c` |
| IVT AMS missing budget | 450 ms | Bounded lead restart ride-through separate from immediate control invalidation. | Same |
| IVT status mask | 0x0E | Result error plus global error bits. Unused U3 is not justification to ignore bit2. | Same |
| IVT retry / restart wait | 500 / 300 ms | Bounded reconfiguration with exact response verification. | Same |
| IVT CAN-only auto clear | <=3 attempts/power-up | Retained lead service; other system errors require sensor power cycle. | Same |
| Charger status/measurement age | <450 ms each | Conservative software health policy; both required, no fault and state<=3. | Safety limits/charger |
| Charger default current | 1.0 A | Retained lead default, still physically disabled pending approval. | Vehicle config |
| Charger runtime max | 9.9 A at 0.1 A resolution | Six-digit BCD /100000 represents at most 9.99999 A. Codec restriction, not a charger rating. | Vehicle config/codec |
| Charger default / max voltage | 4.02 / 4.15 V per populated cell | Lead setpoint/headroom policy; derived from topology, max 50 mV below OV. | Same |
| SAVE cadence / reserved flash | >=2000 ms; final 4 KiB reserved | Lead CRC/version storage, page/bank behavior requires physical power-loss test. | Settings/linker scripts |
| SoC | Supplied Battery Tracker Chebyshev model | Estimate only; valid no-load current required. No protection permission from SoC. | Sensor calibration, `bms_soc.c` |
| No-load current / energy gap | 5 A / 1000 ms | Lead estimator inputs; verified current/configuration needed. Wh deltas reject invalid/gap/restart data. | Sensor calibration |
| IMD clock | 128 MHz assumption | Lead TIM15 source, physical clock/capture routing must be verified. No division in capture ISR. | Vehicle config/CubeMX |

## Non-numeric permissions

All physical profiles default calibration/output/charger/inverter/balancing confirmations to zero. `BMS_ENABLE_BALANCING`, `BMS_ENABLE_CONTACTORS`, `BMS_ENABLE_CHARGING`, `BMS_ENABLE_INVERTER` also default zero. A confirmation without its explicit enable grants no energetic permission. A host-only DIGITAL profile supplies synthetic confirmed hardware for testing and is compile-rejected on ARM. Profile selection and physical confirmation are compile-time reviewable changes, not GUI settings.

## Documentation provenance

The source documents remain in the read-only lead `Supporting Docs/`: `adbms6830b.pdf`, `adbms6821-adbms6822.pdf`, `Datasheet_IVT-S.pdf`, `Datasheet_with_VTC6_rev_D2024-10-28.pdf`, `Can_Manual_V2.4.pdf`, `Battery Tracker.xlsx`, supplied competition rules, MB1367 schematic and the EVAL design support schematics/BOM. Current charger gateway documentation is referenced by the authoritative integration plan. See that report's source register for inspected revision/page evidence. Unresolved hardware facts are enumerated in [hardware-validation-pending.md](hardware-validation-pending.md).
