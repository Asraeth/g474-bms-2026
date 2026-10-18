The seven original P1 warning occurrences and the two separately identified NaN defects have been addressed. Only two production files changed in this task. The full ARM build, 37-group Level-2 suite, 20-scenario virtual bench and UBSan trap runs passed. No hardware was flashed.

The expanded host warning count is **196 → 189**: W026, W027, W030, W031, W035, W036 and W159 are eliminated; **all 33 P2 and 156 P3 diagnostics remain, and no new warnings were introduced**. This is a software verification result; physical timing and hardware qualification remain outstanding.

**Scope and complete diff**

This task started with the prior migrations already present as uncommitted work. The comparison baseline is the exact snapshot taken before this task's edits, rather than Git HEAD, which would include those earlier migrations.

| Changed file | Purpose |
|---|---|
| [Core/custom_lib/src/bms_libWrapper.c](C:/Users/SR/g474-bms-2026/Core/custom_lib/src/bms_libWrapper.c) | Explicit protection-measurement narrowing; dormant NaN lookup guard. |
| [Core/Src/main.c](C:/Users/SR/g474-bms-2026/Core/Src/main.c) | Deferred IMD arithmetic and coherent capture; fan NaN guard. |
| [tests/virtual_bms_bench.c](C:/Users/SR/g474-bms-2026/tests/virtual_bms_bench.c) | Simulated TIM3 registers and TIM15 captures; foreground IMD processing; reuse of fixture for numeric regressions. |
| [tests/run_virtual_bench.py](C:/Users/SR/g474-bms-2026/tests/run_virtual_bench.py) | Locate the existing intentional Error_Handler loop by function range after source lines move. No additional analyzer finding is allowed. |
| [tests/bms_p1_regression.c](C:/Users/SR/g474-bms-2026/tests/bms_p1_regression.c) | New exhaustive numeric, boundary, deferred-capture and NaN tests. |
| [tests/run_p1_regression.py](C:/Users/SR/g474-bms-2026/tests/run_p1_regression.py) | Compile the pre-edit wrapper as the golden reference; run current O1/O2/UBSan comparisons. |
| This report | Verification evidence and limits. |

[Complete task diff, including tests and this report](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/complete-task.diff). [Production-only diff](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/production.diff).

**Findings explained before the production edits**

| Original P1 IDs | Finding and reason for the selected change |
|---|---|
| W026/W027 | Cell conversion mixed a float NaN branch with double intermediates and implicitly narrowed the final result to float. The arithmetic feeds protection, so changing constants to single precision could alter threshold decisions. Retain the double expression, explicitly widen the signed raw code and explicitly narrow only the completed result. |
| W030/W031 | Die temperature had the same promotion/narrowing issue. In particular, raw 7150 previously produced exactly 70.0 °C; a naive single-precision rewrite can produce 70.0000305 °C and change the limit decision. Preserve the original expression and one final float conversion. |
| W035/W036 | VPV/board voltage used double intermediate arithmetic, internal ×25 scaling and implicit float publication. Preserve that exact scaling and evaluation, with explicit final narrowing. |
| W159 | TIM15 capture computed double multiplication/division and narrowed inside an ISR. Cortex-M4's single-precision FPU does not execute double operations directly. Capture only raw counts in the ISR; copy a coherent snapshot briefly under the existing interrupt-mask helpers, then perform the unchanged arithmetic in the foreground. |

No P1 rewrite changes a provisional threshold. Explicit casts document intentional precision boundaries; they do not eliminate the deliberate foreground software-double cost. The exhaustive comparisons below verify the resulting stored values rather than merely relying on a warning disappearing.

**Every production function changed**

| Function / current location | Change and behavior |
|---|---|
| [bms_parseVoltage](C:/Users/SR/g474-bms-2026/Core/custom_lib/src/bms_libWrapper.c:540) | Retains signed int16 decoding, INT16_MIN invalidation and `raw × 0.00015 + 1.5` in double. Publishes through an explicit float conversion. Indexing, payload lengths, topology and acquisition validity rules are unchanged. |
| [bms_parseAuxVoltage](C:/Users/SR/g474-bms-2026/Core/custom_lib/src/bms_libWrapper.c:566) | Die-temperature and VPV branches retain their original double calculations and reset-code rejection, then explicitly narrow. The raw external-temperature voltage calculation, TEMP mapping and Celsius validity are unchanged. |
| [convertCellTemp](C:/Users/SR/g474-bms-2026/Core/custom_lib/src/bms_libWrapper.c:821) | Returns NaN before the lookup when input is NaN. Previously NaN bypassed all ordered comparisons, left idx=32 and accessed index 33 of a 33-element table. Undefined reads could produce incorrect numeric/validity behavior if called. The function remains unused and unapproved; no sensor model, table, interpolation, finite endpoint behavior or 888 sentinel was changed. ±Infinity still follows the existing out-of-range behavior. |
| [main](C:/Users/SR/g474-bms-2026/Core/Src/main.c:116) | Calls the new capture consumer at the start of each foreground iteration, before the existing loop timing measurement. No work is added to a protection ISR. |
| [BMS_ProcessImdCapture](C:/Users/SR/g474-bms-2026/Core/Src/main.c:386) | Copies the pending flag, uint32 period and uint32 high time and clears pending in a short critical section; restores the previous interrupt mask before arithmetic. Retains the original 128000000/period frequency and double duty arithmetic with explicit final float conversion. Period zero still updates period while retaining previous duty/frequency. No SPI, waiting or floating-point arithmetic occurs with interrupts masked. |
| [HAL_TIM_IC_CaptureCallback](C:/Users/SR/g474-bms-2026/Core/Src/main.c:408) | Reads TIM15 period/high counts and publishes them with a pending flag; performs no division or float conversion. Other timers are ignored as before. The high register is still read only for nonzero period. |
| [BMS_WriteFanDuty](C:/Users/SR/g474-bms-2026/Core/Src/main.c:495) | Rejects nonfinite duty before the existing range check and float-to-uint16 conversion. NaN previously bypassed both ordered comparisons and reached an undefined integer conversion/register write. Rejected values leave the register unchanged, as existing out-of-range finite values did. Finite duty policy and encoding remain unchanged. The only current production caller still supplies finite 0.7. |

The additional `<math.h>` include supports the fan finite check. The new private volatile capture structure and private function declaration support the ISR/foreground handoff. `imdStatus` now has a single foreground writer. Repository search found no protection decision or other production reader of that diagnostic structure.

**Width, precision and timing review**

- Signed ADC inputs remain int16. Every input is exactly representable in double; all scaled/intermediate results are far inside float and double finite ranges. Existing reset-code rejection is preserved. No integer multiplication was substituted for voltage scaling.
- IMD period/high stay uint32 without signed subtraction, narrowing or intermediate integer multiplication. Any uint32 value converts exactly to double; `high × 100` is below 2^39 and is exact as an integer in double. Nonzero-period division produces a finite float even at uint32 extremes. Frequency uses the same uint32 integer division and zero guard.
- No acquisition/freshness timestamp arithmetic changed. Existing wraparound scenarios and exact freshness-deadline tests were rerun.
- IMD publication now occurs on the next foreground pass; intermediate captures coalesce to the most recent coherent pair. It is not an event queue. Long foreground/recovery work can delay that diagnostic update. No new IMD freshness or protection policy was added.
- The old 128 MHz IMD timer-clock assumption is retained, not newly validated. Physical timer clock/prescaler, waveform capture, interrupt latency and maximum foreground latency still require target measurements.
- The actual ARM object for the ISR calls only HAL_TIM_ReadCapturedValue (two call sites); it contains no divide/float-conversion instruction or `__aeabi_*` arithmetic helper. Double helpers remain in the new foreground consumer. [ISR disassembly](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/arm-HAL_TIM_IC_CaptureCallback.disassembly.txt), [foreground disassembly](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/arm-BMS_ProcessImdCapture.disassembly.txt).

**Build and test results**

| Verification | Result | Evidence / qualification |
|---|---|---|
| Full ARM build | PASS: 41 translation units, link success, 0 warnings, 0 errors | ARM GCC 14.3.1, existing GNU11/O0/-Wall/Cortex-M4 hard-float settings; text 80616, data 684, bss 33156 bytes. [Build log](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/firmware/build.log). |
| Complete Level-2 | PASS: 37/37 groups | Existing host runner and GCC 6.3.0. [Log](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/level2-regression/host-tests.log). |
| Complete Level-2 with UBSan | PASS: 37/37 groups | GCC 15.2.0; undefined/bounds/float-cast-overflow trap instrumentation. [Log](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/level2-ubsan/host-tests.log). |
| Complete virtual bench | PASS: 20/20 scenarios in each of normal O1, expanded-warning O2 and UBSan trap builds | 60 scenario executions; 56 checkpoints per build, 168 total. Detailed injected inputs, expected/actual state, validity, faults, charging, shutdown, balancing and telemetry: [full bench report](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/virtual-bench.md). |
| P1 numeric regressions | PASS at O1, O2 and UBSan | All 65536 signed raw codes × cell/die/VPV = 196608 bitwise comparisons per build against the compiled pre-edit wrapper; another 65536 finite legacy conversions per build. No stored result changed. [Results and golden hashes](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/numeric-regression/results.json). |
| Exact threshold regressions | PASS at O1, O2 and UBSan | Raw-code cell UV/OV, board UV/OV, die 0/70 °C, INT16_MIN invalidation and die INT16_MAX invalidation. |
| IMD regression group | PASS at O1, O2 and UBSan | ISR leaves published status untouched; foreground exact result; uint32 extremes, zero period, coalescing, unrelated timer, no pending work, interrupt-mask restoration. |
| NaN regression group | PASS at O1, O2 and UBSan | Converter NaN stays NaN without table overrun; finite legacy behavior unchanged; fan NaN/±Infinity/out-of-range rejected without writes, accepted finite encoding preserved; Celsius validity remains false. |
| Static analyzer | PASS with one existing intentional finding | GCC -fanalyzer completed. Only analyzer-specific finding is the deliberate fatal Error_Handler infinite loop at main.c:517; all other warnings retained as audit evidence. |
| Maximum-warning comparison | PASS: 189 retained, 7 eliminated, 0 introduced | GCC 15.2 O2 with the same expanded warning flags. No diagnostics originate in the new test code. [Full 196-row reconciliation](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/warning-reconciliation.md), [CSV](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/warning-reconciliation.csv). |

UBSan uses `-fsanitize=undefined,bounds,float-cast-overflow -fsanitize-undefined-trap-on-error -fno-sanitize-recover=all`. Separate negative controls for signed overflow, bounds and float-to-integer overflow all trapped with Windows status 0xC000001D. Normal runtime UBSan and ASan cannot link in the installed MinGW runtime (missing sanitizer libraries); no ASan success is claimed. Instrumented trap-mode tests did run successfully.

The additional 64-bit UBSan Level-2 build initially stopped on unused CMSIS 32-bit vector-helper pointer casts under -Werror. The final host build treats vendor headers as system headers, consistently with the existing virtual bench; project warnings remain errors. No vendor or production code was modified. This host setup adjustment is separate from the unchanged 196-warning comparison.

| Level-2 group family | Groups | Normal | UBSan |
|---|---:|---|---|
| IVT protocol | 6 | PASS | PASS |
| ADBMS counter sequencing | 5 | PASS | PASS |
| Pack integration, production configuration | 4 | PASS | PASS |
| Safety/ADBMS application, production configuration | 7 | PASS | PASS |
| Pack integration, existing synthetic balancing fixture | 4 | PASS | PASS |
| Safety/ADBMS application, existing synthetic balancing fixture | 7 | PASS | PASS |
| Real transport/PEC/INC hooks | 4 | PASS | PASS |
| Total | 37 | PASS | PASS |

The existing synthetic balancing fixture uses test-only confirmed/cap overrides. Production balancing remains unconfirmed with cap zero; those test settings are not an engineering approval or a production policy change.

| Virtual scenario | Normal / O2 / UBSan |
|---|---|
| 1. Normal startup, all 14 boards | PASS / PASS / PASS |
| 2. 112 nominal cells, healthy IVT | PASS / PASS / PASS |
| 3. One cell undervoltage | PASS / PASS / PASS |
| 4. One cell overvoltage | PASS / PASS / PASS |
| 5. Missing complete board | PASS / PASS / PASS |
| 6. Bad PEC acquisition | PASS / PASS / PASS |
| 7. Duplicate command counter | PASS / PASS / PASS |
| 8. Skipped command counter | PASS / PASS / PASS |
| 9. Counter rollover | PASS / PASS / PASS |
| 10. ADBMS stopped/stale/timeouts | PASS / PASS / PASS |
| 11. IVT current stale, voltage fresh | PASS / PASS / PASS |
| 12. IVT voltage stale, current fresh | PASS / PASS / PASS |
| 13. IVT system error | PASS / PASS / PASS |
| 14. IVT Alive/restart | PASS / PASS / PASS |
| 15. Pack-voltage disagreement | PASS / PASS / PASS |
| 16. Raw temperatures valid, Celsius unavailable | PASS / PASS / PASS |
| 17. Balancing request on channels 1–8 | PASS / PASS / PASS |
| 18. Balancing request on channels 9–16 | PASS / PASS / PASS |
| 19. Fault recovery/reset | PASS / PASS / PASS |
| 20. Timer tick rollover | PASS / PASS / PASS |

Healthy acquisition scenarios intentionally retain the temperature fault, inhibit charging and assert shutdown because external Celsius is still unavailable. The bench did not bypass that behavior to produce a passing result. Balancing remains disabled under production configuration.

**Repeated warning audit**

| Class | Before | After |
|---|---:|---:|
| -Wold-style-declaration | 5 | 5 |
| -Wsign-compare | 35 | 35 |
| -Wsign-conversion | 8 | 8 |
| -Wmissing-prototypes | 11 | 11 |
| -Wpedantic | 46 | 46 |
| -Wdouble-promotion | 18 | 15 |
| -Wfloat-conversion | 39 | 35 |
| -Wconversion | 33 | 33 |
| -Wstrict-prototypes | 1 | 1 |
| Total | 196 | 189 |

Source-line moves were reconciled against the original warning IDs, columns, classes and messages, retaining multiplicity for repeated macro diagnostics. Every remaining original occurrence matches; no warning flags or pragmas were changed to suppress P1 findings. All P3 source expressions and unrelated P2 expressions remain untouched, including raw-temperature arithmetic, thermistor constants/endpoints, balancing bitfields and IVT float representation. The fan P2 warning remains because its finite arithmetic was deliberately preserved; only its NaN input contract changed under the separate authorization.

**Unchanged parameters and remaining qualification**

Hash comparison confirms no changes to safety/topology configuration, IVT driver, CAN encoding/IDs, FDCAN configuration or other production files besides the two listed. The thermistor table and finite converter body, provisional limits, balancing control/policy/PWM cap, IVT sign convention and wiring assumptions remain unchanged. The legacy Celsius routine remains disabled; external Celsius remains NaN/invalid.

The seven warning occurrences are closed by explicit arithmetic contracts and ISR deferral, but target execution-time approval is not supplied by host tests. Foreground software-double cost, interrupt latency, real concurrent capture timing, actual clock configuration, physical CAN/isoSPI behavior, shutdown electrical behavior and sensor calibration still require physical validation. No target startup/clock initialization or physical register behavior was executed in the host harness. Those limitations are retained rather than inferred away from passing synthetic scenarios.

[Machine-readable verification summary](C:/Users/SR/.codex/visualizations/2026/10/17/01a14967-96bd-7591-ad1a-d4eed0ae3432/p1-fixes/verification-summary.json).
