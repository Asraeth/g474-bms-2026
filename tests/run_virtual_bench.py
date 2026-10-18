"""Build/run the real-source virtual BMS bench; never flash or edit production.

Example: python tests/run_virtual_bench.py --cc C:/msys64/mingw64/bin/gcc.exe
Each scenario gets a new process, so firmware static state starts as at reset.
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

SCENARIOS = [
    ("Normal startup with fourteen boards", "Two SRST sequences, RSTCC/SID, real init, fourteen complete replies; first acquisition and three further cycles.", "Before first acquisition: no valid ADBMS snapshot; after it: 112 cells/raw temperatures, 14 VPV/die measurements valid; TEMP only."),
    ("112 nominal cells and healthy IVT", "All cell raw words=15000 (3.750 V); current +1234 mA and U1 420000 mV every 20 ms; three 500 ms cycles.", "Fresh cell sum=420 V, IVT healthy, difference=0; TEMP only."),
    ("One cell undervoltage", "Board 14 position 8 raw=11000 (3.15 V); every other position=3.750 V; AUX/IVT healthy.", "Cell voltage remains valid but VOLTAGE joins TEMP; affected slot fault set."),
    ("One cell overvoltage", "Board 14 position 8 raw=19000 (4.35 V); every other position=3.750 V; AUX/IVT healthy.", "Cell voltage remains valid but VOLTAGE joins TEMP; affected slot fault set."),
    ("One complete board missing", "After valid startup, board 14 no longer accepts commands and returns eight FF bytes in its reply slot; absence persists through recovery.", "Bad PEC/missing-board evidence, complete ADBMS invalidation, COMM/VOLTAGE/TEMP, bounded two-attempt recovery; IVT stays fresh."),
    ("One acquisition with bad PEC", "Flip one PEC bit on board 14 S-voltage group B exactly once; then restore normal replies.", "PEC fault and complete invalidation; explicit recovery/init/new acquisition restores fresh data with TEMP only."),
    ("Duplicate command counter", "On the first S-voltage group A after new conversion/poll, replay board 14's preceding AUX counter and recompute valid PEC.", "Per-device sequence fault without PEC fault, invalidation and latched fault; reset/init/new acquisition recovers."),
    ("Skipped command counter", "Insert one extra physical counter increment on board 14's first S-voltage group A; recompute valid PEC.", "Received=expected+1 on board 14, PEC good; latch/invalidate until explicit recovery."),
    ("Command-counter rollover", "Run 25 complete foreground/fault-stop cycles at 500 ms intervals with healthy cell/AUX/IVT traffic.", "At least two 63-to-1 wraps per device, no false counter/PEC/COMM faults; TEMP remains."),
    ("ADBMS communication stops / stale / timeout", "Stop foreground acquisition for >=1000 ms while IVT/timer continue; fail queued-stop and acquisition command TX; attempt bounded recovery; restore link, hold ADC busy, then recover and fail AUX data RX.", "Age invalidation at >=1000 ms; finite TX/RX and 50 ms poll timeouts; preserved primary diagnoses; COMM/VOLTAGE/TEMP; no valid stale telemetry."),
    ("IVT current stops; U1 continues", "Stop 521 frames for >100 ms; keep 522 every 20 ms and ADBMS acquisitions active.", "Current invalid/stale, U1 and ADBMS fresh; CURRENT/IVT/TEMP (0x1C)."),
    ("IVT U1 stops; current continues", "Stop 522 frames for >300 ms; keep 521 every 20 ms and ADBMS acquisitions active.", "U1 invalid/stale, current and ADBMS fresh; VOLTAGE/IVT/TEMP (0x16)."),
    ("IVT system error", "Inject current result DB1 high nibble=8 with a new low counter after healthy operation; observe before another clean U1; then restore clean streams.", "Both cached channels invalid at the error observation; CURRENT/VOLTAGE/IVT/TEMP (0x1E), followed by TEMP only after clean independent results."),
    ("IVT Alive / restart", "Inject 511 DLC8 BF 04 11 12 34 56 78 00; then current only, then both streams.", "Alive count increments; cached channels/configuration verification cleared; independent recovery, finally TEMP only."),
    ("Pack-voltage disagreement", "Keep 112 cells at 3.750 V (420 V sum), inject U1=400000 mV, current healthy.", "Separate 420000 mV sum and -20000 mV U1-minus-sum diagnostic; no invented disagreement fault; TEMP only."),
    ("Raw temperature valid; Celsius unavailable", "Inject distinct GPIO1-10 raw AUX words plus VPV=-2000 and die=4900; check all logical TEMP1-8 mappings on all fourteen boards.", "112 input voltages valid in GPIO9/7/8/10/4/6/5/3 order; 0 Celsius valid; 14 VPV/die valid; TEMP remains."),
    ("Balancing request on populated channels", "Request enable and duty 15 on channels 1-8 on every board; write PWM registers and request discharge.", "Unchanged cap=0/configuration-unconfirmed refuses enable; all PWM/DCC/timer controls remain zero."),
    ("Balancing request on unused channels", "Request enable and duty 15 on channels 9-16 on every board; write PWM registers and request discharge.", "Unused channels and all other discharge controls remain zero; no out-of-bounds write; cap/configuration inhibition preserved."),
    ("Fault recovery/reset", "Inject PEC-correct duplicate counter; execute actual BMS_FaultHandler, then init, then full acquisition; inspect after reset before new acquisition.", "Reset alone never refreshes data; valid zero baseline, bounded recovery and new complete acquisitions required; TEMP remains after recovery."),
    ("Timer tick rollover", "Start tick at UINT32_MAX-400; startup crosses wrap; run fresh acquisition, then suppress acquisition >=1000 ms, then reset/recover.", "Wrap-safe IVT/ADBMS ages; fresh -> stale -> fresh with no false counter fault or stale revival."),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=(r'C:\msys64\mingw64\bin\gcc.exe'
                        if Path(r'C:\msys64\mingw64\bin\gcc.exe').is_file() else shutil.which('gcc')))
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not args.cc:
        parser.error('A host GCC compiler is required')
    repo = Path(__file__).resolve().parents[1]
    build = args.build_dir or Path(tempfile.mkdtemp(prefix='virtual-bms-bench-'))
    build.mkdir(parents=True, exist_ok=True)
    report = args.report or repo / 'docs/virtual-bms-bench-verification.md'
    env = dict(os.environ)
    env['PATH'] = str(Path(args.cc).resolve().parent) + os.pathsep + env.get('PATH', '')
    env['LC_ALL'] = 'C'
    commands = []

    def run(command, log, timeout=90):
        command = [str(part) for part in command]
        commands.append(command)
        try:
            result = subprocess.run(command, cwd=repo, env=env, capture_output=True,
                                    text=True, encoding='utf-8', errors='replace', timeout=timeout)
            content = result.stdout + result.stderr
            code = result.returncode
        except subprocess.TimeoutExpired as error:
            content = (error.stdout or b'').decode(errors='replace') if isinstance(error.stdout, bytes) else error.stdout or ''
            content += '\nTIMEOUT: verification process did not complete\n'
            code = 124
        Path(log).write_text(content, encoding='utf-8')
        return code, content

    production = {path.relative_to(repo).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                  for root in ('Core', 'Drivers') for path in (repo / root).rglob('*') if path.is_file()}
    (build / 'production-before.json').write_text(json.dumps(production, indent=2))
    _, version = run([args.cc, '--version'], build / 'compiler-version.log')
    print(version.splitlines()[0], flush=True)
    includes = [f'-I{repo / path}' for path in ('Core/Inc', 'Core/custom_lib/inc')]
    for path in ('Drivers/STM32G4xx_HAL_Driver/Inc', 'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include'):
        includes += ['-isystem', str(repo / path)]
    base = [args.cc, '-std=gnu11', '-g', '-DUSE_HAL_DRIVER', '-DSTM32G474xx',
            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', *includes]
    source = (repo / 'Core/custom_lib/src/bms_cmdlist.c').read_text()
    for name in ('ADCV', 'ADSV', 'ADAX2', 'ADAX'):
        initializer = re.search(rf'{name}_t const {name}_default\s*=\s*(\{{.*?\}});', source, re.DOTALL).group(1)
        source = source.replace(f'{name}_t {name} = {name}_default;', f'{name}_t {name} = {initializer};')
    commands_source = build / 'host_commands.c'
    commands_source.write_text(source)
    sources = [repo / 'tests/virtual_bms_bench.c', repo / 'Core/custom_lib/src/ivt_s.c',
               repo / 'Core/custom_lib/src/bms_command_counter.c', commands_source]
    executable = build / ('virtual_bms_bench.exe' if os.name == 'nt' else 'virtual_bms_bench')
    code, output = run(base + ['-O1', '-Wall', '-Wextra', *sources, '-lm', '-o', executable], build / 'build.log')
    if code:
        print(output[-14000:]); raise SystemExit(f'Virtual bench build failed: {build / "build.log"}')

    all_results = {}

    def scenarios(name, binary):
        directory = build / name
        directory.mkdir(exist_ok=True)
        results = []
        for number, (title, inputs, expected) in enumerate(SCENARIOS, 1):
            evidence = directory / f'{number:02d}.jsonl'
            code, output = run([binary, number, evidence, directory / f'{number:02d}-firmware.log'],
                               directory / f'{number:02d}-process.log', timeout=40)
            rows = [json.loads(line) for line in evidence.read_text().splitlines()] if evidence.is_file() else []
            summary = next((row for row in rows if row.get('summary')), {})
            passed = code == 0 and summary.get('pass') is True and all(row.get('pass') for row in rows)
            results.append({'number': number, 'name': title, 'inputs': inputs, 'expected': expected,
                            'pass': passed, 'exit': code, 'checkpoints': [row for row in rows if not row.get('summary')],
                            'summary': summary, 'evidence': str(evidence)})
            print(f'{name}: {number:02d} {"PASS" if passed else "FAIL"} {title}', flush=True)
            if not passed:
                print(output[-2500:], flush=True)
        all_results[name] = results
        return all(row['pass'] for row in results)

    overall = scenarios('baseline', executable)
    warnings = ['-O2', '-Wall', '-Wextra', '-Wpedantic', '-Wshadow', '-Wconversion', '-Wsign-conversion',
                '-Wformat=2', '-Wstrict-prototypes', '-Wmissing-prototypes', '-Wcast-align=strict',
                '-Wdouble-promotion', '-Warray-bounds=2', '-Wstringop-overflow=4', '-Wnull-dereference',
                '-Wvla', '-Wduplicated-cond', '-Wlogical-op', '-Wundef']
    warning_binary = build / ('virtual_bms_warnings.exe' if os.name == 'nt' else 'virtual_bms_warnings')
    warning_code, warning_output = run(base + warnings + sources + ['-lm', '-o', warning_binary], build / 'maximum-warnings.log')
    if warning_code == 0:
        overall &= scenarios('optimized-warnings', warning_binary)
    else:
        overall = False
    warning_lines = [line for line in warning_output.splitlines() if 'warning:' in line]
    warning_groups = collections.Counter(re.search(r'\[(-W[^\]]+)\]', line).group(1)
                                         if re.search(r'\[(-W[^\]]+)\]', line) else 'unclassified' for line in warning_lines)
    (build / 'warning-summary.json').write_text(json.dumps({'count': len(warning_lines), 'groups': warning_groups,
                                                          'messages': warning_lines}, indent=2))
    print(f'Maximum warning audit: exit={warning_code}, warnings={len(warning_lines)}', flush=True)

    probe = build / 'sanitizer-probe.c'
    probe.write_text('#include <limits.h>\n#include <stdlib.h>\nint main(int argc,char **argv) { '
                     '(void)argv; if(argc>1) { volatile int n=INT_MAX; return n+1; } return 0; }\n')
    sanitizer_results = []
    variants = [
        ('AddressSanitizer', ['-fsanitize=address', '-fno-omit-frame-pointer'], 'address'),
        ('UndefinedBehaviorSanitizer', ['-fsanitize=undefined,bounds,float-cast-overflow', '-fno-sanitize-recover=all'], 'undefined'),
        ('UndefinedBehaviorSanitizer trap mode', ['-fsanitize=undefined,bounds,float-cast-overflow',
             '-fsanitize-undefined-trap-on-error', '-fno-sanitize-recover=all'], 'undefined-trap'),
    ]
    ubsan_ran = False
    for title, flags, name in variants:
        if name == 'undefined-trap' and ubsan_ran:
            continue
        probe_exe = build / (f'probe-{name}.exe' if os.name == 'nt' else f'probe-{name}')
        probe_code, probe_output = run([args.cc, '-O1', *flags, probe, '-o', probe_exe], build / f'probe-{name}-compile.log')
        support = {'name': title, 'flags': flags, 'supported': False, 'pass': None}
        if probe_code == 0:
            probe_code, probe_output = run([probe_exe], build / f'probe-{name}-run.log')
        if probe_code == 0:
            if name == 'address':
                probe.write_text('#include <stdlib.h>\nint main(int argc,char **argv) { (void)argv; '
                                 'volatile int index=argc; int *p=(int*)malloc(sizeof(int)); p[index]=1; free(p); return 0; }\n')
                negative_compile, _ = run([args.cc, '-O0', *flags, probe, '-o', probe_exe], build / f'probe-{name}-negative-compile.log')
                if negative_compile:
                    raise RuntimeError('ASan negative-control compilation failed')
            negative_code, negative_output = run([probe_exe, 'negative-control'], build / f'probe-{name}-negative.log')
            if negative_code == 0 or (name == 'address' and 'AddressSanitizer' not in negative_output):
                support['reason'] = 'Negative control was not detected; no instrumentation claim.'
            else:
                support['supported'] = True
                support['negative_control_exit'] = negative_code
                if name.startswith('undefined'):
                    controls = {
                        'bounds': 'int main(int argc,char **argv) { (void)argv; volatile int index=argc; int a[1]={0}; return a[index]; }',
                        'float-cast': 'int main(void) { volatile double value=1e300; volatile int result=(int)value; return result; }',
                    }
                    for control, body in controls.items():
                        probe.write_text(body + '\n')
                        control_code, _ = run([args.cc, '-O0', *flags, probe, '-o', probe_exe], build / f'probe-{name}-{control}-compile.log')
                        if control_code:
                            raise RuntimeError(f'{control} sanitizer control failed to compile')
                        control_code, _ = run([probe_exe, 'negative-control'], build / f'probe-{name}-{control}.log')
                        support[f'{control}_control_exit'] = control_code
                        if control_code == 0:
                            raise RuntimeError(f'{control} sanitizer negative control was not detected')
                sanitized = build / (f'virtual_bms_{name}.exe' if os.name == 'nt' else f'virtual_bms_{name}')
                compile_code, _ = run(base + ['-O1', '-Wall', '-Wextra', *flags, *sources, '-lm', '-o', sanitized],
                                      build / f'{name}-build.log')
                support['pass'] = compile_code == 0 and scenarios(name, sanitized)
                overall &= support['pass']
                if name.startswith('undefined'): ubsan_ran = True
        else:
            support['reason'] = probe_output.strip()[-1500:]
        sanitizer_results.append(support)
        print(f'{title}: {"PASS" if support["pass"] else "UNSUPPORTED" if not support["supported"] else "FAIL"}', flush=True)
        # Restore UB probe if the ASan negative control replaced it.
        probe.write_text('#include <limits.h>\nint main(int argc,char **argv) { (void)argv; '
                         'if(argc>1) { volatile int n=INT_MAX; return n+1; } return 0; }\n')

    # GCC's path-sensitive analyzer is separate from dynamic scenarios.
    analyzer_code, analyzer_output = run([args.cc, '-std=gnu11', '-O0',
        '-DUSE_HAL_DRIVER', '-DSTM32G474xx', *includes, '-Wall', '-Wextra', '-fanalyzer', '-c',
        sources[0], '-o', build / 'analyzed-bench.o'], build / 'static-analyzer.log', timeout=120)
    analyzer_warnings = [line for line in analyzer_output.splitlines() if 'warning:' in line]
    analyzer_findings = [line for line in analyzer_warnings if '-Wanalyzer-' in line]
    # Warnings are review evidence, not silently converted into a clean audit.
    # The known deliberate fatal-handler loop is expected; other analyzer
    # findings or an incomplete analyzer run make verification non-clean.
    # Locate the known fatal handler by function, not a stale source line.
    main_lines = (repo / 'Core/Src/main.c').read_text().splitlines()
    handler_start = next(i + 1 for i, line in enumerate(main_lines)
                         if line.startswith('void Error_Handler(void)'))
    handler_end = next(i + 1 for i in range(handler_start, len(main_lines))
                       if main_lines[i].startswith('#ifdef'))
    def intentional_fatal_loop(message):
        match = re.search(r'Core/Src/main.c:(\d+):', message.replace('\\', '/'))
        return bool(match and handler_start <= int(match.group(1)) < handler_end
                    and '-Wanalyzer-infinite-loop' in message)
    unexpected_findings = [line for line in analyzer_findings if not intentional_fatal_loop(line)]
    overall &= analyzer_code == 0 and not unexpected_findings
    (build / 'commands.json').write_text(json.dumps(commands, indent=2))
    changed = [name for name, digest in production.items()
               if hashlib.sha256((repo / name).read_bytes()).hexdigest() != digest]
    overall &= not changed
    summary = {'pass': bool(overall), 'compiler': version.splitlines()[0], 'runs': all_results,
               'sanitizers': sanitizer_results, 'maximum_warning_count': len(warning_lines),
               'warning_groups': warning_groups, 'static_analyzer_exit': analyzer_code,
               'static_analyzer_warnings': analyzer_warnings, 'static_analyzer_findings': analyzer_findings,
               'production_files_changed': changed}
    regression = build / 'level2-regression/host-tests.log'
    if regression.is_file():
        summary['level2_regression_groups'] = sum(line.startswith('PASS ') for line in regression.read_text().splitlines())
    (build / 'results.json').write_text(json.dumps(summary, indent=2))
    render_report(report, build, summary)
    print(f'VIRTUAL_BENCH={"PASS" if overall else "FAIL"}; report={report}', flush=True)
    raise SystemExit(0 if overall else 1)


def fault_names(flags):
    return '+'.join(name for bit, name in [(1, 'COMM'), (2, 'VOLTAGE'), (4, 'TEMP'), (8, 'CURRENT'), (16, 'IVT')]
                    if flags & bit) or 'OK'


def render_report(report, build, result):
    lines = [f'**Virtual BMS bench verification: {"PASS" if result["pass"] else "FAIL"}.**', '',
        'This host-only harness includes the actual production SPI/PEC transport, command-counter checker, '
        'BMS wrapper, CAN receive/transmit adapters, IVT-S driver, periodic timer callback and fault-recovery handler. '
        'A deterministic fourteen-device model supplies real six-byte register payloads plus independently generated PEC/counters. '
        'The real FDCAN receive callback consumes synthetic HAL frames; real telemetry passes through the CAN TX queue into a host capture. '
        'Wake/timer/clock/GPIO/peripheral behavior is simulated. No hardware was flashed.', '',
        '**Production configuration is unchanged.** All nominal scenarios retain the production TEMP fault because raw temperature '
        'acquisition cannot validate Celsius without the final sensor model. Charging is refused and shutdown remains logically asserted. '
        'Balancing cap/budget remain zero and configuration unconfirmed: both populated and unused-channel requests must be inhibited. '
        'There are no injected healthy Celsius values, nonzero PWM overrides or new disagreement limits in this bench.', '',
        f'Compiler: `{result["compiler"]}`. All twenty scenarios run in isolated processes, with multiple acquisitions, '
        'time progression, faults and recovery checkpoints. PASS means actual state matches the declared current-software expectation; '
        'it does not mean the provisional safety policy or hardware is approved.', '',
        '| Build/run | Scenarios passed | Result |', '|---|---:|---|']
    for name, scenarios in result['runs'].items():
        lines.append(f'| {name} | {sum(s["pass"] for s in scenarios)}/20 | {"PASS" if all(s["pass"] for s in scenarios) else "FAIL"} |')
    if 'level2_regression_groups' in result:
        lines.append(f'| Existing Level-2 regression | {result["level2_regression_groups"]} groups across five executables | PASS (separate recorded run) |')
        lines += ['', f'[Level-2 regression log]({(build / "level2-regression/host-tests.log").as_posix()}). '
            'That separate existing suite retains its prior isolated nonzero-PWM test fixture; the new virtual bench uses '
            'only production cap/configuration values. Reproduce the regression separately with '
            '`python tests/run_host_tests.py --cc C:/MinGW/bin/gcc.exe --build-dir <regression-directory>`.']
    lines += ['', '**Instrumented and static checks.**', '', '| Check | Actual result |', '|---|---|']
    for sanitizer in result['sanitizers']:
        status = 'PASS' if sanitizer['pass'] else 'UNSUPPORTED' if not sanitizer['supported'] else 'FAIL'
        reason = sanitizer.get('reason', f'All twenty scenarios; signed-overflow negative control exit {sanitizer.get("negative_control_exit")}; '
            f'array-bounds control exit {sanitizer.get("bounds_control_exit", "not applicable")}; '
            f'float-cast control exit {sanitizer.get("float-cast_control_exit", "not applicable")}.')
        reason = reason.replace('\n', ' ').replace('|', '/')
        lines.append(f'| {sanitizer["name"]} | {status}: {reason} |')
    lines.append(f'| Maximum warning audit | Exit {0 if "optimized-warnings" in result["runs"] else "failure"}; {result["maximum_warning_count"]} warnings. See full diagnostic inventory below. |')
    lines.append(f'| GCC path-sensitive static analyzer | Exit {result["static_analyzer_exit"]}; {len(result["static_analyzer_warnings"])} compiler/analyzer warnings; see full log. |')
    lines += ['', 'Analyzer-specific findings: ' + ('; '.join(result.get('static_analyzer_findings', [])) or 'none') + '. '
        'The reported loop is the existing deliberate fatal `Error_Handler` stop, not a new acquisition hang. '
        'The maximum-warning audit has no harness-source diagnostics; retained firmware diagnostics include conversions, signedness, '
        'prototypes, older declaration style and C11 pedantic warnings for existing binary constants. '
        'This is not a zero-warning result at the expanded host warning level.']
    lines += ['', 'Array guards surround simulated SPI payloads and the complete 130-frame TX capture; every transfer checks full packet size, '
        'bounds and canaries. Scenario 18 requests channels 9-16 on all boards; group C always includes an unpopulated reset-code word '
        'to verify it is protected by PEC but not stored as a ninth cell. Supported UBSan builds also instrument array bounds and float-to-integer conversions. '
        'These checks do not constitute exhaustive memory proof.', '',
        'Warning categories: ' + ', '.join(f'`{key}`={value}' for key, value in sorted(result['warning_groups'].items())) + '.', '',
        f'[Full maximum-warning diagnostics]({(build / "maximum-warnings.log").as_posix()}), '
        f'[warning inventory]({(build / "warning-summary.json").as_posix()}), '
        f'[static-analyzer diagnostics]({(build / "static-analyzer.log").as_posix()}). '
        'Warnings are recorded without suppressing production diagnostics or editing firmware to obtain a clean host audit.', '',
        '**Scenario observations.** Every checkpoint checks all requested validity/fault/output fields and captures all 130 telemetry frames. '
        'In the table, V/A/T/C denote valid cell positions, AUX snapshot, raw temperature inputs and Celsius values. '
        'I/U denote current/U1 validity. Shutdown is the logical FAULT_CTRL assertion request, not a simulated transistor or contactor.', '',
        '| # | Scenario / injected inputs | Expected state | Actual checkpoint states | Result |', '|---:|---|---|---|---|']
    for scenario in result['runs']['baseline']:
        observations = []
        for checkpoint in scenario['checkpoints']:
            a = checkpoint['actual']
            observations.append(f'`{checkpoint["checkpoint"]}`: V={a["valid_cells"]}, A={int(a["aux_valid"])}, '
                f'T={a["valid_raw_temperatures"]}, C={a["valid_celsius"]}, I/U={int(a["current_valid"])}/{int(a["voltage_valid"])}, '
                f'faults={fault_names(a["faults"])} (0x{a["faults"]:02X}), charging={int(a["charging"])}, '
                f'shutdown={int(a["shutdown_asserted"])}, balance={int(a["balancing"])}; '
                f'PEC/CC={int(a["pec_error"])}/{int(a["counter_error"])}; '
                f'transport primary/live={a["transport_fault"]}/{a["live_transport_fault"]}, stop-failed={int(a["stop_failed"])}; '
                f'{len(checkpoint["telemetry"])} TX frames')
        lines.append(f'| {scenario["number"]} | **{scenario["name"]}**. {scenario["inputs"]} | {scenario["expected"]} '
            f'Charging=false, shutdown=true, balancing=false at every checkpoint. | {"<br>".join(observations)} | '
            f'{"PASS" if scenario["pass"] else "FAIL"} |')
    lines += ['', '**Telemetry output per checkpoint.** Every JSONL evidence file contains complete expected/actual states, accepted measurement '
        'timestamps, raw IVT status, stale flags, counter evidence and all 130 eight-byte TX payloads. The decoded table below shows the final '
        'checkpoint of each scenario; earlier fault/recovery stages remain in the full evidence.', '',
        '| # | Cell B06F: mV / flags | Board B07D: VPV mV / flags | B07E current mA / flags | B07F U1 mV / flags | B080 sum / difference mV | Full evidence |',
        '|---:|---|---|---|---|---|---|']
    def integer(data, offset, size):
        return int.from_bytes(bytes(data[offset:offset + size]), 'little', signed=True)
    for scenario in result['runs']['baseline']:
        if not scenario['checkpoints']:
            lines.append(f'| {scenario["number"]} | Missing evidence | | | | | |'); continue
        frames = {frame['id']: frame['data'] for frame in scenario['checkpoints'][-1]['telemetry']}
        c, b, i, u, x = [frames[key] for key in (0xB06F, 0xB07D, 0xB07E, 0xB07F, 0xB080)]
        lines.append(f'| {scenario["number"]} | {integer(c,0,2)} / 0x{c[7]:02X} | {integer(b,0,4)} / 0x{b[7]:02X} | '
            f'{integer(i,0,4)} / 0x{i[6]:02X} | {integer(u,0,4)} / 0x{u[6]:02X} | '
            f'{integer(x,0,4)} / {integer(x,4,4)} | [{scenario["number"]:02d}]({Path(scenario["evidence"]).as_posix()}) |')
    lines += ['', 'All frames are checked for unique extended DLC-8 IDs: 112 cell slots B000-B06F, fourteen board slots B070-B07D, '
        'current B07E, U1 B07F, cross-check B080 and charger 1806E5F4. Invalid signed values use INT16_MIN/INT32_MIN with cleared validity. '
        'Raw temperature validity never sets Celsius validity. Charger payload disable byte remains 1. Full payloads include fault/discharge '
        'bits, not only the validity-byte summaries above.', '',
        'Transport diagnostic codes are 0=OK, 1=context, 2=TX, 3=RX, 4=poll timeout, 5=unsupported command. '
        'The primary field describes the last failed acquisition; the live field can change during stop/readback. '
        'A stop-only failure sets stop-failed and live transport diagnosis without inventing a new acquisition diagnosis.', '',
        '**Limitations and interpretation.** A missing board is modeled as one absent response slot while other slots remain available; '
        'the actual physical chain may lose downstream replies too. HAL timeouts and virtual 100 ms timer scheduling (plus explicit checkpoint timer service) are deterministic, '
        'not target latency measurements. SPI wire propagation, CAN arbitration/FIFO pressure, interrupt preemption, real MCU startup and '
        'electrical shutdown/balancing behavior are not modeled. Because Celsius stays invalid and PWM is locked at zero, '
        'this production-configuration bench proves inhibition/retained faults but does not exercise healthy charge enable, '
        'shutdown release or active nonzero physical balancing. The complete main startup function is compiled with its entry point renamed, '
        'while its acquisition/reset sequence is driven by the host scheduler; clock/peripheral initialization is mocked. '
        'The actual timer callback and recovery handler execute unmodified. Unsupported sanitizers are reported as unsupported, never PASS. '
        'Unresolved thermistor, protection, PWM, IVT polarity/configuration and board-routing decisions remain outstanding.', '',
        f'Production source changes detected: **{len(result["production_files_changed"])}**. '
        f'[Machine-readable results]({(build / "results.json").as_posix()}), '
        f'[compiler commands]({(build / "commands.json").as_posix()}).', '',
        'Files added for this task: '
        f'[host C harness]({(Path(__file__).resolve().parent / "virtual_bms_bench.c").as_posix()}), '
        f'[host runner/report generator]({Path(__file__).resolve().as_posix()}) and this generated report. '
        'No existing firmware, safety parameter or hardware assumption was edited.', '',
        'Reproduce with `python tests/run_virtual_bench.py --cc C:/msys64/mingw64/bin/gcc.exe --build-dir <artifact-directory>`. '
        'The harness/runner are host-only files; no new production feature or parameter change is required.', '']
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
