"""Verify P1 fixes against an exact pre-edit wrapper snapshot; never flash.

The snapshot is an explicit input, not a regenerated approximation. Golden
results cover all signed 16-bit ADC inputs and finite legacy conversions.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--baseline-dir', required=True, type=Path)
    parser.add_argument('--build-dir', required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    build = args.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    original = args.baseline_dir.resolve() / 'Core/custom_lib/src/bms_libWrapper.c'
    if not original.is_file():
        parser.error('The pre-edit bms_libWrapper.c snapshot is required')
    env = dict(os.environ)
    env['PATH'] = str(Path(args.cc).resolve().parent) + os.pathsep + env.get('PATH', '')
    env['LC_ALL'] = 'C'
    commands, results = [], []

    def run(command, name):
        command = [str(part) for part in command]
        commands.append(command)
        process = subprocess.run(command, cwd=repo, env=env, capture_output=True,
                                 text=True, encoding='utf-8', errors='replace', timeout=120)
        output = process.stdout + process.stderr
        (build / f'{name}.log').write_text(output, encoding='utf-8')
        results.append({'name': name, 'exit': process.returncode})
        if process.returncode:
            print(output[-12000:])
            raise RuntimeError(f'{name} failed: exit {process.returncode}')
        if not name.endswith('compile'):
            print(output, end='', flush=True)

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
    sources = [repo / 'tests/bms_p1_regression.c', repo / 'Core/custom_lib/src/ivt_s.c',
               repo / 'Core/custom_lib/src/bms_command_counter.c', commands_source]
    golden = build / 'pre-edit-golden.bin'
    suffix = '.exe' if os.name == 'nt' else ''
    original_exe = build / f'pre-edit{suffix}'
    # Quoting is a single compiler argument; no shell interpolation is used.
    original_define = f'-DBMS_BASELINE_WRAPPER_PATH="{original.as_posix()}"'
    run(base + ['-O1', original_define, *sources, '-lm', '-o', original_exe], 'pre-edit-compile')
    run([original_exe, '--write-baseline', golden], 'pre-edit-golden')
    variants = {
        'current-O1': ['-O1', '-Wall', '-Wextra'],
        'current-O2': ['-O2', '-Wall', '-Wextra', '-Wpedantic', '-Wshadow', '-Wconversion',
                       '-Wsign-conversion', '-Wformat=2', '-Wstrict-prototypes', '-Wmissing-prototypes',
                       '-Wcast-align=strict', '-Wdouble-promotion', '-Warray-bounds=2',
                       '-Wstringop-overflow=4', '-Wnull-dereference', '-Wvla',
                       '-Wduplicated-cond', '-Wlogical-op', '-Wundef'],
        'current-UBSan': ['-O1', '-Wall', '-Wextra', '-fsanitize=undefined,bounds,float-cast-overflow',
                          '-fsanitize-undefined-trap-on-error', '-fno-sanitize-recover=all'],
    }
    try:
        for name, flags in variants.items():
            executable = build / f'{name}{suffix}'
            run(base + [*flags, *sources, '-lm', '-o', executable], f'{name}-compile')
            run([executable, '--verify', golden], name)
    finally:
        (build / 'commands.json').write_text(json.dumps(commands, indent=2))
        (build / 'results.json').write_text(json.dumps({
            'baseline_wrapper': str(original),
            'baseline_sha256': hashlib.sha256(original.read_bytes()).hexdigest(),
            'golden_sha256': hashlib.sha256(golden.read_bytes()).hexdigest(),
            'results': results,
        }, indent=2))
    print('P1 numeric/ISR/NaN regressions: PASS (O1, O2, UBSan trap mode)')


if __name__ == '__main__':
    main()
