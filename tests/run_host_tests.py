"""Run IVT-S synthetic-frame and pack-integration checks without flashing.

Usage: python tests/run_host_tests.py [--cc /path/to/gcc] [--build-dir /path]
The protocol tests have no HAL dependency. The integration test uses transport
stubs and the actual BMS wrapper; firmware builds compile all real sources.
"""
import argparse
import os
import re
from pathlib import Path
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default=os.environ.get('CC', shutil.which('gcc')))
parser.add_argument('--build-dir', type=Path)
args = parser.parse_args()
if not args.cc:
    parser.error('Set CC or pass --cc with a host GCC compiler path')
repo = Path(__file__).resolve().parents[1]
build = args.build_dir or Path(tempfile.mkdtemp(prefix='bms-ivt-tests-'))
build.mkdir(parents=True, exist_ok=True)
includes = ['Core/Inc', 'Core/custom_lib/inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
            'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include']
base = [args.cc, '-std=gnu11', '-O0', '-Wall', '-Wextra', '-Werror',
        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections']
base += [f'-I{repo / inc}' for inc in includes]
logs = []

def run(command):
    result = subprocess.run([str(arg) for arg in command], cwd=repo, capture_output=True, text=True)
    logs.append(result.stdout + result.stderr)
    print(logs[-1], end='')
    if result.returncode:
        (build / 'host-tests.log').write_text('\n'.join(logs))
        raise SystemExit(result.returncode)

protocol = build / ('ivt_s_test.exe' if os.name == 'nt' else 'ivt_s_test')
run(base + [repo / 'tests/ivt_s_test.c', repo / 'Core/custom_lib/src/ivt_s.c', '-o', protocol])
run([protocol])
counter_source = repo / 'Core/custom_lib/src/bms_command_counter.c'
counter = build / ('adbms_counter_test.exe' if os.name == 'nt' else 'adbms_counter_test')
run(base + [repo / 'tests/adbms_counter_test.c', counter_source, '-o', counter])
run([counter])
integration = repo / 'tests/bms_pack_test.c'
if integration.is_file():
    # Older host GCC cannot compile the existing file-scope struct-copy
    # initializers in bms_cmdlist.c. Expand those four copies to their exact
    # initializer bodies, retaining real ADC opcodes. ARM builds use real source.
    command_source = (repo / 'Core/custom_lib/src/bms_cmdlist.c').read_text()
    commands = build / 'host_commands.c'
    for name in ('ADCV', 'ADSV', 'ADAX2', 'ADAX'):
        initializer = re.search(rf'{name}_t const {name}_default\s*=\s*(\{{.*?\}});',
                                command_source, re.DOTALL).group(1)
        command_source = command_source.replace(f'{name}_t {name} = {name}_default;',
                                                f'{name}_t {name} = {initializer};')
    commands.write_text(command_source)
    executable = build / ('bms_pack_test.exe' if os.name == 'nt' else 'bms_pack_test')
    # Existing wrapper diagnostics contain pre-existing signedness warnings
    # under -Wextra. Keep -Wall/-Werror and use the firmware's warning level.
    wrapper_flags = [flag for flag in base if flag != '-Wextra']
    run(wrapper_flags + ['-DUSE_HAL_DRIVER', '-DSTM32G474xx', integration,
        repo / 'Core/custom_lib/src/ivt_s.c', counter_source, commands, '-lm', '-o', executable])
    run([executable])
    simulated_balance = build / ('bms_balance_fixture.exe' if os.name == 'nt' else 'bms_balance_fixture')
    run(wrapper_flags + ['-DUSE_HAL_DRIVER', '-DSTM32G474xx',
        '-DBMS_BALANCING_CONFIG_CONFIRMED=1', '-DBMS_BALANCING_MAX_PWM=7', '-DBMS_BALANCING_PWM_BUDGET=56',
        integration, repo / 'Core/custom_lib/src/ivt_s.c', counter_source, commands, '-lm', '-o', simulated_balance])
    run([simulated_balance])
else:
    raise SystemExit(f'Missing integration test: {integration}')
transport = build / ('adbms_transport_test.exe' if os.name == 'nt' else 'adbms_transport_test')
run([flag for flag in base if flag != '-Wextra'] + ['-DUSE_HAL_DRIVER', '-DSTM32G474xx',
    repo / 'tests/adbms_transport_test.c', counter_source, '-o', transport])
run([transport])
(build / 'host-tests.log').write_text('\n'.join(logs))
print(f'HOST_TESTS=PASS; log={build / "host-tests.log"}')
