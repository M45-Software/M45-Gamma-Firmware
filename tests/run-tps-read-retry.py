#!/usr/bin/env python3
"""Run the firmware's TPS failure classification and safety checks with stubbed I/O."""
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r'^(?:static )?[\w *]+\b' + name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise RuntimeError(f'Cannot extract {name}')
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


source = (ROOT / 'main/bitaxe_hw.c').read_text()
code = (ROOT / 'tests/tps_read_retry_test.c').read_text()
helpers = re.search(r'^#define TPS546_STATUS_FAULT_MASK.*', source.replace('\\\n', ''), re.M).group() + '\n'
helpers += re.search(r'^#define TPS546_VOUT_TOLERANCE_VOLTS.*', source, re.M).group() + '\n'
for name in ['remember_power_extrema', 'safety_shutdown', 'fail_regulator_safety',
             'validate_regulator_safety', 'validate_regulator_startup_snapshot',
             'is_tps_communication_error', 'log_tps_read_retry', 'capture_power_snapshot',
             'bitaxe_gamma602_set_voltage_mv_for_config_unlocked']:
    helpers += function(source, name)
with tempfile.TemporaryDirectory(prefix='m45-tps-read-retry-') as directory:
    out = pathlib.Path(directory)
    (out / 'test.c').write_text(code.replace('/* POWER_VALIDATION_UNDER_TEST */', helpers))
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g',
                    '-I' + str(ROOT / 'tests/diagnostic_stubs'), '-I' + str(ROOT / 'main'),
                    str(out / 'test.c'), str(ROOT / 'main/tps546_diagnostics.c'),
                    str(ROOT / 'main/m45_fault_context.c'),
                    str(ROOT / 'main/m45_telemetry_extrema.c'),
                    '-lm', '-o', str(out / 'test')], check=True)
    subprocess.run([str(out / 'test')], check=True)
