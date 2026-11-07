#!/usr/bin/env python3
"""Exercise the real ASIC temperature safety path with a controlled sensor/clock."""
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
helpers = '\n'.join(re.search(r'^#define ' + name + r' .*$', source, re.M).group(0)
                    for name in ['ASIC_TEMP_NO_READING_C', 'ASIC_TEMP_UNAVAILABLE_TIMEOUT_US'])
helpers += '\n' + '\n'.join(function(source, name) for name in [
    'asic_temp_is_no_reading', 'unavailable_asic_temperature', 'update_asic_temperature'])
code = (ROOT / 'tests/asic_temperature_test.c').read_text()
with tempfile.TemporaryDirectory(prefix='m45-asic-temperature-') as directory:
    out = pathlib.Path(directory)
    (out / 'test.c').write_text(code.replace('/* TEMPERATURE_UNDER_TEST */', helpers))
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g', str(out / 'test.c'),
                    '-lm', '-o', str(out / 'test')], check=True)
    subprocess.run([str(out / 'test')], check=True)
