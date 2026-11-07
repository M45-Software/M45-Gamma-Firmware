#!/usr/bin/env python3
"""Check actual extrema save/clear control flow with simulated persistent NVS."""
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
helpers = '\n'.join(function(source, name) for name in [
    'bitaxe_gamma602_telemetry_extrema', 'persist_telemetry_extrema',
    'bitaxe_gamma602_clear_telemetry_extrema'])
code = (ROOT / 'tests/telemetry_extrema_store_test.c').read_text()
with tempfile.TemporaryDirectory(prefix='m45-extrema-store-') as directory:
    out = pathlib.Path(directory)
    (out / 'test.c').write_text(code.replace('/* EXTREMA_CONTROL_UNDER_TEST */', helpers))
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pthread',
                    '-fsanitize=address,undefined', '-g',
                    '-I' + str(ROOT / 'tests/diagnostic_stubs'), '-I' + str(ROOT / 'main'),
                    str(out / 'test.c'), str(ROOT / 'main/m45_telemetry_extrema.c'),
                    str(ROOT / 'main/m45_telemetry_extrema_store.c'),
                    '-lm', '-o', str(out / 'test')], check=True)
    subprocess.run([str(out / 'test')], check=True)
