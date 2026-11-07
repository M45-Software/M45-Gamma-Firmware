#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$repo_dir"

for test_script in \
    tests/run-asic-temperature.py \
    tests/run-hardware-startup.py \
    tests/run-tps-read-retry.py \
    tests/run-telemetry-extrema-store.py \
    tests/run-asic-rx-quiesce.py \
    tests/run-auto-clock-voltage.py; do
    python3 "$test_script"
done
for test_script in \
    tests/run-fault-diagnostics.sh \
    tests/run-tps-readback.sh \
    tests/run-chip-detection.sh; do
    bash "$test_script"
done
node --check web/root.js
node --test tests/*.test.cjs
