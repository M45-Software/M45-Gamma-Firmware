#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "$0")/.." && pwd)"
test_bin="$(mktemp /tmp/m45-fault-test.XXXXXX)"
trap 'rm -f "$test_bin"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -pthread \
    -I"$repo_dir/tests/diagnostic_stubs" -I"$repo_dir/tests/stubs" -I"$repo_dir/main" \
    "$repo_dir/tests/fault_diagnostics_test.c" "$repo_dir/main/tps546_diagnostics.c" \
    "$repo_dir/main/m45_fault_log.c" "$repo_dir/main/m45_error_log.c" \
    "$repo_dir/main/m45_fault_context.c" \
    "$repo_dir/main/m45_telemetry_extrema.c" \
    "$repo_dir/main/m45_log_buffer.c" -o "$test_bin"
"$test_bin"
