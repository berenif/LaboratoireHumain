#!/usr/bin/env bash
# Build only the opt-in read-only helper against the retained official source ABI.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
base="${1:?usage: build_source_step_probe.sh SOURCE_SESSION OUTPUT_LIBRARY}"
output="${2:?usage: build_source_step_probe.sh SOURCE_SESSION OUTPUT_LIBRARY}"
lib="$(cd "$base/dependency/official-wheel/mujoco" && pwd)"
mkdir -p "$(dirname "$output")"
gcc -O2 -fPIC -shared "$root/scripts/getup-step/source_dynamics.c" \
  -I"$lib/include" -L"$lib" -Wl,-rpath,"$lib" \
  -l:libmujoco_nogl.so.2.1.3 -o "$output"
