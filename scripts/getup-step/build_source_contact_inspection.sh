#!/usr/bin/env bash
# Build only the read-only inspector; no native helper/controller is rebuilt.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
base="${1:?usage: build_source_contact_inspection.sh SOURCE_SESSION OUTPUT_LIBRARY}"
output="${2:?usage: build_source_contact_inspection.sh SOURCE_SESSION OUTPUT_LIBRARY}"
lib="$(cd "$base/dependency/official-wheel/mujoco" && pwd -P)"
test -r "$lib/include/mujoco.h"
test -r "$lib/libmujoco_nogl.so.2.1.3"
mkdir -p "$(dirname "$output")"
"${CC:-cc}" -O2 -std=c99 -shared -fPIC -fvisibility=hidden \
  -I"$lib/include" "$root/scripts/getup-step/source_contact_inspection.c" \
  "$lib/libmujoco_nogl.so.2.1.3" -Wl,-rpath,"$lib" -o "$output"
