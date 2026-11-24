#!/usr/bin/env bash
# Host unit tests of the hardware-free code, and a check that led_map.h is what
# its generator writes.
set -euo pipefail
here=$(cd "$(dirname "$0")/.." && pwd)
out=$here/build/host_test
mkdir -p "$out"
python3 "$here/tools/gen_led_map.py" --check
cc -std=c11 -Wall -Wextra -Werror -O1 -fsanitize=address,undefined -I"$here/include" \
  "$here/src/led_view.c" "$here/test/test_led_view.c" -lm -o "$out/test_led_view"
"$out/test_led_view"
