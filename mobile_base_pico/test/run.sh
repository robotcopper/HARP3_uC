#!/usr/bin/env bash
# Host unit tests of the hardware-free code.
set -euo pipefail
here=$(cd "$(dirname "$0")/.." && pwd)
out=$here/build/host_test
mkdir -p "$out"
cc -std=c11 -Wall -Wextra -Werror -O1 -fsanitize=address,undefined -I"$here/include" \
  "$here/src/motion.c" "$here/test/test_motion.c" -lm -o "$out/test_motion"
"$out/test_motion"
