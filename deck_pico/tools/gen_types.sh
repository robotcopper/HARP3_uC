#!/usr/bin/env bash
# Regenerates include/deck_pico_types.h: the message types of sensor_msgs and its
# dependencies, with the generator and the checks of mobile_base_pico. This
# board exchanges no harp3_msgs type.
#
#   tools/gen_types.sh
#
# The environment variables of mobile_base_pico/tools/gen_types.sh apply.
set -euo pipefail

here=$(cd "$(dirname "$0")/.." && pwd)
exec env TYPES_HEADER=deck_pico_types.h TYPES_OUT_DIR="$here/include" WITH_HARP3_MSGS=0 \
  PICOROS_USB_ETH_PATH="${PICOROS_USB_ETH_PATH:-$here/../lib/picoros-usb-eth}" \
  "$here/../mobile_base_pico/tools/gen_types.sh"
