#!/usr/bin/env bash
# Regenerates include/mobile_base_types.h from the interfaces of the ROS 2
# distribution the robot runs and from harp3_msgs, then checks every type hash
# against the .json files rosidl installed. Commit the result: the firmware
# build needs no Python.
#
#   tools/gen_types.sh
#
# ROS_SHARE       installed interfaces       (default /opt/ros/lyrical/share)
# HARP3_MSGS      harp3_msgs source          (default ~/HARP3_ROS/harp3_msgs)
# HARP3_MSGS_SHARE  built harp3_msgs share, to check its hashes too
# PICOROS_USB_ETH_PATH  picoros-usb-eth checkout holding the Pico-ROS generator
#                       (default ../lib/picoros-usb-eth)
set -euo pipefail

here=$(cd "$(dirname "$0")/.." && pwd)
ros_share=${ROS_SHARE:-/opt/ros/lyrical/share}
harp3_msgs=${HARP3_MSGS:-$HOME/HARP3_ROS/harp3_msgs}
kit=${PICOROS_USB_ETH_PATH:-$here/../lib/picoros-usb-eth}
generator=$kit/third_party/Pico-ROS/tools/type-gen/generate_type_header.py
work=$here/build/type-gen
packages=(builtin_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs std_srvs service_msgs)

if [ ! -d "$(dirname "$generator")/rosidl/rosidl_generator_type_description" ]; then
  echo "Pico-ROS generator incomplete: git -C $kit/third_party/Pico-ROS submodule update --init tools/type-gen/rosidl" >&2
  exit 1
fi

# The generator does not follow symbolic links: copy the definitions.
copy_package() {
  local src=$1 dst=$work/interfaces/$(basename "$1")
  mkdir -p "$dst"
  cp -f "$src/package.xml" "$dst/"
  for kind in msg srv; do
    if compgen -G "$src/$kind/*.$kind" > /dev/null; then
      mkdir -p "$dst/$kind"
      cp -f "$src/$kind"/*."$kind" "$dst/$kind/"
    fi
  done
}

rm -rf "$work/interfaces" "$work/out"
mkdir -p "$work/interfaces"
for p in "${packages[@]}"; do
  copy_package "$ros_share/$p"
done
copy_package "$harp3_msgs"

if [ ! -x "$work/venv/bin/python" ]; then
  python3 -m venv "$work/venv"
  "$work/venv/bin/pip" install -q -r "$(dirname "$generator")/requirements.txt"
fi

(cd "$work" && "$work/venv/bin/python" "$generator" --packages-dir interfaces --output-dir out \
  --header-file mobile_base_types.h) > "$work/generator.log" 2>&1 ||
  { cat "$work/generator.log" >&2; exit 1; }

shares=("$ros_share")
if [ -n "${HARP3_MSGS_SHARE:-}" ]; then
  shares+=("$HARP3_MSGS_SHARE")
fi
python3 "$here/tools/check_type_hashes.py" "$work/out/mobile_base_types.h" "${shares[@]}"
cp -f "$work/out/mobile_base_types.h" "$here/include/mobile_base_types.h"
echo "include/mobile_base_types.h updated"
