#!/bin/bash
# Makes microros/firmware, the workspace that cmake/microros.cmake builds
# libmicroros.a from: micro_ros_setup's generate_lib workspace, its repositories
# then checked out at the commits of microros.repos, those the comparison in
# docs/picoros_vs_microros was measured with.
#
# Needs micro_ros_setup for Lyrical built in a workspace and sourced:
#
#   source /opt/ros/lyrical/setup.bash
#   source ~/uros_ws/install/setup.bash
#   ./create_microros_ws.sh

set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
if [ -e "$here/microros/firmware" ]; then
  echo "$here/microros/firmware already exists" >&2
  exit 1
fi
mkdir -p "$here/microros"
cd "$here/microros"

# rosdep would install these with sudo; generate_lib uses none of them.
EXTERNAL_SKIP="python3-pytest-mock gperf wget" ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
vcs import --input "$here/microros.repos" firmware/mcu_ws
