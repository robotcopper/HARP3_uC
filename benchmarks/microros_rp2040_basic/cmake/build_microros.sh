#!/bin/bash
# Builds libmicroros.a as micro_ros_setup's build_firmware.sh does for
# generate_lib: colcon over mcu_ws and the given package directories, in an
# environment holding only dev_ws, then every static library merged into one
# archive and the include tree flattened to one directory.
#
# Usage: build_microros.sh <firmware dir> <build dir> <ar> <ranlib> [<package dir>...]
#
# <firmware dir> is the directory made by create_firmware_ws.sh generate_lib.
# <build dir> holds the toolchain.cmake and colcon.meta written by
# microros.cmake, and receives libmicroros.a and include/.

set -euo pipefail

if [ -z "${MICROROS_CLEAN_ENV:-}" ]; then
  exec env -i MICROROS_CLEAN_ENV=1 HOME="$HOME" PATH=/usr/bin:/bin TERM="${TERM:-dumb}" "$0" "$@"
fi

fw_dir=$1
build_dir=$2
ar=$3
ranlib=$4
shift 4

set +u
source "$fw_dir/dev_ws/install/setup.bash"
set -u

# rmw_zenoh_pico replaces rmw_microxrcedds, so the XRCE-DDS client goes too.
colcon --log-base "$build_dir/log" build \
  --base-paths "$fw_dir/mcu_ws" "$@" \
  --build-base "$build_dir/build" \
  --install-base "$build_dir/install" \
  --merge-install \
  --packages-ignore-regex '.*_cpp' \
  --packages-ignore rmw_microxrcedds microxrcedds_client \
  --metas "$fw_dir/mcu_ws/colcon.meta" "$build_dir/colcon.meta" \
  --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
    -DTHIRDPARTY=ON \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$build_dir/toolchain.cmake" \
    -DRMW_IMPLEMENTATION=rmw_zenoh_pico \
    -DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON

# ADDLIB keeps every member, where extracting with ar x would let zenoh-pico's
# unicast and multicast rx.c, among others, overwrite each other.
lib="$build_dir/libmicroros.a"
if [ ! -f "$lib" ] || [ -n "$(find "$build_dir/install/lib" -name '*.a' -newer "$lib" -print -quit)" ]; then
  {
    echo "CREATE $lib.tmp"
    find "$build_dir/install/lib" -name '*.a' | sort | sed 's/^/ADDLIB /'
    echo "SAVE"
    echo "END"
  } | "$ar" -M
  "$ranlib" "$lib.tmp"
  mv "$lib.tmp" "$lib"
  echo "libmicroros.a: $(find "$build_dir/install/lib" -name '*.a' | wc -l) libraries merged"
fi

# include/<pkg>/<pkg>/ becomes include/<pkg>/, so one include directory
# serves every package. cp -a keeps the timestamps the firmware's
# dependency tracking compares.
staging="$build_dir/include.tmp"
rm -rf "$staging"
cp -a "$build_dir/install/include" "$staging"
for dir in "$staging"/*/; do
  pkg=$(basename "$dir")
  if [ -d "$dir$pkg" ]; then
    cp -a "$dir$pkg/." "$dir"
    rm -rf "$dir$pkg"
  fi
done
rm -rf "$build_dir/include"
mv "$staging" "$build_dir/include"
