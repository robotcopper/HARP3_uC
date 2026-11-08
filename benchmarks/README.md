# Benchmarks

The two test firmwares and the scripts behind
[the comparison of Pico-ROS and micro-ROS over Zenoh](../docs/picoros_vs_microros/README.md).
Both firmwares declare the same topics and service, so the same scripts measure
either.

| Directory | Content |
| --- | --- |
| [`picoros_rp2040_basic/`](picoros_rp2040_basic) | Pico-ROS firmware: picoros-usb-eth's `rp2040_basic` example with a `std_srvs/Trigger` service |
| [`microros_rp2040_basic/`](microros_rp2040_basic) | micro-ROS firmware: `rclc` and `rcl` over rmw_zenoh_pico, on picoros-usb-eth's rp2040 platform |
| `microros_rp2040_basic/lib/zenoh-pico/` | submodule, zenoh-pico 1.10.1 with the fixes the comparison found |
| `microros_rp2040_basic/lib/rmw_zenoh_pico/` | submodule, rmw_zenoh_pico for Lyrical with the same |
| [`tools/`](tools) | measurement, profiling and figure scripts |

## Prerequisites

Those of [the repository](../README.md#prerequisites), and for the micro-ROS
firmware, `micro_ros_setup` for Lyrical built in a workspace of its own:

```sh
mkdir -p ~/uros_ws/src
git clone -b lyrical https://github.com/micro-ROS/micro_ros_setup.git ~/uros_ws/src/micro_ros_setup
cd ~/uros_ws
source /opt/ros/lyrical/setup.bash
colcon build
```

The scripts need ROS 2 Lyrical with `rmw_zenoh_cpp` on the host, `ping`, and
for `figures.py`, matplotlib and Graphviz.

## Build

```sh
export PICO_SDK_PATH=~/pico/pico-sdk
export FREERTOS_KERNEL_PATH=~/src/FreeRTOS-Kernel
```

The Pico-ROS firmware:

```sh
cd picoros_rp2040_basic
cmake -S . -B build/rp2040 -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2040
```

The micro-ROS firmware first needs the micro-ROS workspace, made once by
`create_microros_ws.sh`: `micro_ros_setup`'s `generate_lib` workspace, checked
out at the commits of `microros.repos`.

```sh
cd microros_rp2040_basic
source /opt/ros/lyrical/setup.bash
source ~/uros_ws/install/setup.bash
./create_microros_ws.sh
cmake -S . -B build/rp2040 -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2040
```

The first build compiles `libmicroros.a`, some 75 packages, with colcon, in an
environment holding only the micro-ROS workspace. rmw_zenoh_pico replaces
rmw_microxrcedds, so the XRCE-DDS client is not built.

| Variable | Default | Meaning |
| --- | --- | --- |
| `PICO_USE_FASTEST_SUPPORTED_CLOCK` | `ON` | 200 MHz; `OFF` gives the SDK's 125 MHz |
| `MICROROS_FIRMWARE_DIR` | `microros/firmware` | the workspace `create_microros_ws.sh` made |
| `MICROROS_EXTRA_PACKAGES` | `lib/zenoh-pico`, `lib/rmw_zenoh_pico/rmw_zenoh_pico` | packages built along with it |

`hot_functions.txt` lists the functions the micro-ROS firmware runs from RAM.
They were chosen from a profile of this firmware; a change to the code may call
for a new profile, see `tools/hot_functions.py`.

## Flash

```sh
picotool load -f -x build/rp2040/<firmware>.uf2
```

## Measure

With `rmw_zenohd` and `chrony` running as described in picoros-usb-eth's
[host setup](https://github.com/robotcopper/picoros-usb-eth#host-setup):

```sh
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 tools/measure.py <label> --rate 99          # about 1 min 30 s
python3 tools/service_rtt.py <label>                # about 1 min
python3 tools/host_reference.py <label> --mode client
python3 tools/service_rtt.py <label> --host-reference client
```

Each writes `<label>_<date>_<time>.txt` to
[`docs/picoros_vs_microros/results`](../docs/picoros_vs_microros/results).
Measure with the host otherwise idle: a build running next to it shows in the
latencies.

| Script | Does |
| --- | --- |
| `measure.py` | idle time, uplink, downlink and their sum, with and without JointState; `--profile` also samples the micro-ROS firmware's program counter |
| `service_rtt.py` | late joiners on `pico/boot`, then 1000 calls of `pico/ping` |
| `host_reference.py` | the exchange of `measure.py` between two processes on the host, peer or client |
| `stamp_gap.py` | the gap between `header.stamp` and the rmw attachment's timestamp |
| `profile_report.py` | a saved profile mapped to functions and components of the ELF |
| `hot_functions.py` | the densest functions of a profile, for a RAM budget |
| `rename_hot.py` | moves those functions to RAM; run by the micro-ROS build |
| `figures.py` | redraws the charts and diagrams of the comparison from the results |
