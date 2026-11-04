# HARP3_uC

Microcontroller firmware of the HARP3 robot. The ROS side lives in
[HARP3_ROS](https://github.com/robotcopper/HARP3_ROS), which holds the
interfaces this firmware is generated from.

| Directory | Content |
| --- | --- |
| [`mobile_base_pico/`](mobile_base_pico/README.md) | mobile base board: three steppers, odometry sensor, limit switches |
| `lib/picoros-usb-eth/` | submodule, [picoros-usb-eth](https://github.com/robotcopper/picoros-usb-eth): Pico-ROS over Ethernet over USB, shared by every board |

## Prerequisites

* The Arm GNU toolchain (`arm-none-eabi-gcc`, tested with 14.2), CMake 3.13 or
  later, and Ninja.
* The [Pico SDK](https://github.com/raspberrypi/pico-sdk), tested with 2.2.0,
  with its submodules.
* The [FreeRTOS kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel), which holds
  the RP2040 port, tested at commit 8be86d4.
* [picotool](https://github.com/raspberrypi/picotool), tested with 2.2.0, and its
  udev rule `60-picotool.rules`, to flash without root.
* On the host the board is plugged into: the router, `chrony` and the firewall
  described in picoros-usb-eth's
  [host setup](https://github.com/robotcopper/picoros-usb-eth#host-setup).

## Get the sources

```sh
git clone --recurse-submodules git@github.com:robotcopper/HARP3_uC.git
```

In an existing clone, after a pull that moved a submodule:

```sh
git submodule update --init --recursive
```

## Build

```sh
export PICO_SDK_PATH=~/pico/pico-sdk
export FREERTOS_KERNEL_PATH=~/src/FreeRTOS-Kernel
cd mobile_base_pico
cmake -S . -B build/rp2040 -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2040
```

The firmware is `build/rp2040/mobile_base_pico.uf2`. The host unit tests run
with `test/run.sh`.

## Flash

### A running board, without the BOOTSEL button

Through the same USB cable, with the board running this firmware:

```sh
picotool load -f -x build/rp2040/mobile_base_pico.uf2
```

`-f` asks the running firmware, through its USB reset interface, to reboot into
BOOTSEL mode; `picotool` then writes the image and `-x` starts it. The board's
network interface disappears for a few seconds and comes back with the same
name. The restart is warm: the odometry sensor keeps its pose.

### First flash, or a board that does not answer

Hold the BOOTSEL button while plugging the board in, then:

```sh
picotool load -x build/rp2040/mobile_base_pico.uf2
```

A board that has hung ignores `-f`: unplug and replug it, which restarts its
firmware, or use BOOTSEL.

## Check

With the router running on the host:

```sh
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ip -br addr | grep enx            # the board's interface, holding 192.168.7.2
ros2 node list                    # /mobile_base_pico
ros2 topic hz /odom/paa5160e1     # about 100 Hz
```

Echoing `/limit_switches` also needs `harp3_msgs` built and sourced from
HARP3_ROS.

## Update picoros-usb-eth

```sh
git submodule update --remote lib/picoros-usb-eth
git add lib/picoros-usb-eth
git submodule sync --recursive
git submodule update --init --recursive
git commit -m "build: bump picoros-usb-eth"
```

`git add` comes first: `git submodule update` checks out what the index
records, and would otherwise put the old version back.
