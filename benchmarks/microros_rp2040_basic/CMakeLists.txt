cmake_minimum_required(VERSION 3.19)

# micro-ROS (rclc over rcl) with rmw_zenoh_pico, over Ethernet over USB (CDC-NCM)
# and TCP, on picoros-usb-eth's rp2040 platform without Pico-ROS: the micro-ROS
# side of docs/picoros_vs_microros. Its topics and services are listed in
# src/app_main.c. libmicroros.a is built by colcon from this configure, see
# cmake/microros.cmake, over the workspace made by create_microros_ws.sh.

set(PICO_BOARD "pico" CACHE STRING "Pico SDK board")
# 200 MHz, the fastest clock the SDK supports on RP2040: it sets the PLL and
# raises the core voltage to 1.15 V; the flash runs at clk_sys / 2, 100 MHz.
# Off, the SDK's 125 MHz.
set(PICO_USE_FASTEST_SUPPORTED_CLOCK ON CACHE BOOL "run the RP2040 at 200 MHz")
set(MICROROS_FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/microros/firmware" CACHE PATH
  "firmware directory made by create_microros_ws.sh")
set(MICROROS_EXTRA_PACKAGES
  "${CMAKE_CURRENT_LIST_DIR}/lib/zenoh-pico"
  "${CMAKE_CURRENT_LIST_DIR}/lib/rmw_zenoh_pico/rmw_zenoh_pico"
  CACHE STRING "package directories built along with mcu_ws")

set(CMAKE_C_STANDARD 11)

include("${CMAKE_CURRENT_LIST_DIR}/../../lib/picoros-usb-eth/picoros_usb_eth_import.cmake")

project(microros_rp2040_basic C CXX ASM)

pico_sdk_init()

# What picoros_usb_eth_init() sets before the platform, without building
# zenoh-pico and Pico-ROS.
set(USB_NET_DEVICE_IP "192.168.7.1" CACHE STRING "board address on the USB link")
set(USB_NET_HOST_IP "192.168.7.2" CACHE STRING "address leased to the host, and SNTP server")
set(USB_NET_NETMASK "255.255.255.0" CACHE STRING "USB link netmask")
include("${PICOROS_USB_ETH_PLATFORM_DIR}/platform.cmake")

include(cmake/microros.cmake)

add_executable(microros_rp2040_basic
  src/app_main.c
  src/clock_gettime.c
)
set_source_files_properties(src/app_main.c src/clock_gettime.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
set_source_files_properties(src/clock_gettime.c PROPERTIES COMPILE_DEFINITIONS "${MICROROS_POSIX_CLOCK_DEFINITIONS}")
# panic() records its message for the next boot, see src/app_main.c.
target_compile_definitions(microros_rp2040_basic PRIVATE PICO_PANIC_FUNCTION=record_panic)
# The functions that ../tools/hot_functions.py chose from a profile run from
# RAM: ../tools/rename_hot.py renames their sections .time_critical.hot.*, which
# the SDK's linker script places in .data, which crt0 copies to RAM.
set(hot_list "${CMAKE_CURRENT_LIST_DIR}/hot_functions.txt")
add_custom_command(TARGET microros_rp2040_basic PRE_LINK
  COMMAND python3 "${CMAKE_CURRENT_LIST_DIR}/../tools/rename_hot.py" "${CMAKE_OBJCOPY}" "${hot_list}"
          "${CMAKE_CURRENT_BINARY_DIR}/hot_functions.applied"
          "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/microros_rp2040_basic.dir" "${microros_lib}"
  VERBATIM)
set_property(TARGET microros_rp2040_basic APPEND PROPERTY LINK_DEPENDS "${hot_list}")
target_compile_definitions(microros_rp2040_basic PRIVATE PICO_DIVIDER_IN_RAM=1)
target_link_libraries(microros_rp2040_basic PRIVATE
  microros
  hardware_watchdog
)
picoros_usb_eth_platform_firmware(microros_rp2040_basic)
