cmake_minimum_required(VERSION 3.13)

# picoros-usb-eth's rp2040_basic example with a service: the Pico-ROS side of
# docs/picoros_vs_microros. A Pico-ROS node on a bare Raspberry Pi Pico over
# Ethernet over USB and TCP, stamped with the host's clock over SNTP. Its topics
# and service are listed in src/app_main.c.

set(PICO_BOARD "pico" CACHE STRING "Pico SDK board")
# 200 MHz, the fastest clock the SDK supports on RP2040: it sets the PLL and
# raises the core voltage to 1.15 V; the flash runs at clk_sys / 2, 100 MHz.
# Off, the SDK's 125 MHz.
set(PICO_USE_FASTEST_SUPPORTED_CLOCK ON CACHE BOOL "run the RP2040 at 200 MHz")
set(CMAKE_C_STANDARD 11)

include("${CMAKE_CURRENT_LIST_DIR}/../../lib/picoros-usb-eth/picoros_usb_eth_import.cmake")

project(picoros_rp2040_basic C CXX ASM)

pico_sdk_init()

picoros_usb_eth_init(
  TYPES_HEADER example_types.h
  TYPES_DIR "${PICOROS_USB_ETH_DIR}/third_party/Pico-ROS/examples"
  DEBUG
)

add_executable(picoros_rp2040_basic src/app_main.c)
picoros_usb_eth_add_firmware(picoros_rp2040_basic)
