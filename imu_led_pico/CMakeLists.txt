cmake_minimum_required(VERSION 3.13)

# HARP3 IMU and obstacle ring firmware: Pico-ROS over Ethernet over USB (CDC-NCM)
# and TCP, against rmw_zenoh_cpp, on picoros-usb-eth's rp2040 platform. Same
# stack as mobile_base_pico.

set(PICO_BOARD "pico" CACHE STRING "")
set(PICOROS_USB_ETH_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/picoros-usb-eth" CACHE PATH "picoros-usb-eth checkout")

# mobile_base_pico owns 192.168.7.0/24. Two boards on one host need two
# subnets, or the host cannot tell which interface 192.168.7.1 is on.
set(USB_NET_DEVICE_IP "192.168.8.1" CACHE STRING "board address on the USB link")
set(USB_NET_HOST_IP "192.168.8.2" CACHE STRING "address leased to the host, and SNTP server")

# A LaserScan of 500 beams with intensities is about 4.1 kB, above zenoh-pico's
# default of 4096 for a reassembled message, which would drop every scan.
set(FRAG_MAX_SIZE 8192 CACHE STRING "largest fragmented message zenoh-pico reassembles")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include("${PICOROS_USB_ETH_PATH}/picoros_usb_eth_import.cmake")

project(imu_led_pico C CXX ASM)

pico_sdk_init()

picoros_usb_eth_init(
  TYPES_HEADER imu_led_types.h
  TYPES_DIR "${CMAKE_CURRENT_LIST_DIR}/include"
)

add_subdirectory(lib/BNO08x_Pico_Library)

set(IMU_LED_SOURCES
  src/app_main.c
  src/imu.cpp
  src/led_view.c
  src/ws2812.c
)
add_executable(imu_led_pico ${IMU_LED_SOURCES})
# The SDK and FreeRTOS sources compiled into the executable keep their own
# warning level.
set_source_files_properties(${IMU_LED_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
target_include_directories(imu_led_pico PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
pico_generate_pio_header(imu_led_pico "${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio")
picoros_usb_eth_add_firmware(imu_led_pico)
# The strip takes GP0, UART0's TX: no console, imu_led_pico/status stands in for it.
pico_enable_stdio_uart(imu_led_pico 0)
target_link_libraries(imu_led_pico PRIVATE
  BNO08x_Pico_Library
  hardware_dma
  hardware_i2c
  hardware_pio
)
