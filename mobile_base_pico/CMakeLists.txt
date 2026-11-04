cmake_minimum_required(VERSION 3.13)

# HARP3 mobile base firmware: Pico-ROS over Ethernet over USB (CDC-NCM) and TCP,
# against rmw_zenoh_cpp, on picoros-usb-eth's rp2040 platform.

set(PICO_BOARD "pico" CACHE STRING "")
set(PICOROS_USB_ETH_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/picoros-usb-eth" CACHE PATH "picoros-usb-eth checkout")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include("${PICOROS_USB_ETH_PATH}/picoros_usb_eth_import.cmake")

project(mobile_base_pico C CXX ASM)

pico_sdk_init()

picoros_usb_eth_init(
  TYPES_HEADER mobile_base_types.h
  TYPES_DIR "${CMAKE_CURRENT_LIST_DIR}/include"
)

add_subdirectory(lib/PAA5160E1_Pico_Library)
# OTOS SDA and SCL on GP26 and GP27, which belong to I2C1.
target_compile_definitions(Qwiic_OTOS_Library PUBLIC
  OTOS_I2C_PORT=i2c1
  OTOS_I2C_SDA_PIN=26
  OTOS_I2C_SCL_PIN=27
)

set(MOBILE_BASE_SOURCES
  src/app_main.c
  src/motion.c
  src/otos.cpp
  src/steppers.c
)
add_executable(mobile_base_pico ${MOBILE_BASE_SOURCES})
# The SDK and FreeRTOS sources compiled into the executable keep their own
# warning level.
set_source_files_properties(${MOBILE_BASE_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
target_include_directories(mobile_base_pico PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
picoros_usb_eth_add_firmware(mobile_base_pico)
target_link_libraries(mobile_base_pico PRIVATE
  Qwiic_OTOS_Library
  hardware_i2c
  hardware_pwm
)
