/*
 * Pin assignment and tuning of the HARP3 IMU and obstacle ring board. The IMU
 * bus is the one BNO08x_Pico_Library uses: I2C0, SDA GP4, SCL GP5, address
 * 0x4A, in its utils.h.
 */

#ifndef IMU_LED_BOARD_H
#define IMU_LED_BOARD_H

/* WS2812B data in, physical pin 1. GP0 is UART0's TX, so this firmware has no
   UART console; see CMakeLists.txt. */
#define BOARD_LED_STRIP_PIN 0

/* 1 when DIN is driven through an inverting stage, such as one NPN transistor
   pulling it down against a pull-up to the strip's 5 V: the pin then outputs the
   inverse of the frame. */
#define BOARD_LED_STRIP_INVERTED 0

/* Ceiling of the strip's output, out of 255. 94 LEDs lit white at full scale
   draw about 5.6 A; green or red alone at 48 draw under 0.5 A. */
#define BOARD_LED_BRIGHTNESS 48

#endif /* IMU_LED_BOARD_H */
