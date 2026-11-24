/*
 * Pin assignment and tuning of the HARP3 deck board. The IMU
 * bus is the one BNO08x_Pico_Library uses: I2C0, SDA GP4, SCL GP5, address
 * 0x4A, in its utils.h.
 */

#ifndef DECK_PICO_BOARD_H
#define DECK_PICO_BOARD_H

/* WS2812B data in, physical pin 1. GP0 is UART0's TX, so this firmware has no
   UART console; see CMakeLists.txt. */
#define BOARD_LED_STRIP_PIN 0

/* 1 when DIN is driven through an inverting stage, such as one NPN transistor
   pulling it down against a pull-up to the strip's 5 V: the pin then outputs the
   inverse of the frame. */
#define BOARD_LED_STRIP_INVERTED 0

/* LEDs sent each frame. The ring is the first LED_COUNT, 94 cm; the rest are sent
   black, since a WS2812B keeps its last colour until told otherwise, so that nothing
   past 94 cm lights on a longer strip, up to a 5 m reel at 60 per metre. 300 LEDs
   take 9 ms of a 20 ms frame. */
#define BOARD_LED_STRIP_PIXELS 300

/* Colour correction of the strip, out of 255 per channel: a WS2812B's green and blue
   outshine its red, and full scale on all three is a cold white. FastLED's
   TypicalLEDStrip figures, 0xFFB0F0. */
#define BOARD_LED_GAIN_R 255
#define BOARD_LED_GAIN_G 176
#define BOARD_LED_GAIN_B 240

/* Ceiling of the strip's output, out of 255. The ring's 57 LEDs lit white at full
   scale draw about 3.4 A. The gradient's colours are dark once gamma-corrected:
   carmine, the brightest, drives red at 31 % of the ceiling, so the ring draws
   under 0.3 A at 160. */
#define BOARD_LED_BRIGHTNESS 160

#endif /* DECK_PICO_BOARD_H */
