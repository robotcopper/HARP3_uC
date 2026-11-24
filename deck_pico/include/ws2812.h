/*
 * WS2812B strip on a PIO state machine, fed by DMA: showing a frame costs the
 * CPU a copy of LED_COUNT words, not the 9 ms the strip takes to shift it in.
 */

#ifndef DECK_PICO_WS2812_H
#define DECK_PICO_WS2812_H

#include <stdbool.h>
#include <stdint.h>

#include "led_view.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Claims a state machine of PIO0 and a DMA channel, and drives pin, with the
   inverse of the frame when inverted. Fills the colour correction tables. */
void ws2812_init(uint32_t pin, bool inverted);

/* Sends one frame. The colours are a screen's: each channel goes through a gamma
   of 2.2 and the strip's colour correction, BOARD_LED_GAIN_* in board.h, so that
   the strip shows what a screen would, then is scaled by brightness / 255.
   Skipped, returning 0, if the previous frame is still going out; 1 once it is on
   its way. The strip latches on the pause until the next frame: call it no more
   often than every few milliseconds. */
int ws2812_show(const led_rgb_t *leds, uint8_t brightness);

/* Frames sent since start-up. */
uint32_t ws2812_frames_sent(void);

/* Sends the last frame again and counts the level changes read back on the pin
   during its first 100 us: none means the state machine drives nothing. Waits for
   the frame going out, up to 9 ms. */
uint32_t ws2812_count_edges(void);

#ifdef __cplusplus
}
#endif

#endif /* DECK_PICO_WS2812_H */
