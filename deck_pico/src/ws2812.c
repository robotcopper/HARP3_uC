#include "ws2812.h"

#include <math.h>

#include "board.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "ws2812.pio.h"

#define BIT_RATE_HZ 800000.0f
#define BITS_PER_WORD 24
#define PROBE_US 100
/* A screen's, which the strip's linear PWM lacks: without it, every mixed colour
   comes out paler and colder than on the screen it was chosen on. */
#define GAMMA 2.2f
#define FULL_SCALE 65535u

static PIO strip_pio;
static uint strip_sm;
static int strip_dma;
static uint strip_pin;
_Static_assert(BOARD_LED_STRIP_PIXELS >= LED_COUNT, "the ring is longer than the strip");

/* The ring, then the rest of the strip, which stays 0: black. */
static uint32_t frame[BOARD_LED_STRIP_PIXELS];
static uint32_t frames_sent;
/* Per channel, red, green, blue: (value / 255)^GAMMA * gain / 255, out of FULL_SCALE. */
static uint16_t corrected[3][256];

static void fill_correction(void) {
  const uint32_t gain[3] = {BOARD_LED_GAIN_R, BOARD_LED_GAIN_G, BOARD_LED_GAIN_B};
  for (int c = 0; c < 3; ++c) {
    for (int v = 0; v < 256; ++v) {
      float linear = powf((float)v / 255.0f, GAMMA) * (float)gain[c] / 255.0f;
      corrected[c][v] = (uint16_t)lroundf(linear * (float)FULL_SCALE);
    }
  }
}

static void send_frame(void) {
  dma_channel_set_read_addr((uint)strip_dma, frame, false);
  dma_channel_set_trans_count((uint)strip_dma, BOARD_LED_STRIP_PIXELS, true);
  ++frames_sent;
}

void ws2812_init(uint32_t pin, bool inverted) {
  fill_correction();
  strip_pin = pin;
  strip_pio = pio0;
  strip_sm = (uint)pio_claim_unused_sm(strip_pio, true);
  uint offset = pio_add_program(strip_pio, &ws2812_program);

  pio_gpio_init(strip_pio, pin);
  /* Sharp edges down the wire, for a 3.3 V signal into a 5 V strip. */
  gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
  /* Inverted, the pin idles high between frames, which holds DIN low. */
  gpio_set_outover(pin, inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
  pio_sm_set_consecutive_pindirs(strip_pio, strip_sm, pin, 1, true);

  pio_sm_config config = ws2812_program_get_default_config(offset);
  sm_config_set_sideset_pins(&config, pin);
  /* Most significant bit first, a word pulled every 24 bits: the 8 low bits stay unused. */
  sm_config_set_out_shift(&config, false, true, BITS_PER_WORD);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
  float cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
  sm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (BIT_RATE_HZ * cycles_per_bit));
  pio_sm_init(strip_pio, strip_sm, offset, &config);
  pio_sm_set_enabled(strip_pio, strip_sm, true);

  strip_dma = dma_claim_unused_channel(true);
  dma_channel_config dma = dma_channel_get_default_config((uint)strip_dma);
  channel_config_set_transfer_data_size(&dma, DMA_SIZE_32);
  channel_config_set_read_increment(&dma, true);
  channel_config_set_write_increment(&dma, false);
  channel_config_set_dreq(&dma, pio_get_dreq(strip_pio, strip_sm, true));
  dma_channel_configure((uint)strip_dma, &dma, &strip_pio->txf[strip_sm], frame, BOARD_LED_STRIP_PIXELS, false);
}

static uint32_t scale(int channel, uint8_t value, uint8_t brightness) {
  return ((uint32_t)corrected[channel][value] * brightness + FULL_SCALE / 2u) / FULL_SCALE;
}

int ws2812_show(const led_rgb_t *leds, uint8_t brightness) {
  if (dma_channel_is_busy((uint)strip_dma)) {
    return 0;
  }
  for (int i = 0; i < LED_COUNT; ++i) {
    /* Green, red, blue, in the top 24 bits. */
    frame[i] = (scale(1, leds[i].g, brightness) << 24) | (scale(0, leds[i].r, brightness) << 16) |
               (scale(2, leds[i].b, brightness) << 8);
  }
  send_frame();
  return 1;
}

uint32_t ws2812_frames_sent(void) { return frames_sent; }

uint32_t ws2812_count_edges(void) {
  while (dma_channel_is_busy((uint)strip_dma)) {
    tight_loop_contents();
  }
  send_frame();
  uint32_t edges = 0;
  bool level = gpio_get(strip_pin);
  uint64_t end_us = time_us_64() + PROBE_US;
  while (time_us_64() < end_us) {
    bool now = gpio_get(strip_pin);
    edges += now != level;
    level = now;
  }
  return edges;
}
