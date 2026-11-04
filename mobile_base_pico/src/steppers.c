#include "steppers.h"

#include "board.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/time.h"

#define CUTOFF_CHECK_MS 10

static const uint step_pins[WHEEL_COUNT] = BOARD_STEP_PINS;
static const uint dir_pins[WHEEL_COUNT] = BOARD_DIR_PINS;
static const uint enable_pins[WHEEL_COUNT] = BOARD_ENABLE_PINS;
static const bool dir_inverted[WHEEL_COUNT] = BOARD_DIR_INVERTED;

/* Shared with the cut-off interrupt: tasks write them with interrupts disabled. */
static volatile bool enabled = false;
static volatile bool running = false;
static volatile uint64_t last_command_us = 0;
static uint64_t timeout_us = 0;
static repeating_timer_t cutoff_timer;

static void stop_pulses(void) {
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    pwm_set_gpio_level(step_pins[i], 0);
  }
  running = false;
}

static bool cutoff_cb(repeating_timer_t *timer) {
  (void)timer;
  if (running && time_us_64() - last_command_us > timeout_us) {
    stop_pulses();
  }
  return true;
}

void steppers_init(uint32_t timeout_ms) {
  timeout_us = (uint64_t)timeout_ms * 1000u;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    /* Latch the level before driving the pin, so ENABLE never glitches low. */
    gpio_init(enable_pins[i]);
    gpio_put(enable_pins[i], 1);
    gpio_set_dir(enable_pins[i], GPIO_OUT);
    gpio_init(dir_pins[i]);
    gpio_put(dir_pins[i], 0);
    gpio_set_dir(dir_pins[i], GPIO_OUT);
    pwm_set_gpio_level(step_pins[i], 0);
    gpio_set_function(step_pins[i], GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(step_pins[i]), true);
  }
  add_repeating_timer_ms(-CUTOFF_CHECK_MS, cutoff_cb, NULL, &cutoff_timer);
}

void steppers_enable(bool enable) {
  uint32_t irq = save_and_disable_interrupts();
  if (!enable) {
    stop_pulses();
  }
  enabled = enable;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    gpio_put(enable_pins[i], !enable);
  }
  restore_interrupts(irq);
}

bool steppers_command(const double wheel_velocity[WHEEL_COUNT]) {
  uint32_t clock_hz = clock_get_hz(clk_sys);
  step_pwm_t pwm[WHEEL_COUNT];
  bool dir[WHEEL_COUNT];
  bool moving = false;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    pwm[i] = step_pwm_for_rate(clock_hz, wheel_step_rate(wheel_velocity[i]));
    dir[i] = wheel_direction(wheel_velocity[i], dir_inverted[i]);
    moving = moving || pwm[i].level != 0;
  }

  uint32_t irq = save_and_disable_interrupts();
  bool accepted = enabled;
  if (accepted) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      uint slice = pwm_gpio_to_slice_num(step_pins[i]);
      gpio_put(dir_pins[i], dir[i]);
      pwm_set_clkdiv_int_frac4(slice, pwm[i].div_int, pwm[i].div_frac);
      pwm_set_wrap(slice, pwm[i].wrap);
      pwm_set_gpio_level(step_pins[i], pwm[i].level);
    }
    last_command_us = time_us_64();
    running = moving;
  }
  restore_interrupts(irq);
  return accepted;
}

bool steppers_running(void) { return running; }
