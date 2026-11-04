#include "motion.h"

#include <math.h>
#include <string.h>

#define PWM_COUNTER_STEPS 65536.0
#define PWM_DIV16_MIN 16.0
#define PWM_DIV16_MAX (255.0 * 16.0 + 15.0)

double wheel_step_rate(double wheel_velocity) { return fabs(wheel_velocity) * STEPS_PER_WHEEL_RADIAN; }

bool wheel_direction(double wheel_velocity, bool inverted) { return (wheel_velocity >= 0.0) != inverted; }

step_pwm_t step_pwm_for_rate(uint32_t clock_hz, double step_rate) {
  step_pwm_t pwm = {.div_int = 1, .div_frac = 0, .wrap = 0, .level = 0};
  double clock = (double)clock_hz;
  if (!(step_rate > 0.0) || step_rate > clock / 2.0) {
    return pwm;
  }
  /* Divider in sixteenths: the smallest that keeps one period within the counter. */
  double div16 = ceil(clock * 16.0 / (step_rate * PWM_COUNTER_STEPS));
  if (div16 < PWM_DIV16_MIN) {
    div16 = PWM_DIV16_MIN;
  }
  if (div16 > PWM_DIV16_MAX) {
    return pwm;
  }
  double counts = floor(clock * 16.0 / (div16 * step_rate) + 0.5);
  if (counts > PWM_COUNTER_STEPS) {
    counts = PWM_COUNTER_STEPS;
  }
  uint32_t div = (uint32_t)div16;
  pwm.div_int = (uint8_t)(div / 16u);
  pwm.div_frac = (uint8_t)(div % 16u);
  pwm.wrap = (uint16_t)(counts - 1.0);
  pwm.level = (uint16_t)(counts / 2.0);
  return pwm;
}

int wheel_index(const char *joint_name) {
  static const char *const names[WHEEL_COUNT] = {"wheel_1_joint", "wheel_2_joint", "wheel_3_joint"};
  if (joint_name == NULL) {
    return -1;
  }
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    if (strcmp(joint_name, names[i]) == 0) {
      return i;
    }
  }
  return -1;
}

bool wheel_command(char *const *names, uint32_t name_count, const double *velocities, uint32_t velocity_count,
                   double out[WHEEL_COUNT]) {
  if (velocity_count != WHEEL_COUNT || (name_count != 0 && name_count != WHEEL_COUNT)) {
    return false;
  }
  double wheel[WHEEL_COUNT];
  bool seen[WHEEL_COUNT] = {false, false, false};
  for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
    int w = name_count == 0 ? (int)i : wheel_index(names[i]);
    if (w < 0 || seen[w] || !isfinite(velocities[i])) {
      return false;
    }
    seen[w] = true;
    wheel[w] = velocities[i];
  }
  memcpy(out, wheel, sizeof(wheel));
  return true;
}
