#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motion.h"

#define CLOCK_HZ 125000000u
#define PI 3.14159265358979323846

static double pwm_rate(step_pwm_t pwm) {
  double div16 = pwm.div_int * 16.0 + pwm.div_frac;
  return CLOCK_HZ * 16.0 / (div16 * (pwm.wrap + 1.0));
}

static void assert_rate(double rate, double tolerance) {
  step_pwm_t pwm = step_pwm_for_rate(CLOCK_HZ, rate);
  assert(pwm.level > 0);
  assert(pwm.div_int >= 1);
  assert(fabs(pwm_rate(pwm) - rate) <= rate * tolerance);
  assert(fabs(pwm.level - (pwm.wrap + 1.0) / 2.0) <= 0.5);
}

static void test_step_rate(void) {
  assert(fabs(wheel_step_rate(2.0 * PI) - 200.0) < 1e-9);
  assert(fabs(wheel_step_rate(-2.0 * PI) - 200.0) < 1e-9);
  assert(wheel_step_rate(0.0) == 0.0);
}

static void test_direction(void) {
  assert(wheel_direction(1.0, false));
  assert(!wheel_direction(-1.0, false));
  assert(!wheel_direction(1.0, true));
  assert(wheel_direction(-1.0, true));
}

static void test_pwm(void) {
  assert_rate(8.0, 0.001);
  assert_rate(200.0, 0.001);
  assert_rate(1234.5, 0.001);
  assert_rate(10000.0, 0.001);
  assert_rate(100000.0, 0.005);

  assert(step_pwm_for_rate(CLOCK_HZ, 0.0).level == 0);
  assert(step_pwm_for_rate(CLOCK_HZ, -5.0).level == 0);
  assert(step_pwm_for_rate(CLOCK_HZ, 7.0).level == 0);
  assert(step_pwm_for_rate(CLOCK_HZ, NAN).level == 0);
  assert(step_pwm_for_rate(CLOCK_HZ, INFINITY).level == 0);
  assert(step_pwm_for_rate(CLOCK_HZ, CLOCK_HZ).level == 0);
}

static void test_wheel_index(void) {
  assert(wheel_index("wheel_1_joint") == 0);
  assert(wheel_index("wheel_3_joint") == 2);
  assert(wheel_index("wheel_4_joint") == -1);
  assert(wheel_index("") == -1);
  assert(wheel_index(NULL) == -1);
}

static void test_command(void) {
  double out[WHEEL_COUNT] = {0.0, 0.0, 0.0};
  double v[WHEEL_COUNT] = {1.0, -2.0, 3.0};

  char *ordered[] = {"wheel_1_joint", "wheel_2_joint", "wheel_3_joint"};
  assert(wheel_command(ordered, 3, v, 3, out));
  assert(out[0] == 1.0 && out[1] == -2.0 && out[2] == 3.0);

  char *shuffled[] = {"wheel_3_joint", "wheel_1_joint", "wheel_2_joint"};
  assert(wheel_command(shuffled, 3, v, 3, out));
  assert(out[0] == -2.0 && out[1] == 3.0 && out[2] == 1.0);

  assert(wheel_command(NULL, 0, v, 3, out));
  assert(out[0] == 1.0 && out[1] == -2.0 && out[2] == 3.0);

  out[0] = out[1] = out[2] = 9.0;
  char *duplicate[] = {"wheel_1_joint", "wheel_1_joint", "wheel_2_joint"};
  assert(!wheel_command(duplicate, 3, v, 3, out));
  char *unknown[] = {"wheel_1_joint", "arm_joint", "wheel_2_joint"};
  assert(!wheel_command(unknown, 3, v, 3, out));
  char *missing[] = {"wheel_1_joint", NULL, "wheel_2_joint"};
  assert(!wheel_command(missing, 3, v, 3, out));
  assert(!wheel_command(ordered, 2, v, 3, out));
  assert(!wheel_command(ordered, 3, v, 2, out));
  double not_finite[WHEEL_COUNT] = {1.0, NAN, 3.0};
  assert(!wheel_command(ordered, 3, not_finite, 3, out));
  assert(out[0] == 9.0 && out[1] == 9.0 && out[2] == 9.0);
}

int main(void) {
  test_step_rate();
  test_direction();
  test_pwm();
  test_wheel_index();
  test_command();
  printf("test_motion: all passed\n");
  return 0;
}
