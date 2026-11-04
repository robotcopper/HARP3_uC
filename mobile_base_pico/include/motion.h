/*
 * Wheel command arithmetic, free of hardware access so that it builds and is
 * tested on the host.
 */

#ifndef MOBILE_BASE_MOTION_H
#define MOBILE_BASE_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_COUNT 3

/* 17HS19-2004S1. */
#define MOTOR_FULL_STEPS 200.0
/* MS1 to MS3 are unconnected on the board: full step. */
#define MICROSTEPS 1.0
/* Wheel on the motor shaft. */
#define GEAR_RATIO 1.0
#define STEPS_PER_WHEEL_RADIAN (MOTOR_FULL_STEPS * MICROSTEPS * GEAR_RATIO / (2.0 * 3.14159265358979323846))

/* PWM settings for one step pulse per period, at 50 % duty. */
typedef struct {
  uint8_t div_int;
  uint8_t div_frac;
  uint16_t wrap;
  uint16_t level; /* 0 stops the pulses */
} step_pwm_t;

/* Step rate in Hz for a wheel speed in rad/s. */
double wheel_step_rate(double wheel_velocity);

/* Level of the DIR pin for a wheel speed. */
bool wheel_direction(double wheel_velocity, bool inverted);

/* A stopped output below the slowest rate the 16-bit counter and 8-bit divider
   reach, about 7.5 Hz at 125 MHz, and above half the clock. */
step_pwm_t step_pwm_for_rate(uint32_t clock_hz, double step_rate);

/* Index of "wheel_1_joint" to "wheel_3_joint", or -1. */
int wheel_index(const char *joint_name);

/* Wheel velocities from a JointState command, by joint name, or by position
   when the message carries no names. False unless every wheel is set exactly
   once to a finite value. */
bool wheel_command(char *const *names, uint32_t name_count, const double *velocities, uint32_t velocity_count,
                   double out[WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* MOBILE_BASE_MOTION_H */
