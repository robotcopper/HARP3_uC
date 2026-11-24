/*
 * C interface to the BNO085 IMU on I2C0 (BNO08x_Pico_Library). Not thread safe:
 * one task owns the sensor.
 */

#ifndef DECK_PICO_IMU_H
#define DECK_PICO_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 1 orients with the rotation vector, which fuses the magnetometer: absolute
   heading, but the stepper motors disturb it. 0, the default, uses the game
   rotation vector, which does not: its heading is relative to the one at
   start-up and drifts slowly. */
#ifndef IMU_USE_MAGNETOMETER
#define IMU_USE_MAGNETOMETER 0
#endif

/* The latest of each report, in the sensor's frame, SI units. */
typedef struct {
  /* Unit quaternion. */
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  /* Estimated error of the orientation in rad; 0 when the report gives none, as
     the game rotation vector does not. */
  float orientation_accuracy_rad;
  /* rad/s, calibrated. */
  float angular_velocity[3];
  /* m/s^2, gravity included (REP-145). */
  float linear_acceleration[3];
} imu_sample_t;

/* Opens the sensor and starts its reports at 100 Hz. A warm start first clocks
   out a transfer a reboot cut short. False if the sensor does not answer; it is
   then retried by imu_poll(). */
bool imu_init(bool cold_start);

/* Reads what the sensor has to say. True when something new arrived and every
   report has arrived at least once, with the latest of each in *sample. A sensor
   silent for half a second, or one that reset, is opened again. */
bool imu_poll(imu_sample_t *sample);

/* The sensor answers. */
bool imu_ready(void);

/* Reports received since start-up, of any kind. */
uint32_t imu_event_count(void);

/* "open", or why the sensor is not, with the addresses answering on its bus. */
const char *imu_diagnosis(void);

#ifdef __cplusplus
}
#endif

#endif /* DECK_PICO_IMU_H */
