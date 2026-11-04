/*
 * C interface to the PAA5160E1 optical tracking odometry sensor. Not thread
 * safe: callers serialise access to the I2C bus.
 */

#ifndef MOBILE_BASE_OTOS_H
#define MOBILE_BASE_OTOS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float h;
} otos_pose_t;

/* Metres, radians, and their rates, as the sensor reports them. */
typedef struct {
  otos_pose_t position;
  otos_pose_t velocity;
  otos_pose_t position_std_dev;
  otos_pose_t velocity_std_dev;
} otos_sample_t;

/* A cold start calibrates the IMU and resets the tracking, so the robot must be
   at rest. A warm start keeps both: the sensor stays powered and tracking while
   the RP2040 reboots. False if the sensor does not answer. */
bool otos_init(bool cold_start);

bool otos_read(otos_sample_t *sample);

/* Back to the origin, heading zero. */
bool otos_reset_tracking(void);

#ifdef __cplusplus
}
#endif

#endif /* MOBILE_BASE_OTOS_H */
