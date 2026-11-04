#include "otos.h"

#include "QwiicOtos.h"

namespace {

QwiicOTOS sensor;

otos_pose_t to_pose(const sfe_otos_pose2d_t &pose) { return otos_pose_t{pose.x, pose.y, pose.h}; }

}  // namespace

bool otos_init(bool cold_start) {
  /* A reboot can cut a transfer short and leave the sensor holding SDA low. */
  initI2C(!cold_start);
  if (sensor.isConnected() != kSTkErrOk) {
    return false;
  }
  sensor.setLinearUnit(kSfeOtosLinearUnitMeters);
  sensor.setAngularUnit(kSfeOtosAngularUnitRadians);
  if (!cold_start) {
    return true;
  }
  return sensor.calibrateImu() == kSTkErrOk && sensor.resetTracking() == kSTkErrOk;
}

bool otos_read(otos_sample_t *sample) {
  sfe_otos_pose2d_t position, velocity, acceleration, position_std_dev, velocity_std_dev, acceleration_std_dev;
  if (sensor.getPosVelAccAndStdDev(position, velocity, acceleration, position_std_dev, velocity_std_dev,
                                   acceleration_std_dev) != kSTkErrOk) {
    return false;
  }
  sample->position = to_pose(position);
  sample->velocity = to_pose(velocity);
  sample->position_std_dev = to_pose(position_std_dev);
  sample->velocity_std_dev = to_pose(velocity_std_dev);
  return true;
}

bool otos_reset_tracking(void) { return sensor.resetTracking() == kSTkErrOk; }
