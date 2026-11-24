#include "imu.h"

#include <cmath>

#include "FreeRTOS.h"
#include "bno08x.h"
#include "pico/stdlib.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "task.h"
#include "utils.h"

namespace {

constexpr uint32_t kReportIntervalUs = 10000;
/* Packets read per poll: about one per 10 ms, often carrying all three reports,
   and a few spare. */
constexpr int kMaxPacketsPerPoll = 8;
constexpr int kOpenAttempts = 3;
constexpr uint32_t kOpenRetryMs = 200;
constexpr uint64_t kSilenceUs = 500000;
/* Between two attempts to reopen a sensor that went quiet. */
constexpr uint64_t kReopenPeriodUs = 2000000;
/* Per address of the bus scan after a failed open: a stuck bus costs 112 of them. */
constexpr uint32_t kProbeTimeoutUs = 1000;
/* Per transfer. This task runs below the network's, which can hold it for
   milliseconds in the middle of one; the library's 5 ms then cuts the transfer
   short and leaves the sensor holding SDA. The controller keeps SCL low while it
   waits, which the sensor tolerates. */
constexpr uint32_t kTransferTimeoutUs = 50000;

#if IMU_USE_MAGNETOMETER
constexpr sh2_SensorId_t kOrientationReport = SH2_ROTATION_VECTOR;
#else
constexpr sh2_SensorId_t kOrientationReport = SH2_GAME_ROTATION_VECTOR;
#endif

BNO08x sensor;
i2c_inst_t *port = nullptr;
bool ready = false;
bool have_orientation = false;
bool have_gyro = false;
bool have_accel = false;
imu_sample_t latest;
uint64_t last_event_us = 0;
uint64_t last_open_us = 0;
uint32_t events_taken = 0;
char diagnosis[112] = "not opened";

bool answers(uint8_t address, uint32_t timeout_us) {
  uint8_t dummy;
  return i2c_read_timeout_us(port, address, &dummy, 1, false, timeout_us) > 0;
}

void diagnose(const char *step) {
  int n = snprintf(diagnosis, sizeof(diagnosis), "%s, sda %d scl %d, i2c answers:", step,
                   (int)gpio_get(CONFIG::I2C_SDA_PIN), (int)gpio_get(CONFIG::I2C_SCL_PIN));
  bool any = false;
  for (uint8_t address = 0x08; address < 0x78 && n + 6 < (int)sizeof(diagnosis); ++address) {
    if (answers(address, kProbeTimeoutUs)) {
      n += snprintf(diagnosis + n, sizeof(diagnosis) - n, " 0x%02x", address);
      any = true;
    }
  }
  if (!any) {
    snprintf(diagnosis + n, sizeof(diagnosis) - n, " none");
  }
}

void take_event(const sh2_SensorValue_t &value) {
  switch (value.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
      if (kOrientationReport == SH2_GAME_ROTATION_VECTOR) {
        latest.orientation_x = value.un.gameRotationVector.i;
        latest.orientation_y = value.un.gameRotationVector.j;
        latest.orientation_z = value.un.gameRotationVector.k;
        latest.orientation_w = value.un.gameRotationVector.real;
        latest.orientation_accuracy_rad = 0.0f;
        have_orientation = true;
      }
      break;
    case SH2_ROTATION_VECTOR:
      if (kOrientationReport == SH2_ROTATION_VECTOR) {
        latest.orientation_x = value.un.rotationVector.i;
        latest.orientation_y = value.un.rotationVector.j;
        latest.orientation_z = value.un.rotationVector.k;
        latest.orientation_w = value.un.rotationVector.real;
        latest.orientation_accuracy_rad = value.un.rotationVector.accuracy;
        have_orientation = true;
      }
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      latest.angular_velocity[0] = value.un.gyroscope.x;
      latest.angular_velocity[1] = value.un.gyroscope.y;
      latest.angular_velocity[2] = value.un.gyroscope.z;
      have_gyro = true;
      break;
    case SH2_ACCELEROMETER:
      latest.linear_acceleration[0] = value.un.accelerometer.x;
      latest.linear_acceleration[1] = value.un.accelerometer.y;
      latest.linear_acceleration[2] = value.un.accelerometer.z;
      have_accel = true;
      break;
    default:
      break;
  }
}

/* The library's own handler keeps only the last event of a packet, and the sensor
   packs the reports due together into one, so every event is taken here instead.
   begin() registers the library's handler again, so this one goes after it. */
void on_sensor_event(void *cookie, sh2_SensorEvent_t *event) {
  (void)cookie;
  sh2_SensorValue_t value;
  if (sh2_decodeSensorEvent(&value, event) == SH2_OK) {
    take_event(value);
    ++events_taken;
  }
}

bool enable_reports() {
  return sensor.enableReport(kOrientationReport, kReportIntervalUs) &&
         sensor.enableReport(SH2_GYROSCOPE_CALIBRATED, kReportIntervalUs) &&
         sensor.enableReport(SH2_ACCELEROMETER, kReportIntervalUs);
}

bool open_sensor() {
  last_open_us = time_us_64();
  have_orientation = have_gyro = have_accel = false;
  const char *step = "";
  for (int attempt = 0; attempt < kOpenAttempts; ++attempt) {
    if (!answers(CONFIG::BNO08X_ADDR, kTransferTimeoutUs)) {
      step = "no answer at 0x4a";
    } else if (!sensor.begin(CONFIG::BNO08X_ADDR, port)) {
      step = "sh2 did not start";
    } else if (sh2_setSensorCallback(on_sensor_event, nullptr) != SH2_OK || !enable_reports()) {
      step = "reports refused";
    } else {
      snprintf(diagnosis, sizeof(diagnosis), "open");
      last_event_us = time_us_64();
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(kOpenRetryMs));
  }
  diagnose(step);
  return false;
}

}  // namespace

bool imu_init(bool cold_start) {
  CONFIG::I2C_TIMEOUT_US = kTransferTimeoutUs;
  /* A reboot can cut a transfer short and leave the sensor holding SDA low. */
  initI2C(port, !cold_start);
  ready = open_sensor();
  return ready;
}

bool imu_ready(void) { return ready; }

uint32_t imu_event_count(void) { return events_taken; }

const char *imu_diagnosis(void) { return diagnosis; }

bool imu_poll(imu_sample_t *sample) {
  uint64_t now_us = time_us_64();
  if (!ready) {
    if (now_us - last_open_us >= kReopenPeriodUs) {
      /* Whatever silenced it may have left SDA held low. */
      initI2C(port, true);
      ready = open_sensor();
    }
    return false;
  }

  uint32_t before = events_taken;
  for (int n = 0; n < kMaxPacketsPerPoll; ++n) {
    uint32_t taken = events_taken;
    sh2_service();
    if (events_taken == taken) {
      break;
    }
  }
  bool got = events_taken != before;
  if (sensor.wasReset()) {
    /* The sensor restarted by itself and forgot its reports. */
    have_orientation = have_gyro = have_accel = false;
    ready = enable_reports();
    last_open_us = now_us;
    last_event_us = now_us;
    return false;
  }
  if (got) {
    last_event_us = now_us;
  } else if (now_us - last_event_us > kSilenceUs) {
    snprintf(diagnosis, sizeof(diagnosis), "went silent");
    ready = false;
    last_open_us = now_us;
    return false;
  }
  if (!got || !have_orientation || !have_gyro || !have_accel) {
    return false;
  }
  *sample = latest;
  return true;
}
