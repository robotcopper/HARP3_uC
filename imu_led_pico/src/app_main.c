/*
 * HARP3 IMU and obstacle ring node, on Pico-ROS over Ethernet over USB and TCP to
 * rmw_zenohd. The same stack as mobile_base_pico.
 *
 *   imu/bno085   sensor_msgs/Imu, 100 Hz, best effort, stamped with the host clock
 *                at the sensor read. Frame imu_link, in the sensor's axes.
 *   scan         sensor_msgs/LaserScan, subscribed, best effort. Each LED of the
 *                ring shows the nearest obstacle in the directions it faces, from
 *                the lidar: carmine at IGNORE_RANGE_M, under which beams are
 *                ignored, to forest green from FAR_RANGE_M.
 *   imu_led_pico/status
 *                std_msgs/String every 5 s, best effort. The board's only
 *                console: the strip takes GP0, UART0's TX.
 *
 * The host clock comes over SNTP from the computer the board is plugged into, and
 * the IMU waits for its first answer. A lost session restarts the board.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "hardware/watchdog.h"
#include "imu.h"
#include "imu_led_types.h"
#include "led_view.h"
#include "pico/stdlib.h"
#include "picoros.h"
#include "picoros_usb_eth/net_time.h"
#include "picoros_usb_eth/usb_net.h"
#include "picoserdes.h"
#include "task.h"
#include "ws2812.h"

#define TICK_MS 10
/* The ring is redrawn every second tick: 50 Hz, above the scan rate. */
#define TICKS_PER_LED_FRAME 2
#define RETRY_MS 1000
#define WAIT_FRAME_MS 20
/* Armed when the session opens and fed every tick; never disarmed. */
#define WATCHDOG_TIMEOUT_MS 5000
#define REPORT_PERIOD_MS 5000
/* A lidar silent for longer than this is no longer evidence that nothing is near. */
#define SCAN_TIMEOUT_US 500000ull

#define IMU_FRAME_ID "imu_link"
/* Data sheet nominal accuracies, as in the earlier micro-ROS example. */
#define GYRO_STD_RAD_S 0.054105f
#define ACCEL_STD_M_S2 0.35f
/* The game rotation vector reports no accuracy of its own. */
#define ORIENTATION_STD_RAD 0.035f

/* The most beams of one scan. The Tmini Plus gives about 500 at 8 Hz. */
#define MAX_BEAMS 720

/* The lidars the ring listens to. Each is its own topic and its own rotation in
   base_link: yaw 0 when the scan is already in base_link's axes, as the YDLIDAR
   launch file's identity transform has it. The ring shows the nearest of them all. */
typedef struct {
  const char *topic;
  float yaw_rad;
} lidar_source_t;

static const lidar_source_t lidar_sources[] = {
    {"scan", 0.0f},
};
#define LIDAR_COUNT (sizeof(lidar_sources) / sizeof(lidar_sources[0]))

static void scan_cb(uint8_t *rx, size_t size, const picoros_sample_info_t *info, void *user_data);

static picoros_node_t node = {
    .name = "imu_led_pico",
};

static picoros_publisher_t pub_imu = {
    .topic = {.name = "imu/bno085", .type = ROSTYPE_NAME(ros_Imu), .rihs_hash = ROSTYPE_HASH(ros_Imu)},
    .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
};

static picoros_publisher_t pub_status = {
    .topic = {.name = "imu_led_pico/status", .type = ROSTYPE_NAME(ros_String), .rihs_hash = ROSTYPE_HASH(ros_String)},
    .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
};

static picoros_subscriber_t sub_scan[LIDAR_COUNT];

static uint8_t imu_buf[512];
static uint8_t status_buf[464];

/* The scan callback runs in the zenoh-pico read task, one sample at a time, so
   these buffers are shared by every source. */
static float scan_ranges[MAX_BEAMS];
static float scan_intensities[MAX_BEAMS];
static float scan_scratch[LED_COUNT];
/* What each lidar sees, written by the read task and read by the application
   task, under a critical section. A stamp of 0 means no scan yet. */
static float source_range[LIDAR_COUNT][LED_COUNT];
static uint64_t source_stamp_us[LIDAR_COUNT];
static volatile uint32_t scans_taken;
static volatile uint32_t scans_rejected;
/* The longest scan callback since the last report: the time it holds the read
   task, above the application's. */
static volatile uint32_t scan_cb_max_us;

static led_rgb_t ring[LED_COUNT];
static uint32_t imu_published;

static ros_Time to_ros_time(int64_t ns) {
  ros_Time t = {.sec = (int32_t)(ns / 1000000000ll), .nanosec = (uint32_t)(ns % 1000000000ll)};
  return t;
}

/* Host time of a time_us_64() reading, false until SNTP has answered. */
static bool host_time(uint64_t local_us, int64_t *host) {
  if (!net_time_synced()) {
    return false;
  }
  *host = net_time_ns_at(local_us);
  return true;
}

static void scan_cb(uint8_t *rx, size_t size, const picoros_sample_info_t *info, void *user_data) {
  (void)info;
  uint64_t start_us = time_us_64();
  size_t source = (size_t)(uintptr_t)user_data;
  ros_LaserScan scan = {
      .ranges = {.data = scan_ranges, .n_elements = MAX_BEAMS},
      .intensities = {.data = scan_intensities, .n_elements = MAX_BEAMS},
  };
  if (source >= LIDAR_COUNT || !ps_deserialize(rx, &scan, size)) {
    ++scans_rejected;
    return;
  }
  led_view_clear(scan_scratch);
  led_view_add_scan(scan_scratch, scan.angle_min, scan.angle_increment, scan_ranges, scan.ranges.n_deserialized,
                    scan.range_min, scan.range_max, lidar_sources[source].yaw_rad);
  uint64_t now_us = time_us_64();
  taskENTER_CRITICAL();
  memcpy(source_range[source], scan_scratch, sizeof(scan_scratch));
  source_stamp_us[source] = now_us;
  taskEXIT_CRITICAL();
  ++scans_taken;
  uint32_t took_us = (uint32_t)(time_us_64() - start_us);
  if (took_us > scan_cb_max_us) {
    scan_cb_max_us = took_us;
  }
}

static void publish_imu(void) {
  imu_sample_t s;
  uint64_t read_us = time_us_64();
  if (!imu_poll(&s)) {
    return;
  }
  int64_t stamp;
  if (!host_time(read_us, &stamp)) {
    return;
  }
  ros_Imu imu = {
      .header = {.stamp = to_ros_time(stamp), .frame_id = IMU_FRAME_ID},
  };
  imu.orientation.x = s.orientation_x;
  imu.orientation.y = s.orientation_y;
  imu.orientation.z = s.orientation_z;
  imu.orientation.w = s.orientation_w;
  imu.angular_velocity.x = s.angular_velocity[0];
  imu.angular_velocity.y = s.angular_velocity[1];
  imu.angular_velocity.z = s.angular_velocity[2];
  imu.linear_acceleration.x = s.linear_acceleration[0];
  imu.linear_acceleration.y = s.linear_acceleration[1];
  imu.linear_acceleration.z = s.linear_acceleration[2];
  float orientation_std = s.orientation_accuracy_rad > 0.0f ? s.orientation_accuracy_rad : ORIENTATION_STD_RAD;
  for (int i = 0; i < 3; ++i) {
    imu.orientation_covariance[4 * i] = (double)orientation_std * orientation_std;
    imu.angular_velocity_covariance[4 * i] = (double)GYRO_STD_RAD_S * GYRO_STD_RAD_S;
    imu.linear_acceleration_covariance[4 * i] = (double)ACCEL_STD_M_S2 * ACCEL_STD_M_S2;
  }
  size_t len = ps_serialize(imu_buf, &imu, sizeof(imu_buf));
  if (len > 0 && picoros_publish(&pub_imu, imu_buf, len) == PICOROS_OK) {
    ++imu_published;
  }
}

/* The ring for the scans taken so far. Green only when every lidar is fresh. */
static void update_ring(void) {
  float merged[LED_COUNT];
  float local[LED_COUNT];
  uint64_t now_us = time_us_64();
  led_view_clear(merged);
  bool fresh = true;
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    uint64_t stamp;
    taskENTER_CRITICAL();
    memcpy(local, source_range[i], sizeof(local));
    stamp = source_stamp_us[i];
    taskEXIT_CRITICAL();
    if (stamp == 0 || now_us - stamp > SCAN_TIMEOUT_US) {
      fresh = false;
    } else {
      led_view_merge(merged, local);
    }
  }
  led_view_render(ring, merged, fresh ? LED_VIEW_SCAN : LED_VIEW_NO_SCAN, to_ms_since_boot(get_absolute_time()));
  ws2812_show(ring, BOARD_LED_BRIGHTNESS);
}

static void led_init(void) {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

/* Breathing blue on the ring while waiting for the host. */
static void wait_ms(uint32_t ms) {
  static bool on = false;
  for (uint32_t waited = 0; waited < ms; waited += WAIT_FRAME_MS) {
    on = !on;
    gpio_put(PICO_DEFAULT_LED_PIN, on);
    led_view_render(ring, NULL, LED_VIEW_WAITING_HOST, to_ms_since_boot(get_absolute_time()));
    ws2812_show(ring, BOARD_LED_BRIGHTNESS);
    vTaskDelay(pdMS_TO_TICKS(WAIT_FRAME_MS));
  }
}

static bool declare_all(void) {
  if (picoros_node_init(&node) != PICOROS_OK || picoros_publisher_declare(&node, &pub_imu) != PICOROS_OK ||
      picoros_publisher_declare(&node, &pub_status) != PICOROS_OK) {
    return false;
  }
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    sub_scan[i] = (picoros_subscriber_t){
        .topic = {.name = lidar_sources[i].topic,
                  .type = ROSTYPE_NAME(ros_LaserScan),
                  .rihs_hash = ROSTYPE_HASH(ros_LaserScan)},
        .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
        .info_callback = scan_cb,
        .user_data = (void *)(uintptr_t)i,
    };
    if (picoros_subscriber_declare(&node, &sub_scan[i]) != PICOROS_OK) {
      return false;
    }
  }
  return true;
}

static void report(const char *start) {
  static uint32_t reported_events;
  static uint32_t reported_imu;
  static uint32_t reported_scans;
  static uint32_t reported_frames;
  uint32_t events = imu_event_count();
  uint32_t edges = ws2812_count_edges();
  uint32_t frames = ws2812_frames_sent();
  char text[400];
  snprintf(text, sizeof(text),
           "%s, up %lu s | imu %s (%s), %lu reports, %lu published | sntp %s | %lu scans taken (%lu rejected),"
           " callback max %lu us | strip %lu frames, %lu edges on GP%d, led 0 at %u %u %u of 255 x %d/255"
           " | heap low %lu B",
           start, (unsigned long)(time_us_64() / 1000000u), imu_ready() ? "ok" : "silent", imu_diagnosis(),
           (unsigned long)(events - reported_events), (unsigned long)(imu_published - reported_imu),
           net_time_synced() ? "synced" : "not synced", (unsigned long)(scans_taken - reported_scans),
           (unsigned long)scans_rejected, (unsigned long)scan_cb_max_us, (unsigned long)(frames - reported_frames),
           (unsigned long)edges, BOARD_LED_STRIP_PIN, ring[0].r, ring[0].g, ring[0].b, BOARD_LED_BRIGHTNESS,
           (unsigned long)xPortGetMinimumEverFreeHeapSize());
  scan_cb_max_us = 0;
  reported_frames = frames;
  reported_events = events;
  reported_imu = imu_published;
  reported_scans = scans_taken;
  char *data = text;
  size_t len = ps_serialize(status_buf, &data, sizeof(status_buf));
  if (len > 0) {
    picoros_publish(&pub_status, status_buf, len);
  }
}

static void run_session(const char *start) {
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
  uint32_t tick = 0;
  TickType_t last_wake = xTaskGetTickCount();
  while (picoros_interface_is_up()) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
    watchdog_update();
    publish_imu();
    if (++tick % TICKS_PER_LED_FRAME == 0) {
      update_ring();
    }
    gpio_put(PICO_DEFAULT_LED_PIN, imu_ready());
    if (tick % (REPORT_PERIOD_MS / TICK_MS) == 0) {
      report(start);
    }
  }
}

int app_main(void) {
  led_init();
  ws2812_init(BOARD_LED_STRIP_PIN, BOARD_LED_STRIP_INVERTED);
  /* Any reboot without a power cycle, the watchdog's or picotool's, leaves the
     sensor powered. */
  bool cold_start = !watchdog_caused_reboot();
  const char *start = cold_start                       ? "cold start"
                      : watchdog_enable_caused_reboot() ? "warm start, watchdog timeout"
                                                        : "warm start";
  imu_init(cold_start);

  usb_net_start();
  while (!usb_net_host_reachable()) {
    wait_ms(WAIT_FRAME_MS);
  }

  picoros_interface_t ifx = {
      .mode = ZENOH_CONFIG_MODE,
      .locator = ZENOH_CONFIG_CONNECT,
  };
  if (picoros_interface_init(&ifx) == PICOROS_OK && declare_all()) {
    run_session(start);
  }

  /* Closing a zenoh-pico session and opening another hangs this board, so it
     restarts instead, warm. The pause keeps a missing router from turning this
     into a tight loop, and the watchdog armed by the session still covers it. */
  wait_ms(RETRY_MS);
  watchdog_reboot(0, 0, 0);
  for (;;) {
  }
  return 0;
}
