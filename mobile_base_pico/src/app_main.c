/*
 * HARP3 mobile base node, on Pico-ROS over Ethernet over USB and TCP to
 * rmw_zenohd.
 *
 *   topic_based_joint_commands  sensor_msgs/JointState, subscribed, best effort.
 *                               Wheel speeds in rad/s from
 *                               joint_state_topic_hardware_interface.
 *   odom/paa5160e1              nav_msgs/Odometry, 100 Hz, best effort, stamped
 *                               with the host clock at the sensor read.
 *   limit_switches              harp3_msgs/LimitSwitches, on change, reliable,
 *                               transient local.
 *   reset_odometry              std_srvs/Trigger, back to the origin.
 *
 * The host clock comes over SNTP from the computer the board is plugged into,
 * and odometry and limit switches wait for its first answer. The drivers are
 * enabled only while a session is open, and a lost session restarts the board.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "hardware/watchdog.h"
#include "motion.h"
#include "otos.h"
#include "pico/stdlib.h"
#include "picoros.h"
#include "picoros_usb_eth/net_time.h"
#include "picoros_usb_eth/usb_net.h"
#include "picoserdes.h"
#include "semphr.h"
#include "steppers.h"
#include "task.h"

#define TICK_MS 10
#define COMMAND_TIMEOUT_MS 200
#define RETRY_MS 1000
#define WAIT_BLINK_MS 100
/* Armed when the session opens and fed every tick; never disarmed. */
#define WATCHDOG_TIMEOUT_MS 5000

#define ODOM_FRAME_ID "odom_paa5160"
#define ODOM_CHILD_FRAME_ID "paa5160_frame"
#define SENSOR_HEIGHT_M 0.019

static void joint_cb(uint8_t *rx, size_t size);
static picoros_service_reply_t reset_cb(picoros_srv_server_t *server, uint8_t *rx, size_t size);

static picoros_node_t node = {
    .name = "mobile_base_pico",
};

static picoros_publisher_t pub_odom = {
    .topic = {.name = "odom/paa5160e1", .type = ROSTYPE_NAME(ros_Odometry), .rihs_hash = ROSTYPE_HASH(ros_Odometry)},
    .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
};

static picoros_publisher_t pub_switches = {
    .topic = {.name = "limit_switches",
              .type = ROSTYPE_NAME(ros_LimitSwitches),
              .rihs_hash = ROSTYPE_HASH(ros_LimitSwitches)},
    .qos = {.reliability = PICOROS_RELIABLE, .durability = PICOROS_TRANSIENT_LOCAL,
            .history = PICOROS_KEEP_LAST, .depth = 1},
};

static picoros_subscriber_t sub_joint = {
    .topic = {.name = "topic_based_joint_commands",
              .type = ROSTYPE_NAME(ros_JointState),
              .rihs_hash = ROSTYPE_HASH(ros_JointState)},
    .qos = {.reliability = PICOROS_BEST_EFFORT},
    .user_callback = joint_cb,
};

static picoros_srv_server_t srv_reset = {
    .topic = {.name = "reset_odometry", .type = ROSTYPE_NAME(srv_Trigger), .rihs_hash = ROSTYPE_HASH(srv_Trigger)},
    .user_callback = reset_cb,
};

static uint8_t odom_buf[1024];
static uint8_t switches_buf[96];
static uint8_t srv_buf[96];

static const uint switch_pins[BOARD_LIMIT_SWITCH_COUNT] = BOARD_LIMIT_SWITCH_PINS;
static uint8_t switch_state;
static uint64_t switch_change_us;
static bool switch_pending;

/* Shared between the zenoh-pico read task and the application task. */
static SemaphoreHandle_t otos_mutex;
static bool otos_ok = false;
static volatile uint32_t rejected_commands = 0;

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

static void joint_cb(uint8_t *rx, size_t size) {
  char *names[WHEEL_COUNT];
  double position[WHEEL_COUNT];
  double velocity[WHEEL_COUNT];
  double effort[WHEEL_COUNT];
  ros_JointState msg = {
      .name = {.data = names, .n_elements = WHEEL_COUNT},
      .position = {.data = position, .n_elements = WHEEL_COUNT},
      .velocity = {.data = velocity, .n_elements = WHEEL_COUNT},
      .effort = {.data = effort, .n_elements = WHEEL_COUNT},
  };
  double wheel[WHEEL_COUNT];
  bool ok = ps_deserialize(rx, &msg, size) &&
            wheel_command(names, msg.name.n_deserialized, velocity, msg.velocity.n_deserialized, wheel);
  if (!ok || !steppers_command(wheel)) {
    ++rejected_commands;
  }
}

static picoros_service_reply_t reset_cb(picoros_srv_server_t *server, uint8_t *rx, size_t size) {
  (void)server;
  (void)rx;
  (void)size;
  reply_srv_Trigger reply = {.success = false, .message = "odometry sensor not detected"};
  if (otos_ok) {
    xSemaphoreTake(otos_mutex, portMAX_DELAY);
    reply.success = otos_reset_tracking();
    xSemaphoreGive(otos_mutex);
    reply.message = reply.success ? "" : "odometry sensor did not acknowledge";
  }
  size_t len = ps_serialize(srv_buf, &reply, sizeof(srv_buf));
  picoros_service_reply_t response = {.data = srv_buf, .length = len, .free_callback = NULL};
  return response;
}

static void publish_odometry(void) {
  if (!otos_ok) {
    return;
  }
  otos_sample_t s;
  uint64_t read_us = time_us_64();
  xSemaphoreTake(otos_mutex, portMAX_DELAY);
  bool read = otos_read(&s);
  xSemaphoreGive(otos_mutex);
  int64_t stamp;
  if (!read || !host_time(read_us, &stamp)) {
    return;
  }
  ros_Odometry odom = {
      .header = {.stamp = to_ros_time(stamp), .frame_id = ODOM_FRAME_ID},
      .child_frame_id = ODOM_CHILD_FRAME_ID,
  };
  odom.pose.pose.position.x = s.position.x;
  odom.pose.pose.position.y = s.position.y;
  odom.pose.pose.position.z = SENSOR_HEIGHT_M;
  odom.pose.pose.orientation.z = sin(s.position.h / 2.0);
  odom.pose.pose.orientation.w = cos(s.position.h / 2.0);
  odom.pose.covariance[0] = (double)s.position_std_dev.x * s.position_std_dev.x;
  odom.pose.covariance[7] = (double)s.position_std_dev.y * s.position_std_dev.y;
  odom.pose.covariance[35] = (double)s.position_std_dev.h * s.position_std_dev.h;
  odom.twist.twist.linear.x = s.velocity.x;
  odom.twist.twist.linear.y = s.velocity.y;
  odom.twist.twist.angular.z = s.velocity.h;
  odom.twist.covariance[0] = (double)s.velocity_std_dev.x * s.velocity_std_dev.x;
  odom.twist.covariance[7] = (double)s.velocity_std_dev.y * s.velocity_std_dev.y;
  odom.twist.covariance[35] = (double)s.velocity_std_dev.h * s.velocity_std_dev.h;
  size_t len = ps_serialize(odom_buf, &odom, sizeof(odom_buf));
  if (len > 0) {
    picoros_publish(&pub_odom, odom_buf, len);
  }
}

static uint8_t read_switches(void) {
  uint8_t state = 0;
  for (int i = 0; i < BOARD_LIMIT_SWITCH_COUNT; ++i) {
    state |= (uint8_t)(gpio_get(switch_pins[i]) ? 1u << i : 0u);
  }
  return state;
}

/* On every change, stamped at the read that saw it. */
static void publish_limit_switches(void) {
  uint64_t now_us = time_us_64();
  uint8_t state = read_switches();
  if (state != switch_state) {
    switch_state = state;
    switch_change_us = now_us;
    switch_pending = true;
  }
  int64_t stamp;
  if (!switch_pending || !host_time(switch_change_us, &stamp)) {
    return;
  }
  ros_LimitSwitches msg = {
      .header = {.stamp = to_ros_time(stamp), .frame_id = ""},
      .switch_11 = (state & 0x01u) != 0,
      .switch_12 = (state & 0x02u) != 0,
      .switch_21 = (state & 0x04u) != 0,
      .switch_22 = (state & 0x08u) != 0,
      .switch_31 = (state & 0x10u) != 0,
      .switch_32 = (state & 0x20u) != 0,
  };
  size_t len = ps_serialize(switches_buf, &msg, sizeof(switches_buf));
  if (len > 0 && picoros_publish(&pub_switches, switches_buf, len) == PICOROS_OK) {
    switch_pending = false;
  }
}

static void switches_init(void) {
  for (int i = 0; i < BOARD_LIMIT_SWITCH_COUNT; ++i) {
    gpio_init(switch_pins[i]);
    gpio_set_dir(switch_pins[i], GPIO_IN);
    gpio_pull_down(switch_pins[i]);
  }
}

static void led_init(void) {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

/* Fast blinking while waiting for the host. */
static void wait_ms(uint32_t ms) {
  static bool on = false;
  for (uint32_t waited = 0; waited < ms; waited += WAIT_BLINK_MS) {
    on = !on;
    gpio_put(PICO_DEFAULT_LED_PIN, on);
    vTaskDelay(pdMS_TO_TICKS(WAIT_BLINK_MS));
  }
}

static bool declare_all(void) {
  return picoros_node_init(&node) == PICOROS_OK && picoros_publisher_declare(&node, &pub_odom) == PICOROS_OK &&
         picoros_publisher_declare(&node, &pub_switches) == PICOROS_OK &&
         picoros_subscriber_declare(&node, &sub_joint) == PICOROS_OK &&
         picoros_service_declare(&node, &srv_reset) == PICOROS_OK;
}

static void run_session(void) {
  switch_state = read_switches();
  switch_change_us = time_us_64();
  switch_pending = true;
  steppers_enable(true);
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

  TickType_t last_wake = xTaskGetTickCount();
  while (picoros_interface_is_up()) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
    watchdog_update();
    publish_odometry();
    publish_limit_switches();
    gpio_put(PICO_DEFAULT_LED_PIN, steppers_running());
  }
  steppers_enable(false);
}

int app_main(void) {
  steppers_init(COMMAND_TIMEOUT_MS);
  switches_init();
  led_init();
  /* Any reboot without a power cycle, the watchdog's or picotool's, leaves the
     sensor powered and tracking. */
  bool cold_start = !watchdog_caused_reboot();
  const char *start = cold_start                       ? "cold start"
                      : watchdog_enable_caused_reboot() ? "warm start, watchdog timeout"
                                                        : "warm start";
  printf("%s\n", start);
  otos_mutex = xSemaphoreCreateMutex();
  otos_ok = otos_init(cold_start);

  usb_net_start();
  printf("usb link up, board %s, host %s\n", USB_NET_DEVICE_IP, USB_NET_HOST_IP);
  while (!usb_net_host_reachable()) {
    wait_ms(WAIT_BLINK_MS);
  }

  picoros_interface_t ifx = {
      .mode = ZENOH_CONFIG_MODE,
      .locator = ZENOH_CONFIG_CONNECT,
  };
  printf("opening %s session on %s\n", ifx.mode, ifx.locator);
  if (picoros_interface_init(&ifx) != PICOROS_OK) {
    printf("no session\n");
  } else if (!declare_all()) {
    printf("declaration failed\n");
  } else {
    printf("node %s declared\n", node.name);
    run_session();
    printf("session lost\n");
  }

  /* Closing a zenoh-pico session and opening another hangs this board, so it
     restarts instead, warm: the odometry sensor keeps its pose. The pause keeps
     a missing router from turning this into a tight loop, and the watchdog
     armed by the session still covers it. */
  steppers_enable(false);
  wait_ms(RETRY_MS);
  watchdog_reboot(0, 0, 0);
  for (;;) {
  }
  return 0;
}
