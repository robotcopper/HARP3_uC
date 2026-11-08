/*
 * picoros-usb-eth's rp2040_basic example, Pico-ROS over Ethernet over USB and
 * TCP on the host's clock, with a service added for the comparison with
 * ../microros_rp2040_basic:
 *
 *   ros2 topic echo --qos-durability transient_local /pico/boot
 *                       String, once per session: start, uptime, SNTP state
 *   ros2 topic delay /pico/odom
 *                       Odometry at 100 Hz stamped with the SNTP clock: the delay
 *                       is the uplink latency plus the sync error
 *   ros2 topic echo /pico/stats
 *                       String at 1 Hz: SNTP updates, downlink delay of the joint
 *                       commands, idle time, heap and link counters
 *   topic_based_joint_commands
 *                       JointState, counted; the host's publication time in its
 *                       rmw attachment gives the downlink delay
 *   pico/ping           std_srvs/Trigger, answering at once, so that a call
 *                       times the stack rather than the work
 *
 * A lost session restarts the board.
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "picoros.h"
#include "picoros_usb_eth/net_time.h"
#include "picoros_usb_eth/usb_net.h"
#include "picoserdes.h"
#include "task.h"

#define TICK_MS 10
#define STATS_PERIOD_TICKS 100
#define LED_PERIOD_TICKS 50
#define WAIT_BLINK_MS 100
#define RETRY_MS 1000
/* Armed when the session opens and fed every tick; never disarmed. */
#define WATCHDOG_TIMEOUT_MS 5000
#define JOINT_MAX 3

static void joint_cb(uint8_t *rx, size_t size, const picoros_sample_info_t *info, void *user_data);
static picoros_service_reply_t ping_cb(picoros_srv_server_t *server, uint8_t *rx, size_t size);

static picoros_node_t node = {
    .name = "picoros_rp2040_basic",
};

static picoros_publisher_t pub_boot = {
    .topic = {.name = "pico/boot", .type = ROSTYPE_NAME(ros_String), .rihs_hash = ROSTYPE_HASH(ros_String)},
    .qos = {.reliability = PICOROS_RELIABLE, .durability = PICOROS_TRANSIENT_LOCAL,
            .history = PICOROS_KEEP_LAST, .depth = 1},
};

static picoros_publisher_t pub_odom = {
    .topic = {.name = "pico/odom", .type = ROSTYPE_NAME(ros_Odometry), .rihs_hash = ROSTYPE_HASH(ros_Odometry)},
    .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
};

static picoros_publisher_t pub_stats = {
    .topic = {.name = "pico/stats", .type = ROSTYPE_NAME(ros_String), .rihs_hash = ROSTYPE_HASH(ros_String)},
    .qos = {.reliability = PICOROS_BEST_EFFORT, .depth = 5},
};

static picoros_subscriber_t sub_joint = {
    .topic = {.name = "topic_based_joint_commands",
              .type = ROSTYPE_NAME(ros_JointState),
              .rihs_hash = ROSTYPE_HASH(ros_JointState)},
    .qos = {.reliability = PICOROS_BEST_EFFORT},
    .info_callback = joint_cb,
};

static picoros_srv_server_t srv_ping = {
    .topic = {.name = "pico/ping", .type = ROSTYPE_NAME(srv_Trigger), .rihs_hash = ROSTYPE_HASH(srv_Trigger)},
    .user_callback = ping_cb,
};

static uint8_t odom_buf[1024];
static uint8_t text_buf[320];
static uint8_t srv_buf[32];

/* Downlink delay of the joint commands over one statistics period, written by
   the zenoh-pico read task. */
typedef struct {
  uint32_t count;
  uint32_t bad;
  uint32_t timed;
  int64_t sum_us;
  int32_t min_us;
  int32_t max_us;
} joint_window_t;

static joint_window_t joint_window;
static uint32_t joint_total = 0;

static void joint_cb(uint8_t *rx, size_t size, const picoros_sample_info_t *info, void *user_data) {
  (void)user_data;
  int64_t arrival_ns = net_time_now_ns();
  char *names[JOINT_MAX];
  double position[JOINT_MAX];
  double velocity[JOINT_MAX];
  double effort[JOINT_MAX];
  ros_JointState msg = {
      .name = {.data = names, .n_elements = JOINT_MAX},
      .position = {.data = position, .n_elements = JOINT_MAX},
      .velocity = {.data = velocity, .n_elements = JOINT_MAX},
      .effort = {.data = effort, .n_elements = JOINT_MAX},
  };
  bool ok = ps_deserialize(rx, &msg, size);
  bool timed = net_time_synced() && info->source_timestamp > 0;
  int32_t delay_us = timed ? (int32_t)((arrival_ns - info->source_timestamp) / 1000) : 0;

  taskENTER_CRITICAL();
  ++joint_total;
  if (!ok) {
    ++joint_window.bad;
  }
  ++joint_window.count;
  if (timed) {
    if (joint_window.timed == 0 || delay_us < joint_window.min_us) {
      joint_window.min_us = delay_us;
    }
    if (joint_window.timed == 0 || delay_us > joint_window.max_us) {
      joint_window.max_us = delay_us;
    }
    joint_window.sum_us += delay_us;
    ++joint_window.timed;
  }
  taskEXIT_CRITICAL();
}

static picoros_service_reply_t ping_cb(picoros_srv_server_t *server, uint8_t *rx, size_t size) {
  (void)server;
  (void)rx;
  (void)size;
  reply_srv_Trigger reply = {.success = true, .message = ""};
  size_t len = ps_serialize(srv_buf, &reply, sizeof(srv_buf));
  picoros_service_reply_t response = {.data = srv_buf, .length = len, .free_callback = NULL};
  return response;
}

static void publish_text(picoros_publisher_t *pub, char *text) {
  size_t len = ps_serialize(text_buf, &text, sizeof(text_buf));
  if (len > 0) {
    picoros_publish(pub, text_buf, len);
  }
}

static void publish_odom(void) {
  if (!net_time_synced()) {
    return;
  }
  int64_t stamp = net_time_now_ns();
  ros_Odometry odom = {
      .header = {.stamp = {.sec = (int32_t)(stamp / 1000000000ll), .nanosec = (uint32_t)(stamp % 1000000000ll)},
                 .frame_id = "odom"},
      .child_frame_id = "base_footprint",
  };
  odom.pose.pose.orientation.w = 1.0;
  size_t len = ps_serialize(odom_buf, &odom, sizeof(odom_buf));
  if (len > 0) {
    picoros_publish(&pub_odom, odom_buf, len);
  }
}

/* Share of the last period spent in the idle task. */
static uint32_t idle_percent(void) {
  static uint64_t last_idle = 0;
  static uint64_t last_total = 0;
  uint64_t idle = ulTaskGetIdleRunTimeCounter();
  uint64_t total = portGET_RUN_TIME_COUNTER_VALUE();
  uint32_t percent = total > last_total ? (uint32_t)((idle - last_idle) * 100u / (total - last_total)) : 0;
  last_idle = idle;
  last_total = total;
  return percent;
}

static void publish_stats(void) {
  joint_window_t window;
  taskENTER_CRITICAL();
  window = joint_window;
  memset(&joint_window, 0, sizeof(joint_window));
  taskEXIT_CRITICAL();

  net_time_stats_t sync;
  net_time_get_stats(&sync);
  usb_net_stats_t link;
  usb_net_get_stats(&link);
  uint32_t age_s = sync.synced ? (uint32_t)((time_us_64() - sync.last_update_us) / 1000000u) : 0;
  int32_t avg_us = window.timed > 0 ? (int32_t)(window.sum_us / (int64_t)window.timed) : 0;

  char text[288];
  snprintf(text, sizeof(text),
           "sntp synced=%d updates=%lu step_us=%ld age_s=%lu | joint n=%lu bad=%lu dl_us min=%ld avg=%ld max=%ld"
           " | idle=%lu%% heap=%lu min=%lu | usb rx=%lu drop=%lu defer=%lu tx=%lu drop=%lu",
           sync.synced ? 1 : 0, (unsigned long)sync.updates, (long)sync.last_step_us, (unsigned long)age_s,
           (unsigned long)window.count, (unsigned long)window.bad, (long)window.min_us, (long)avg_us,
           (long)window.max_us, (unsigned long)idle_percent(), (unsigned long)xPortGetFreeHeapSize(),
           (unsigned long)xPortGetMinimumEverFreeHeapSize(), (unsigned long)link.rx_frames,
           (unsigned long)link.rx_dropped, (unsigned long)link.rx_deferred, (unsigned long)link.tx_frames,
           (unsigned long)link.tx_dropped);
  publish_text(&pub_stats, text);
}

static void led_init(void) {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static void led_toggle(void) {
  static bool on = false;
  on = !on;
  gpio_put(PICO_DEFAULT_LED_PIN, on);
}

/* Fast blinking while waiting. */
static void wait_ms(uint32_t ms) {
  for (uint32_t waited = 0; waited < ms; waited += WAIT_BLINK_MS) {
    led_toggle();
    vTaskDelay(pdMS_TO_TICKS(WAIT_BLINK_MS));
  }
}

static bool declare_all(void) {
  return picoros_node_init(&node) == PICOROS_OK && picoros_publisher_declare(&node, &pub_boot) == PICOROS_OK &&
         picoros_publisher_declare(&node, &pub_odom) == PICOROS_OK &&
         picoros_publisher_declare(&node, &pub_stats) == PICOROS_OK &&
         picoros_subscriber_declare(&node, &sub_joint) == PICOROS_OK &&
         picoros_service_declare(&node, &srv_ping) == PICOROS_OK;
}

static void run_session(const char *start) {
  char text[128];
  snprintf(text, sizeof(text), "%s, up %lu ms, sntp %s", start, (unsigned long)(time_us_64() / 1000),
           net_time_synced() ? "synced" : "not synced");
  publish_text(&pub_boot, text);

  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
  uint32_t tick = 0;
  TickType_t last_wake = xTaskGetTickCount();
  while (picoros_interface_is_up()) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
    watchdog_update();
    ++tick;
    publish_odom();
    if (tick % STATS_PERIOD_TICKS == 0) {
      publish_stats();
    }
    if (tick % LED_PERIOD_TICKS == 0) {
      led_toggle();
    }
  }
}

int app_main(void) {
  led_init();
  const char *start = !watchdog_caused_reboot()        ? "cold start"
                      : watchdog_enable_caused_reboot() ? "warm start, watchdog timeout"
                                                        : "warm start";
  printf("%s\n", start);

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
    run_session(start);
    printf("session lost\n");
  }

  /* Closing a zenoh-pico session and opening another hangs this board, so it
     restarts instead. The pause keeps a missing router from turning this into a
     tight loop, and the watchdog armed by the session still covers it. */
  wait_ms(RETRY_MS);
  watchdog_reboot(0, 0, 0);
  for (;;) {
  }
  return 0;
}
