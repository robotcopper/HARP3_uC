/*
 * micro-ROS (rclc over rcl) with rmw_zenoh_pico on picoros-usb-eth's rp2040
 * platform. It declares the entities of ../picoros_rp2040_basic, so that the
 * same measurements compare the two stacks, and instruments itself:
 *
 *   ros2 topic echo --qos-durability transient_local /pico/boot
 *                       String, once per session, as soon as the node exists:
 *                       how the board last started, uptime, SNTP state
 *   ros2 topic delay /pico/odom
 *                       Odometry at 100 Hz stamped with the SNTP clock: the delay
 *                       is the uplink latency plus the sync error
 *   ros2 topic echo --full-length /pico/stats
 *                       String at 1 Hz: SNTP updates, downlink delay of the joint
 *                       commands, also split at their arrival in rmw_zenoh_pico,
 *                       idle time, system clock, CPU share of each task, XIP
 *                       cache hits and accesses, what rmw_wait did, the time
 *                       zenoh-pico spends finding the subscriptions of each
 *                       sample, heaps, link counters, unused stack of each task
 *                       and how the board last started
 *   topic_based_joint_commands
 *                       JointState, counted; rclc passes no message info to the
 *                       callback, so the downlink delay is taken from
 *                       header.stamp, which the publisher sets with "header: auto"
 *   pico/ping           std_srvs/Trigger, answering at once, so that a call
 *                       times the stack rather than the work
 *   pico/report         std_srvs/Trigger, answering with boot, counters, heaps
 *                       and unused stacks
 *   pico/profiler       std_srvs/SetBool: true samples the program counter from
 *                       a timer interrupt that runs from RAM, so as not to
 *                       disturb the XIP cache it measures; false stops it and
 *                       dumps the samples per 64-byte bucket of flash on
 *                       pico/profile, one String per spin, the last one
 *                       starting with "profile end"
 *
 * A task that the FreeRTOS tick wakes every 10 ms triggers a guard condition, on
 * which the executor publishes the odometry, and the stats every hundredth time.
 * There is no rcl timer: rmw_wait sleeps in whole 1 ms ticks rounded down, so it
 * would wake before a timer is due and spin until it is.
 *
 * The cause of a restart survives it in uninitialised RAM: a panic, which the
 * FreeRTOS stack overflow and heap exhaustion hooks raise, a hard fault with its
 * program counter, a watchdog timeout, or a failure handled here, each with the
 * stage the board had reached. After a panic, a hard fault or a watchdog
 * timeout the board parks: it only brings the USB link up, so that picotool can
 * reboot it into BOOTSEL without a power cycle and read boot_report from RAM.
 *
 * A session that fails to open or is lost restarts the board.
 */

#include <malloc.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait_counts.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>
#include <zenoh-pico/session/subscription_timing.h>

#include "FreeRTOS.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/xip.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"
#include "pico.h"
#include "picoros_usb_eth/net_time.h"
#include "picoros_usb_eth/usb_net.h"
#include "task.h"

/* The mobile base's stepper drivers: ENABLE is active low, and the RP2040's
   reset pull-downs would enable them. */
#define STEPPER_ENABLE_PINS {11, 14, 8}

#define CADENCE_PERIOD_MS 10
#define STATS_EVERY_CADENCE 100u
/* Above the app task, which it wakes, and below the network tasks. */
#define CADENCE_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define CADENCE_TASK_STACK_WORDS 128
/* Only bounds a stall: the cadence wakes the executor every 10 ms. */
#define SPIN_TIMEOUT_MS 100
#define RETRY_MS 1000
#define WAIT_MS 100
/* Armed at start and fed by every wait; never disarmed. */
#define WATCHDOG_TIMEOUT_MS 5000
#define MAX_REPORTED_TASKS 12
#define JOINT_MAX 3
#define JOINT_NAME_CAPACITY 16
#define RUN_RECORD_MAGIC 0x7370696bu
/* Prime, so that sampling does not lock onto the 1 ms tick or the 10 ms timers. */
#define PROFILE_PERIOD_US 251u
#define PROFILE_BUCKET_SHIFT 6u
/* Covers the program, which ends at __flash_binary_end, with room to grow. */
#define PROFILE_SPAN (384u * 1024u)
#define PROFILE_BUCKETS (PROFILE_SPAN >> PROFILE_BUCKET_SHIFT)
#define ROM_END 0x4000u

/* What this run has reached and, once known, why it ends. A watchdog reset
   leaves RAM as it is, so the next boot reads it. */
typedef struct {
  uint32_t magic;
  uint32_t crashed;
  char stage[24];
  char cause[96];
} run_record_t;

static run_record_t __uninitialized_ram(run_record);
static char boot_report[160];
static bool parked;

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_publisher_t boot_pub;
static rcl_publisher_t odom_pub;
static rcl_publisher_t stats_pub;
static rcl_subscription_t joint_sub;
static rcl_service_t ping_srv;
static rcl_service_t trigger_srv;
static rcl_publisher_t profile_pub;
static rcl_service_t profile_srv;
static rcl_guard_condition_t cadence_gc;
/* Written by the cadence task only. */
static volatile uint32_t cadence_count;
static volatile uint32_t cadence_failures;
static uint32_t cadence_handled;

static std_msgs__msg__String boot_msg;
static nav_msgs__msg__Odometry odom_msg;
static std_msgs__msg__String stats_msg;
static sensor_msgs__msg__JointState joint_msg;
static std_srvs__srv__Trigger_Request ping_req;
static std_srvs__srv__Trigger_Response ping_res;
static std_srvs__srv__Trigger_Request trigger_req;
static std_srvs__srv__Trigger_Response trigger_res;
static std_msgs__msg__String profile_msg;
static std_srvs__srv__SetBool_Request profile_req;
static std_srvs__srv__SetBool_Response profile_res;

static char odom_frame_id[] = "odom";
static char odom_child_frame_id[] = "base_footprint";
static char joint_frame_id[32];
static char joint_name_buffers[JOINT_MAX][JOINT_NAME_CAPACITY];
static rosidl_runtime_c__String joint_names[JOINT_MAX];
static double joint_positions[JOINT_MAX];
static double joint_velocities[JOINT_MAX];
static double joint_efforts[JOINT_MAX];

static char boot_text[192];
static char ping_text[] = "";
static char stats_text[1024];
static char trigger_text[512];
static char profile_text[768];
static char profile_res_text[96];

/* Written by the sampling interrupt while profiling, read by the app task once
   it has stopped. */
static uint16_t profile_buckets[PROFILE_BUCKETS];
static volatile bool profiling;
static volatile uint32_t profile_samples;
static volatile uint32_t profile_rom;
static volatile uint32_t profile_ram;
static volatile uint32_t profile_saturated;
static int profile_alarm = -1;
static bool profile_dumping;
static uint32_t profile_next;
static uint32_t profile_chunk;

typedef struct {
  uint32_t n;
  int64_t sum_us;
  int32_t min_us;
  int32_t max_us;
} span_t;

/* Downlink delay of the joint commands over one statistics period, split at
   the sample's arrival in rmw_zenoh_pico: before it, and in the executor. A
   callback that cannot tell which arrival was its own counts as ambiguous.
   Every executor callback runs in the app task, so this needs no lock. */
typedef struct {
  uint32_t count;
  uint32_t timed;
  int64_t sum_us;
  int32_t min_us;
  int32_t max_us;
  span_t before_rmw;
  span_t in_executor;
  uint32_t ambiguous;
} joint_window_t;

static joint_window_t joint_window;
static uint32_t joint_samples_seen;
static uint32_t joint_total;
static uint32_t odom_total;
static uint32_t publish_failures;

static void set_stage(const char *stage) {
  strncpy(run_record.stage, stage, sizeof(run_record.stage) - 1);
}

static void set_cause(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(run_record.cause, sizeof(run_record.cause), fmt, args);
  va_end(args);
}

static void __attribute__((noreturn)) restart(void) {
  watchdog_reboot(0, 0, 0);
  for (;;) {
  }
}

/* Stands for the SDK's panic(), see PICO_PANIC_FUNCTION in CMakeLists.txt. */
void __attribute__((noreturn)) record_panic(const char *fmt, ...) {
  run_record.crashed = 1;
  int len = snprintf(run_record.cause, sizeof(run_record.cause), "panic: ");
  va_list args;
  va_start(args, fmt);
  vsnprintf(run_record.cause + len, sizeof(run_record.cause) - (size_t)len, fmt, args);
  va_end(args);
  restart();
}

static void __attribute__((used, noreturn)) record_hard_fault(const uint32_t *frame) {
  run_record.crashed = 1;
  set_cause("hard fault, pc 0x%08lx, lr 0x%08lx", (unsigned long)frame[6], (unsigned long)frame[5]);
  restart();
}

/* Replaces the SDK's breakpoint. Bit 2 of EXC_RETURN tells which stack holds
   the exception frame. */
void __attribute__((naked)) isr_hardfault(void) {
  __asm volatile(
      "movs r0, #4\n"
      "mov r1, lr\n"
      "tst r0, r1\n"
      "beq 1f\n"
      "mrs r0, psp\n"
      "bl record_hard_fault\n"
      "1:\n"
      "mrs r0, msp\n"
      "bl record_hard_fault\n");
}

static void read_last_run(void) {
  int stage_len = (int)sizeof(run_record.stage) - 1;
  int cause_len = (int)sizeof(run_record.cause) - 1;
  if (!watchdog_caused_reboot()) {
    snprintf(boot_report, sizeof(boot_report), "cold start");
  } else if (run_record.magic != RUN_RECORD_MAGIC) {
    snprintf(boot_report, sizeof(boot_report), "warm start");
  } else if (run_record.cause[0] != '\0') {
    snprintf(boot_report, sizeof(boot_report), "warm start after %.*s, at stage %.*s", cause_len, run_record.cause,
             stage_len, run_record.stage);
    parked = run_record.crashed != 0;
  } else if (watchdog_enable_caused_reboot()) {
    snprintf(boot_report, sizeof(boot_report), "warm start after a watchdog timeout, at stage %.*s", stage_len,
             run_record.stage);
    parked = true;
  } else {
    snprintf(boot_report, sizeof(boot_report), "warm start");
  }
  memset(&run_record, 0, sizeof(run_record));
  run_record.magic = RUN_RECORD_MAGIC;
}

static void disable_steppers(void) {
  static const uint pins[] = STEPPER_ENABLE_PINS;
  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    gpio_init(pins[i]);
    gpio_put(pins[i], 1);
    gpio_set_dir(pins[i], GPIO_OUT);
  }
}

static bool ok(rcl_ret_t ret, const char *what) {
  if (ret == RCL_RET_OK) {
    return true;
  }
  set_cause("%s failed: %d, %s", what, (int)ret, rcl_get_error_string().str);
  printf("%s\n", run_record.cause);
  rcl_reset_error();
  return false;
}

/* snprintf() at buf + len, never past the end of buf. */
static size_t appendf(char *buf, size_t size, size_t len, const char *fmt, ...) {
  if (len >= size - 1) {
    return len;
  }
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(buf + len, size - len, fmt, args);
  va_end(args);
  if (written < 0) {
    return len;
  }
  len += (size_t)written;
  return len < size ? len : size - 1;
}

static void span_add(span_t *span, int32_t us) {
  if (span->n == 0 || us < span->min_us) {
    span->min_us = us;
  }
  if (span->n == 0 || us > span->max_us) {
    span->max_us = us;
  }
  span->sum_us += us;
  ++span->n;
}

static size_t append_span(char *buf, size_t size, size_t len, const char *name, const span_t *span) {
  int32_t avg_us = span->n > 0 ? (int32_t)(span->sum_us / (int64_t)span->n) : 0;
  return appendf(buf, size, len, "%s n=%lu min=%ld avg=%ld max=%ld", name, (unsigned long)span->n,
                 (long)span->min_us, (long)avg_us, (long)span->max_us);
}

static void set_string(rosidl_runtime_c__String *string, char *text, size_t capacity) {
  string->data = text;
  string->size = strlen(text);
  string->capacity = capacity;
}

static void publish(const rcl_publisher_t *pub, const void *msg) {
  if (rcl_publish(pub, msg, NULL) != RCL_RET_OK) {
    ++publish_failures;
    rcl_reset_error();
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

/* zenoh-pico's background task has no name. */
static const char *task_name(const TaskStatus_t *task) {
  return task->pcTaskName[0] != '\0' ? task->pcTaskName : "zenoh";
}

static size_t append_stacks(char *buf, size_t size, size_t len) {
  static TaskStatus_t tasks[MAX_REPORTED_TASKS];
  UBaseType_t count = uxTaskGetSystemState(tasks, MAX_REPORTED_TASKS, NULL);
  len = appendf(buf, size, len, "stack free");
  for (UBaseType_t i = 0; i < count; i++) {
    len = appendf(buf, size, len, " %s=%u", task_name(&tasks[i]),
                  (unsigned)(tasks[i].usStackHighWaterMark * sizeof(StackType_t)));
  }
  return len;
}

/* Share of the period since the last call spent in each task, in percent with
   one decimal. A task is matched to its previous counter by its number. */
static size_t append_cpu(char *buf, size_t size, size_t len) {
  static TaskStatus_t tasks[MAX_REPORTED_TASKS];
  static UBaseType_t last_numbers[MAX_REPORTED_TASKS];
  static configRUN_TIME_COUNTER_TYPE last_counters[MAX_REPORTED_TASKS];
  static UBaseType_t last_count = 0;
  static configRUN_TIME_COUNTER_TYPE last_total = 0;
  configRUN_TIME_COUNTER_TYPE total = 0;
  UBaseType_t count = uxTaskGetSystemState(tasks, MAX_REPORTED_TASKS, &total);
  configRUN_TIME_COUNTER_TYPE period = total - last_total;
  len = appendf(buf, size, len, "cpu");
  for (UBaseType_t i = 0; i < count; i++) {
    configRUN_TIME_COUNTER_TYPE last = 0;
    for (UBaseType_t j = 0; j < last_count; j++) {
      if (last_numbers[j] == tasks[i].xTaskNumber) {
        last = last_counters[j];
        break;
      }
    }
    unsigned long permille = period > 0 ? (unsigned long)((tasks[i].ulRunTimeCounter - last) * 1000u / period) : 0;
    len = appendf(buf, size, len, " %s=%lu.%lu", task_name(&tasks[i]), permille / 10u, permille % 10u);
  }
  for (UBaseType_t i = 0; i < count; i++) {
    last_numbers[i] = tasks[i].xTaskNumber;
    last_counters[i] = tasks[i].ulRunTimeCounter;
  }
  last_count = count;
  last_total = total;
  return len;
}

/* XIP cache hits and accesses since the last call. Both counters saturate and
   clear on any write. */
static size_t append_xip(char *buf, size_t size, size_t len) {
  uint32_t hits = xip_ctrl_hw->ctr_hit;
  uint32_t accesses = xip_ctrl_hw->ctr_acc;
  xip_ctrl_hw->ctr_hit = 0;
  xip_ctrl_hw->ctr_acc = 0;
  uint32_t permille = accesses > 0 ? (uint32_t)((uint64_t)hits * 1000u / accesses) : 1000u;
  return appendf(buf, size, len, "xip hit=%lu.%lu%% acc=%lu", (unsigned long)(permille / 10u),
                 (unsigned long)(permille % 10u), (unsigned long)accesses);
}

/* What rmw_wait did since the last call. Both run in the app task. */
static size_t append_waits(char *buf, size_t size, size_t len) {
  static ZenohPicoWaitCounts last;
  ZenohPicoWaitCounts now = rmw_zenoh_pico_wait_counts;
  size_t written = appendf(
      buf, size, len, "wait calls=%lu ready=%lu zero=%lu short=%lu signalled=%lu timeout=%lu raced=%lu",
      (unsigned long)(now.calls - last.calls), (unsigned long)(now.ready - last.ready),
      (unsigned long)(now.zero_timeout - last.zero_timeout), (unsigned long)(now.short_timeout - last.short_timeout),
      (unsigned long)(now.signalled - last.signalled), (unsigned long)(now.timed_out - last.timed_out),
      (unsigned long)(now.raced - last.raced));
  last = now;
  return written;
}

/* What zenoh-pico spent finding and releasing the subscriptions of each sample
   since the last call. */
static size_t append_lookup(char *buf, size_t size, size_t len) {
  static uint32_t last_count, last_sum_us;
  uint32_t count = _z_subscription_timing.count;
  uint32_t sum_us = _z_subscription_timing.sum_us;
  uint32_t max_us = _z_subscription_timing.max_us;
  _z_subscription_timing.max_us = 0;
  size_t written = appendf(buf, size, len, "lookup n=%lu sum_us=%lu max_us=%lu", (unsigned long)(count - last_count),
                           (unsigned long)(sum_us - last_sum_us), (unsigned long)max_us);
  last_count = count;
  last_sum_us = sum_us;
  return written;
}

static size_t append_heaps(char *buf, size_t size, size_t len) {
  struct mallinfo libc_heap = mallinfo();
  return appendf(buf, size, len, "heap=%lu min=%lu libc=%lu/%lu", (unsigned long)xPortGetFreeHeapSize(),
                 (unsigned long)xPortGetMinimumEverFreeHeapSize(), (unsigned long)libc_heap.uordblks,
                 (unsigned long)libc_heap.arena);
}

static void publish_odom(void) {
  if (!net_time_synced()) {
    return;
  }
  int64_t stamp = net_time_now_ns();
  odom_msg.header.stamp.sec = (int32_t)(stamp / 1000000000ll);
  odom_msg.header.stamp.nanosec = (uint32_t)(stamp % 1000000000ll);
  publish(&odom_pub, &odom_msg);
  ++odom_total;
}

static void publish_stats(void) {
  joint_window_t window = joint_window;
  memset(&joint_window, 0, sizeof(joint_window));

  net_time_stats_t sync;
  net_time_get_stats(&sync);
  usb_net_stats_t link;
  usb_net_get_stats(&link);
  uint32_t age_s = sync.synced ? (uint32_t)((time_us_64() - sync.last_update_us) / 1000000u) : 0;
  int32_t avg_us = window.timed > 0 ? (int32_t)(window.sum_us / (int64_t)window.timed) : 0;

  size_t len = appendf(stats_text, sizeof(stats_text), 0,
                       "sntp synced=%d updates=%lu step_us=%ld age_s=%lu | joint n=%lu dl_us min=%ld avg=%ld max=%ld"
                       " | idle=%lu%% clk_mhz=%lu ",
                       sync.synced ? 1 : 0, (unsigned long)sync.updates, (long)sync.last_step_us,
                       (unsigned long)age_s, (unsigned long)window.count, (long)window.min_us, (long)avg_us,
                       (long)window.max_us, (unsigned long)idle_percent(),
                       (unsigned long)(clock_get_hz(clk_sys) / 1000000u));
  len = appendf(stats_text, sizeof(stats_text), len, "| ");
  len = append_cpu(stats_text, sizeof(stats_text), len);
  len = appendf(stats_text, sizeof(stats_text), len, " | ");
  len = append_xip(stats_text, sizeof(stats_text), len);
  len = appendf(stats_text, sizeof(stats_text), len, " | ");
  len = append_waits(stats_text, sizeof(stats_text), len);
  len = appendf(stats_text, sizeof(stats_text), len, " | ");
  len = append_lookup(stats_text, sizeof(stats_text), len);
  len = appendf(stats_text, sizeof(stats_text), len, " | split ");
  len = append_span(stats_text, sizeof(stats_text), len, "before_rmw_us", &window.before_rmw);
  len = appendf(stats_text, sizeof(stats_text), len, " ");
  len = append_span(stats_text, sizeof(stats_text), len, "in_executor_us", &window.in_executor);
  len = appendf(stats_text, sizeof(stats_text), len, " ambiguous=%lu | ", (unsigned long)window.ambiguous);
  len = append_heaps(stats_text, sizeof(stats_text), len);
  len = appendf(stats_text, sizeof(stats_text), len,
                " | usb rx=%lu drop=%lu defer=%lu tx=%lu drop=%lu | publish failures=%lu cadence failures=%lu | ",
                (unsigned long)link.rx_frames, (unsigned long)link.rx_dropped, (unsigned long)link.rx_deferred,
                (unsigned long)link.tx_frames, (unsigned long)link.tx_dropped, (unsigned long)publish_failures,
                (unsigned long)cadence_failures);
  len = append_stacks(stats_text, sizeof(stats_text), len);
  appendf(stats_text, sizeof(stats_text), len, " | boot: %s", boot_report);
  set_string(&stats_msg.data, stats_text, sizeof(stats_text));
  publish(&stats_pub, &stats_msg);
}

/* Triggers that arrive before the executor takes the guard condition merge into
   one call, so the count tells how many periods this call covers. */
static void cadence_cb(void) {
  uint32_t count = cadence_count;
  publish_odom();
  if (count / STATS_EVERY_CADENCE != cadence_handled / STATS_EVERY_CADENCE) {
    publish_stats();
  }
  cadence_handled = count;
}

static void cadence_task(void *arg) {
  (void)arg;
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CADENCE_PERIOD_MS));
    ++cadence_count;
    if (rcl_trigger_guard_condition(&cadence_gc) != RCL_RET_OK) {
      ++cadence_failures;
    }
  }
}

uint32_t rmw_zenoh_pico_clock_us(void) {
  return time_us_32();
}

uint32_t _z_subscription_clock_us(void) {
  return time_us_32();
}

/* The callback's own arrival is the last one only if exactly one sample came
   since the previous callback and none during the read. */
static void joint_cb(const void *msg) {
  const sensor_msgs__msg__JointState *joint = msg;
  int64_t arrival_ns = net_time_now_ns();
  uint32_t now_us = time_us_32();
  uint32_t samples = rmw_zenoh_pico_samples;
  uint32_t last_sample_us = rmw_zenoh_pico_last_sample_us;
  bool own_arrival = samples - joint_samples_seen == 1u && rmw_zenoh_pico_samples == samples;
  joint_samples_seen = samples;
  int64_t stamp_ns = (int64_t)joint->header.stamp.sec * 1000000000ll + joint->header.stamp.nanosec;
  ++joint_total;
  ++joint_window.count;
  if (!net_time_synced() || stamp_ns <= 0) {
    return;
  }
  int32_t delay_us = (int32_t)((arrival_ns - stamp_ns) / 1000);
  if (joint_window.timed == 0 || delay_us < joint_window.min_us) {
    joint_window.min_us = delay_us;
  }
  if (joint_window.timed == 0 || delay_us > joint_window.max_us) {
    joint_window.max_us = delay_us;
  }
  joint_window.sum_us += delay_us;
  ++joint_window.timed;
  if (own_arrival) {
    int32_t executor_us = (int32_t)(now_us - last_sample_us);
    span_add(&joint_window.in_executor, executor_us);
    span_add(&joint_window.before_rmw, delay_us - executor_us);
  } else {
    ++joint_window.ambiguous;
  }
}

static void ping_cb(const void *request, void *response) {
  (void)request;
  std_srvs__srv__Trigger_Response *res = response;
  res->success = true;
  set_string(&res->message, ping_text, sizeof(ping_text));
}

static void trigger_cb(const void *request, void *response) {
  (void)request;
  std_srvs__srv__Trigger_Response *res = response;
  size_t len = appendf(trigger_text, sizeof(trigger_text), 0, "%s | odom sent=%lu, joint received=%lu | ",
                       boot_report, (unsigned long)odom_total, (unsigned long)joint_total);
  len = append_heaps(trigger_text, sizeof(trigger_text), len);
  len = appendf(trigger_text, sizeof(trigger_text), len, " | ");
  append_stacks(trigger_text, sizeof(trigger_text), len);
  res->success = true;
  set_string(&res->message, trigger_text, sizeof(trigger_text));
}

/* Runs from RAM and touches only RAM and the timer, so that sampling adds no
   XIP cache traffic. It re-arms the alarm only while profiling. */
static void __attribute__((used)) __not_in_flash_func(profile_sample)(const uint32_t *frame) {
  timer_hw->intr = 1u << profile_alarm;
  if (!profiling) {
    return;
  }
  timer_hw->alarm[profile_alarm] = timer_hw->timerawl + PROFILE_PERIOD_US;
  uint32_t pc = frame[6];
  uint32_t offset = pc - XIP_BASE;
  ++profile_samples;
  if (offset < PROFILE_SPAN) {
    uint16_t *bucket = &profile_buckets[offset >> PROFILE_BUCKET_SHIFT];
    if (*bucket != UINT16_MAX) {
      ++*bucket;
    } else {
      ++profile_saturated;
    }
  } else if (pc < ROM_END) {
    ++profile_rom;
  } else {
    ++profile_ram;
  }
}

/* The interrupted context stacked its program counter on the process stack
   when a task was running, on the main stack otherwise; bit 2 of EXC_RETURN
   says which. */
void __attribute__((naked, section(".time_critical.profile_isr"))) profile_isr(void) {
  __asm volatile(
      "movs r0, #4\n"
      "mov r1, lr\n"
      "tst r0, r1\n"
      "beq 1f\n"
      "mrs r0, psp\n"
      "b 2f\n"
      "1:\n"
      "mrs r0, msp\n"
      "2:\n"
      "push {r4, lr}\n"
      "bl profile_sample\n"
      "pop {r4, pc}\n");
}

static void profile_init(void) {
  profile_alarm = hardware_alarm_claim_unused(true);
  uint irq = TIMER_IRQ_0 + (uint)profile_alarm;
  irq_set_exclusive_handler(irq, profile_isr);
  irq_set_priority(irq, PICO_HIGHEST_IRQ_PRIORITY);
  hw_set_bits(&timer_hw->inte, 1u << profile_alarm);
  irq_set_enabled(irq, true);
}

static void profile_cb(const void *request, void *response) {
  const std_srvs__srv__SetBool_Request *req = request;
  std_srvs__srv__SetBool_Response *res = response;
  if (req->data) {
    profiling = false;
    memset(profile_buckets, 0, sizeof(profile_buckets));
    profile_samples = 0;
    profile_rom = 0;
    profile_ram = 0;
    profile_saturated = 0;
    profile_dumping = false;
    profiling = true;
    timer_hw->alarm[profile_alarm] = timer_hw->timerawl + PROFILE_PERIOD_US;
    snprintf(profile_res_text, sizeof(profile_res_text), "sampling every %u us", PROFILE_PERIOD_US);
  } else {
    profiling = false;
    profile_next = 0;
    profile_chunk = 0;
    profile_dumping = true;
    snprintf(profile_res_text, sizeof(profile_res_text), "stopped after %lu samples, dumping on pico/profile",
             (unsigned long)profile_samples);
  }
  res->success = true;
  set_string(&res->message, profile_res_text, sizeof(profile_res_text));
}

/* One chunk per call: "profile <n>" then "<bucket>:<samples>" in hex and
   decimal for each non-empty bucket, and finally the totals. */
static void profile_dump_step(void) {
  if (!profile_dumping) {
    return;
  }
  size_t len;
  if (profile_next >= PROFILE_BUCKETS) {
    len = appendf(profile_text, sizeof(profile_text), 0,
                  "profile end samples=%lu rom=%lu ram=%lu saturated=%lu period_us=%u base=0x%08lx shift=%u",
                  (unsigned long)profile_samples, (unsigned long)profile_rom, (unsigned long)profile_ram,
                  (unsigned long)profile_saturated, PROFILE_PERIOD_US, (unsigned long)XIP_BASE,
                  PROFILE_BUCKET_SHIFT);
    profile_dumping = false;
  } else {
    len = appendf(profile_text, sizeof(profile_text), 0, "profile %lu", (unsigned long)profile_chunk++);
    while (profile_next < PROFILE_BUCKETS && len < sizeof(profile_text) - 16) {
      if (profile_buckets[profile_next] != 0) {
        len = appendf(profile_text, sizeof(profile_text), len, " %lx:%u", (unsigned long)profile_next,
                      (unsigned)profile_buckets[profile_next]);
      }
      ++profile_next;
    }
  }
  (void)len;
  set_string(&profile_msg.data, profile_text, sizeof(profile_text));
  publish(&profile_pub, &profile_msg);
}

static void init_messages(void) {
  set_string(&odom_msg.header.frame_id, odom_frame_id, sizeof(odom_frame_id));
  set_string(&odom_msg.child_frame_id, odom_child_frame_id, sizeof(odom_child_frame_id));
  odom_msg.pose.pose.orientation.w = 1.0;

  joint_msg.header.frame_id = (rosidl_runtime_c__String){.data = joint_frame_id, .capacity = sizeof(joint_frame_id)};
  for (size_t i = 0; i < JOINT_MAX; i++) {
    joint_names[i] = (rosidl_runtime_c__String){.data = joint_name_buffers[i], .capacity = JOINT_NAME_CAPACITY};
  }
  joint_msg.name = (rosidl_runtime_c__String__Sequence){.data = joint_names, .capacity = JOINT_MAX};
  joint_msg.position = (rosidl_runtime_c__double__Sequence){.data = joint_positions, .capacity = JOINT_MAX};
  joint_msg.velocity = (rosidl_runtime_c__double__Sequence){.data = joint_velocities, .capacity = JOINT_MAX};
  joint_msg.effort = (rosidl_runtime_c__double__Sequence){.data = joint_efforts, .capacity = JOINT_MAX};
}

static void publish_boot(void) {
  snprintf(boot_text, sizeof(boot_text), "%s, up %lu ms, sntp %s", boot_report,
           (unsigned long)(time_us_64() / 1000), net_time_synced() ? "synced" : "not synced");
  set_string(&boot_msg.data, boot_text, sizeof(boot_text));
  publish(&boot_pub, &boot_msg);
}

static bool declare_all(rcl_allocator_t *allocator) {
  set_stage("session");
  if (!ok(rclc_support_init(&support, 0, NULL, allocator), "rclc_support_init")) {
    return false;
  }
  printf("session open\n");

  set_stage("node");
  rmw_qos_profile_t latched = rmw_qos_profile_default;
  latched.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  latched.depth = 1;
  latched.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  if (!ok(rclc_node_init_default(&node, "microros_rp2040_basic", "", &support), "node") ||
      !ok(rclc_publisher_init(&boot_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "pico/boot",
                              &latched),
          "pico/boot")) {
    return false;
  }
  publish_boot();

  set_stage("entities");
  if (!ok(rclc_publisher_init_best_effort(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                          "pico/odom"),
          "pico/odom") ||
      !ok(rclc_publisher_init_best_effort(&stats_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                          "pico/stats"),
          "pico/stats") ||
      !ok(rclc_subscription_init_best_effort(&joint_sub, &node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                             "topic_based_joint_commands"),
          "topic_based_joint_commands") ||
      !ok(rclc_service_init_default(&ping_srv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "pico/ping"),
          "pico/ping") ||
      !ok(rclc_service_init_default(&trigger_srv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                    "pico/report"),
          "pico/report") ||
      !ok(rclc_publisher_init_default(&profile_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                      "pico/profile"),
          "pico/profile") ||
      !ok(rclc_service_init_default(&profile_srv, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                    "pico/profiler"),
          "pico/profiler")) {
    return false;
  }

  set_stage("executor");
  return ok(rcl_guard_condition_init(&cadence_gc, &support.context, rcl_guard_condition_get_default_options()),
            "cadence guard condition") &&
         ok(rclc_executor_init(&executor, &support.context, 5, allocator), "executor") &&
         ok(rclc_executor_add_guard_condition(&executor, &cadence_gc, cadence_cb), "add cadence") &&
         ok(rclc_executor_add_subscription(&executor, &joint_sub, &joint_msg, joint_cb, ON_NEW_DATA),
            "add subscription") &&
         ok(rclc_executor_add_service(&executor, &ping_srv, &ping_req, &ping_res, ping_cb), "add ping service") &&
         ok(rclc_executor_add_service(&executor, &trigger_srv, &trigger_req, &trigger_res, trigger_cb),
            "add trigger service") &&
         ok(rclc_executor_add_service(&executor, &profile_srv, &profile_req, &profile_res, profile_cb),
            "add profile service");
}

int app_main(void) {
  disable_steppers();
  read_last_run();
  set_stage("usb");
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
  printf("%s\n", boot_report);

  if (parked) {
    set_stage("parked");
    printf("parked\n");
    usb_net_start();
    for (;;) {
      watchdog_update();
      vTaskDelay(pdMS_TO_TICKS(WAIT_MS));
    }
  }

  usb_net_start();
  printf("usb link up, board %s, host %s\n", USB_NET_DEVICE_IP, USB_NET_HOST_IP);
  set_stage("wait host");
  while (!usb_net_host_reachable()) {
    watchdog_update();
    vTaskDelay(pdMS_TO_TICKS(WAIT_MS));
  }

  init_messages();
  profile_init();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  if (declare_all(&allocator)) {
    printf("node microros_rp2040_basic declared\n");
    if (xTaskCreate(cadence_task, "cadence", CADENCE_TASK_STACK_WORDS, NULL, CADENCE_TASK_PRIORITY, NULL) != pdPASS) {
      set_cause("cadence task not created");
      vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
      restart();
    }
    set_stage("spinning");
    rcl_ret_t ret;
    do {
      watchdog_update();
      ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
      profile_dump_step();
    } while (ret == RCL_RET_OK || ret == RCL_RET_TIMEOUT);
    ok(ret, "rclc_executor_spin_some");
  }

  /* As on the mobile base, restart instead of opening another session. The
     pause keeps a missing router from turning this into a tight loop. */
  vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
  restart();
}
