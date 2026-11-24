#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "led_view.h"

static int failures;

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      printf("%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond);     \
      ++failures;                                                         \
    }                                                                     \
  } while (0)

#define PI_F 3.14159265358979f
#define BEAMS 500
#define MAX_BEAMS 2048

static float ranges[MAX_BEAMS];

/* The YDLIDAR driver's scan: -pi to pi, a beam every 2 pi / BEAMS. */
static void full_scan(float value) {
  for (int i = 0; i < BEAMS; ++i) {
    ranges[i] = value;
  }
}

static float full_increment(void) { return 2.0f * PI_F / BEAMS; }

static float wrap_pi(float a) { return atan2f(sinf(a), cosf(a)); }

/* The LED whose span holds a direction of base_link. */
static int led_at(float angle) {
  for (int i = 0; i < LED_COUNT; ++i) {
    float into = wrap_pi(angle - LED_SECTORS[i].start_rad);
    if (into >= 0.0f && into < LED_SECTORS[i].width_rad) {
      return i;
    }
  }
  return -1;
}

static int count_below(const float *led, float limit) {
  int n = 0;
  for (int i = 0; i < LED_COUNT; ++i) {
    if (led[i] < limit) {
      ++n;
    }
  }
  return n;
}

static void test_map_tiles_the_circle(void) {
  float total = 0.0f;
  for (int i = 0; i < LED_COUNT; ++i) {
    CHECK(LED_SECTORS[i].width_rad > 0.0f);
    CHECK(LED_SECTORS[i].radius_m > 0.05f && LED_SECTORS[i].radius_m < 0.2f);
    total += LED_SECTORS[i].width_rad;
    /* Each span begins where the previous one ends. */
    const led_sector_t *next = &LED_SECTORS[(i + 1) % LED_COUNT];
    float gap = wrap_pi(next->start_rad - (LED_SECTORS[i].start_rad + LED_SECTORS[i].width_rad));
    CHECK(fabsf(gap) < 1e-4f);
  }
  CHECK(fabsf(total - 2.0f * PI_F) < 1e-3f);
  /* The strip starts in the notch at -60 degrees and the first LED runs towards +x. */
  CHECK(fabsf(LED_SECTORS[0].start_rad - (-PI_F / 3.0f)) < 1e-3f);
  CHECK(led_at(0.0f) > 0 && led_at(0.0f) < LED_COUNT / 2);
  for (int i = 0; i < 360; ++i) {
    CHECK(led_at((float)i * PI_F / 180.0f) >= 0);
  }
}

static void test_obstacle_lights_its_led(void) {
  float led[LED_COUNT];
  for (int deg = -170; deg <= 170; deg += 10) {
    led_view_clear(led);
    full_scan(5.0f);
    int beam = (int)lroundf(((float)deg * PI_F / 180.0f + PI_F) / full_increment());
    ranges[beam] = 0.15f;
    led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
    int expected = led_at(-PI_F + (float)beam * full_increment());
    CHECK(led[expected] < OBSTACLE_RANGE_M);
    /* One beam is under a degree: at most the LED and a neighbour or two. */
    CHECK(count_below(led, OBSTACLE_RANGE_M) >= 1 && count_below(led, OBSTACLE_RANGE_M) <= 3);
    CHECK(led_view_color(led[expected]).r == 255 && led_view_color(led[expected]).g == 0);
    int far = (expected + LED_COUNT / 2) % LED_COUNT;
    CHECK(led_view_color(led[far]).g == 255 && led_view_color(led[far]).r == 0);
  }
}

static void test_invalid_beams_show_nothing(void) {
  float led[LED_COUNT];
  /* The Tmini Plus reports 0 for a missed point. */
  led_view_clear(led);
  full_scan(0.0f);
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);

  full_scan(INFINITY);
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);

  full_scan(NAN);
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);

  full_scan(0.05f); /* nearer than the lidar can measure */
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);

  full_scan(20.0f); /* beyond its range */
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);

  /* A range_max that describes no limit does not discard everything. */
  full_scan(3.0f);
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.0f, 0.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == LED_COUNT);
}

static void test_yaw_turns_the_scan(void) {
  float led[LED_COUNT];
  led_view_clear(led);
  full_scan(5.0f);
  int beam = BEAMS / 2; /* angle 0 in the lidar's frame */
  ranges[beam] = 0.1f;
  led_view_add_scan(led, -PI_F, full_increment(), ranges, BEAMS, 0.08f, 12.0f, PI_F / 2.0f);
  CHECK(led[led_at(PI_F / 2.0f)] < OBSTACLE_RANGE_M);
  CHECK(led[led_at(0.0f)] > OBSTACLE_RANGE_M);
}

static void test_sparse_scan_reaches_every_led(void) {
  float led[LED_COUNT];
  led_view_clear(led);
  /* 10 degrees between beams, wider than most LEDs. */
  const int n = 36;
  for (int i = 0; i < n; ++i) {
    ranges[i] = 2.0f;
  }
  led_view_add_scan(led, -PI_F, 2.0f * PI_F / n, ranges, n, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == LED_COUNT);
}

static void test_seam_of_a_scan_from_zero(void) {
  float led[LED_COUNT];
  /* 0 to 2 pi: the LEDs behind the robot sit on the seam. */
  led_view_clear(led);
  full_scan(5.0f);
  ranges[0] = 0.1f;
  led_view_add_scan(led, 0.0f, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(led[led_at(0.0f)] < OBSTACLE_RANGE_M);

  led_view_clear(led);
  full_scan(5.0f);
  ranges[BEAMS - 1] = 0.1f;
  led_view_add_scan(led, 0.0f, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(led[led_at(-full_increment())] < OBSTACLE_RANGE_M);

  /* The same, across the +-pi seam of the robot. */
  led_view_clear(led);
  full_scan(5.0f);
  ranges[BEAMS / 2] = 0.1f; /* angle pi */
  led_view_add_scan(led, 0.0f, full_increment(), ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(led[led_at(PI_F - 0.001f)] < OBSTACLE_RANGE_M || led[led_at(-PI_F + 0.001f)] < OBSTACLE_RANGE_M);
}

static void test_partial_scan(void) {
  float led[LED_COUNT];
  led_view_clear(led);
  /* The front half only, -90 to 90 degrees. */
  const int n = 250;
  for (int i = 0; i < n; ++i) {
    ranges[i] = 1.0f;
  }
  ranges[0] = 0.1f; /* -90 degrees */
  led_view_add_scan(led, -PI_F / 2.0f, PI_F / n, ranges, n, 0.08f, 12.0f, 0.0f);
  CHECK(led[led_at(0.0f)] == 1.0f);
  CHECK(led[led_at(PI_F)] == LED_NO_RANGE);
  CHECK(led[led_at(-PI_F / 2.0f + 0.001f)] < OBSTACLE_RANGE_M);
  /* A partial scan whose angle_min is above pi in the robot's frame, by yaw. */
  led_view_clear(led);
  led_view_add_scan(led, -PI_F / 2.0f, PI_F / n, ranges, n, 0.08f, 12.0f, PI_F);
  CHECK(led[led_at(PI_F)] == 1.0f);
  CHECK(led[led_at(0.0f)] == LED_NO_RANGE);
  CHECK(led[led_at(PI_F / 2.0f + 0.001f)] < OBSTACLE_RANGE_M);
}

static void test_bad_scans_change_nothing(void) {
  float led[LED_COUNT];
  led_view_clear(led);
  full_scan(0.1f);
  led_view_add_scan(led, -PI_F, 0.0f, ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  led_view_add_scan(led, -PI_F, -0.01f, ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  led_view_add_scan(led, -PI_F, NAN, ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  led_view_add_scan(led, NAN, 0.01f, ranges, BEAMS, 0.08f, 12.0f, 0.0f);
  led_view_add_scan(led, -PI_F, 0.01f, ranges, 0, 0.08f, 12.0f, 0.0f);
  led_view_add_scan(led, -PI_F, 0.01f, NULL, BEAMS, 0.08f, 12.0f, 0.0f);
  CHECK(count_below(led, INFINITY) == 0);
}

static void test_keeps_the_nearer(void) {
  float a[LED_COUNT], b[LED_COUNT];
  led_view_clear(a);
  led_view_clear(b);
  a[3] = 0.5f;
  b[3] = 0.3f;
  a[4] = 0.1f;
  b[5] = 0.7f;
  led_view_merge(a, b);
  CHECK(a[3] == 0.3f && a[4] == 0.1f && a[5] == 0.7f && a[6] == LED_NO_RANGE);
}

static void test_color(void) {
  CHECK(led_view_color(0.0f).r == 255 && led_view_color(0.0f).g == 0);
  CHECK(led_view_color(0.1999f).r == 255);
  CHECK(led_view_color(OBSTACLE_RANGE_M).g == 255 && led_view_color(OBSTACLE_RANGE_M).r == 0);
  CHECK(led_view_color(LED_NO_RANGE).g == 255);
  CHECK(led_view_color(LED_NO_RANGE).b == 0);
}

static void test_render(void) {
  led_rgb_t out[LED_COUNT];
  float led[LED_COUNT];
  led_view_clear(led);
  led[10] = 0.1f;

  led_view_render(out, led, LED_VIEW_SCAN, 0);
  CHECK(out[10].r == 255 && out[10].g == 0);
  CHECK(out[11].g == 255 && out[11].r == 0);

  /* Without a scan nothing is green, whatever the ranges hold. */
  led_view_clear(led);
  led_view_render(out, led, LED_VIEW_NO_SCAN, 0);
  for (int i = 0; i < LED_COUNT; ++i) {
    CHECK(out[i].r > 0 && out[i].g < out[i].r && out[i].b == 0);
  }

  for (uint32_t t = 0; t < 4000; t += 50) {
    led_view_render(out, led, LED_VIEW_WAITING_HOST, t);
    CHECK(out[0].r == 0 && out[0].g == 0 && out[0].b >= 20 && out[0].b <= 128);
    CHECK(out[0].b == out[LED_COUNT - 1].b);
  }
  led_view_render(out, led, LED_VIEW_WAITING_HOST, 0);
  uint8_t dark = out[0].b;
  led_view_render(out, led, LED_VIEW_WAITING_HOST, 1000);
  CHECK(out[0].b > dark);
}

int main(void) {
  test_map_tiles_the_circle();
  test_obstacle_lights_its_led();
  test_invalid_beams_show_nothing();
  test_yaw_turns_the_scan();
  test_sparse_scan_reaches_every_led();
  test_seam_of_a_scan_from_zero();
  test_partial_scan();
  test_bad_scans_change_nothing();
  test_keeps_the_nearer();
  test_color();
  test_render();
  if (failures != 0) {
    printf("%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  printf("led_view: all checks passed\n");
  return EXIT_SUCCESS;
}
