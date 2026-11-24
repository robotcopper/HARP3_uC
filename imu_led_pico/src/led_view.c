#include "led_view.h"

#include <stddef.h>

#define TWO_PI 6.28318530717958647692f

/* Waiting for the host: blue breathing once every 2 s. */
#define BREATH_PERIOD_MS 2000u
#define BREATH_MIN 20u
#define BREATH_MAX 128u

/* The ends of the gradient in HSV, as a screen shows them: carmine, #960018, near,
   and forest green, #228B22, far. The hue turns through olive and gold, not blue. */
#define NEAR_HUE_DEG (-9.6f)
#define NEAR_SAT 1.0f
#define NEAR_VAL (150.0f / 255.0f)
#define FAR_HUE_DEG 120.0f
#define FAR_SAT (105.0f / 139.0f)
#define FAR_VAL (139.0f / 255.0f)

/* Session open, a lidar silent: dim white, outside the gradient. */
#define NO_SCAN_LEVEL 96u

/* Hue in degrees, saturation and value in [0, 1]. */
static led_rgb_t from_hsv(float hue_deg, float sat, float val) {
  float h = fmodf(hue_deg + 360.0f, 360.0f) / 60.0f;
  float c = val * sat;
  float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  switch ((int)h) {
    case 0:
      r = c, g = x;
      break;
    case 1:
      r = x, g = c;
      break;
    case 2:
      g = c, b = x;
      break;
    case 3:
      g = x, b = c;
      break;
    case 4:
      r = x, b = c;
      break;
    default:
      r = c, b = x;
      break;
  }
  float m = val - c;
  led_rgb_t color = {(uint8_t)lroundf(255.0f * (r + m)), (uint8_t)lroundf(255.0f * (g + m)),
                     (uint8_t)lroundf(255.0f * (b + m))};
  return color;
}

/* Angle in [0, 2 pi). */
static float wrap_2pi(float angle) {
  float a = fmodf(angle, TWO_PI);
  return a < 0.0f ? a + TWO_PI : a;
}

static bool beam_is_valid(float range, float range_min, float range_max) {
  if (!isfinite(range) || range < IGNORE_RANGE_M) {
    return false;
  }
  if (range_min > 0.0f && range < range_min) {
    return false;
  }
  /* A range_max that does not exceed range_min describes no limit. */
  return !(range_max > range_min && range > range_max);
}

void led_view_clear(float *led_range) {
  for (int i = 0; i < LED_COUNT; ++i) {
    led_range[i] = LED_NO_RANGE;
  }
}

/* The beams k_first to k_last of one LED, when the scan runs in k * increment
   from angle_min. Beams outside the scan are skipped. */
static void fold_beams(float *led, long first, long last, const float *ranges, uint32_t count, bool wraps,
                       float range_min, float range_max) {
  for (long k = first; k <= last; ++k) {
    long index = k;
    if (wraps) {
      index = k % (long)count;
      if (index < 0) {
        index += (long)count;
      }
    } else if (k < 0 || k >= (long)count) {
      continue;
    }
    float range = ranges[index];
    if (beam_is_valid(range, range_min, range_max) && range < *led) {
      *led = range;
    }
  }
}

void led_view_add_scan(float *led_range, float angle_min, float angle_increment, const float *ranges, uint32_t count,
                       float range_min, float range_max, float yaw_rad) {
  if (ranges == NULL || count == 0 || !isfinite(angle_min) || !isfinite(yaw_rad) || !isfinite(angle_increment) ||
      !(angle_increment > 0.0f)) {
    return;
  }
  const float inc = angle_increment;
  /* Turns all the way round, to within a beam: the last beam is next to the first. */
  const bool wraps = (float)count * inc >= TWO_PI - 1.5f * inc;
  for (int i = 0; i < LED_COUNT; ++i) {
    /* A beam belongs to a LED when it points within half a beam of the LED's span,
       so that a span narrower than the beam spacing still gets one. */
    float offset = wrap_2pi(LED_SECTORS[i].start_rad - yaw_rad - 0.5f * inc - angle_min);
    float reach = offset + LED_SECTORS[i].width_rad + inc;
    fold_beams(&led_range[i], (long)ceilf(offset / inc), (long)floorf(reach / inc), ranges, count, wraps, range_min,
               range_max);
    if (!wraps) {
      /* The same span, one turn earlier, for a scan that starts before it. */
      fold_beams(&led_range[i], (long)ceilf((offset - TWO_PI) / inc), (long)floorf((reach - TWO_PI) / inc), ranges,
                 count, false, range_min, range_max);
    }
  }
}

void led_view_merge(float *dst, const float *src) {
  for (int i = 0; i < LED_COUNT; ++i) {
    if (src[i] < dst[i]) {
      dst[i] = src[i];
    }
  }
}

led_rgb_t led_view_color(float range_m) {
  /* 0 at IGNORE_RANGE_M and nearer, 1 at FAR_RANGE_M and beyond. */
  float t = (range_m - IGNORE_RANGE_M) / (FAR_RANGE_M - IGNORE_RANGE_M);
  if (!(t < 1.0f)) {
    t = 1.0f;
  } else if (t < 0.0f) {
    t = 0.0f;
  }
  return from_hsv(NEAR_HUE_DEG + t * (FAR_HUE_DEG - NEAR_HUE_DEG), NEAR_SAT + t * (FAR_SAT - NEAR_SAT),
                  NEAR_VAL + t * (FAR_VAL - NEAR_VAL));
}

void led_view_render(led_rgb_t *out, const float *led_range, led_view_mode_t mode, uint32_t time_ms) {
  led_rgb_t flat = {0, 0, 0};
  switch (mode) {
    case LED_VIEW_SCAN:
      for (int i = 0; i < LED_COUNT; ++i) {
        out[i] = led_view_color(led_range[i]);
      }
      return;
    case LED_VIEW_NO_SCAN:
      flat.r = NO_SCAN_LEVEL;
      flat.g = NO_SCAN_LEVEL;
      flat.b = NO_SCAN_LEVEL;
      break;
    case LED_VIEW_WAITING_HOST:
    default: {
      uint32_t phase = time_ms % BREATH_PERIOD_MS;
      uint32_t ramp = phase < BREATH_PERIOD_MS / 2 ? phase : BREATH_PERIOD_MS - phase;
      flat.b = (uint8_t)(BREATH_MIN + ramp * (BREATH_MAX - BREATH_MIN) / (BREATH_PERIOD_MS / 2));
      break;
    }
  }
  for (int i = 0; i < LED_COUNT; ++i) {
    out[i] = flat;
  }
}
