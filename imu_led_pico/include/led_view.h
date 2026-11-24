/*
 * What the obstacle ring shows, free of hardware access so that it builds and is
 * tested on the host: LaserScan beams to a range per LED, and a range to a colour.
 */

#ifndef IMU_LED_LED_VIEW_H
#define IMU_LED_LED_VIEW_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "led_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Obstacle distance from the lidar, in metres, at the far end of the gradient: from
   there, and with no obstacle in sight, a LED is forest green. */
#define FAR_RANGE_M 0.60f

/* Beams nearer than this, in metres from the lidar, are ignored, whatever the scan's
   range_min: within it the lidar sees the robot's own structure. */
#define IGNORE_RANGE_M 0.14f

/* Range of a LED no beam of the scan measured: no obstacle in sight. */
#define LED_NO_RANGE INFINITY

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_rgb_t;

typedef enum {
  /* No session with the host yet. */
  LED_VIEW_WAITING_HOST,
  /* Session open, but a lidar is silent: nothing is known, so the ring is dim white,
     outside the gradient. */
  LED_VIEW_NO_SCAN,
  /* Every lidar answered recently. */
  LED_VIEW_SCAN,
} led_view_mode_t;

/* One lidar's view: the nearest valid measurement in front of each LED. */
void led_view_clear(float *led_range);

/* Folds a LaserScan into led_range, keeping the nearer of what it holds and the
   new measurements. yaw_rad is the lidar's rotation in base_link, 0 when its
   frame is base_link's. A beam is valid when finite, no nearer than range_min nor
   IGNORE_RANGE_M and no farther than range_max: the Tmini Plus reports 0 for a
   missed point, and
   infinity beyond its range. A scan that does not describe a direction, or that
   has no beam, changes nothing. A scan that does not turn all the way round
   leaves the LEDs it does not reach at LED_NO_RANGE. */
void led_view_add_scan(float *led_range, float angle_min, float angle_increment, const float *ranges, uint32_t count,
                       float range_min, float range_max, float yaw_rad);

/* Keeps in dst the nearer of dst and src, LED by LED: several lidars, one ring. */
void led_view_merge(float *dst, const float *src);

/* The colour of one LED for its range: carmine at IGNORE_RANGE_M, through olive
   and gold, to forest green at FAR_RANGE_M, interpolated in HSV. A screen's
   colours: the strip driver corrects them and applies the ceiling. */
led_rgb_t led_view_color(float range_m);

/* The whole ring. time_ms drives the animations of the two waiting modes. */
void led_view_render(led_rgb_t *out, const float *led_range, led_view_mode_t mode, uint32_t time_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LED_LED_VIEW_H */
