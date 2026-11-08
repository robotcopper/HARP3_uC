/*
 * clock_gettime() for rcutils and rmw_zenoh_pico, which newlib leaves to the
 * platform. CLOCK_REALTIME is the wall clock that SNTP sets through
 * settimeofday(), CLOCK_MONOTONIC the 1 MHz hardware timer.
 */

#include <errno.h>
#include <sys/time.h>
#include <time.h>

#include "pico/time.h"

int clock_gettime(clockid_t clock_id, struct timespec *tp) {
  if (clock_id == CLOCK_REALTIME) {
    struct timeval now;
    gettimeofday(&now, NULL);
    tp->tv_sec = now.tv_sec;
    tp->tv_nsec = now.tv_usec * 1000L;
    return 0;
  }
  if (clock_id == CLOCK_MONOTONIC) {
    uint64_t us = time_us_64();
    tp->tv_sec = (time_t)(us / 1000000u);
    tp->tv_nsec = (long)(us % 1000000u) * 1000L;
    return 0;
  }
  errno = EINVAL;
  return -1;
}
