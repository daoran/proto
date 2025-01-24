#include "time.h"

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic(void) {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  assert(tic != NULL);
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in milli-seconds
 */
float mtoc(struct timespec *tic) {
  assert(tic != NULL);
  return toc(tic) * 1000.0;
}

/**
 * Get time now since epoch.
 * @return Time now in nano-seconds since epoch
 */
timestamp_t time_now(void) {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  const time_t sec = spec.tv_sec;
  const long int ns = spec.tv_nsec;
  const uint64_t BILLION = 1000000000L;

  return (uint64_t) sec * BILLION + (uint64_t) ns;
}

/**
 * Convert string to timestamp
 */
timestamp_t str2ts(const char *ts_str) { return strtoll(ts_str, NULL, 10); }

/**
 * Convert timestamp to seconds
 */
double ts2sec(const timestamp_t ts) { return ts * 1e-9; }

/**
 * Convert seconds to timestamp
 */
timestamp_t sec2ts(const double time_s) { return time_s * 1e9; }
