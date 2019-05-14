#ifndef PROTOTYPE_CORE_TIME_HPP
#define PROTOTYPE_CORE_TIME_HPP

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include <vector>
#include <cstdint>
#include <string>

namespace proto {

typedef uint64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

/**
 * Print timestamp.
 */
void timestamp_print(const timestamp_t &ts,
                     const std::string &prefix="");

/**
 * Convert ts to second.
 */
double ts2sec(const timestamp_t &ts);

/**
 * Convert nano-second to second.
 */
double ns2sec(const uint64_t ns);

/**
 * Start timer.
 */
struct timespec tic();

/**
 * Stop timer and return number of seconds.
 */
float toc(struct timespec *tic);

/**
 * Stop timer and return miliseconds elasped.
 */
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

} //  namespace proto
#endif // PROTOTYPE_CORE_TIME_HPP
