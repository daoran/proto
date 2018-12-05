#ifndef PROTOTYPE_CORE_TIME_HPP
#define PROTOTYPE_CORE_TIME_HPP

#include <sys/time.h>
#include <time.h>

namespace prototype {

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

} //  namespace prototype
#endif // PROTOTYPE_CORE_TIME_HPP
