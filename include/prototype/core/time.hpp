/**
 * @file
 * @defgroup time time
 * @ingroup core
 */
#ifndef PROTOTYPE_CORE_TIME_HPP
#define PROTOTYPE_CORE_TIME_HPP

#include <sys/time.h>
#include <time.h>

namespace prototype {
/**
 * @addtogroup time
 * @{
 */

struct timespec tic();

float toc(struct timespec *tic);

float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

/** @} group time */
} //  namespace prototype
#endif // PROTOTYPE_CORE_TIME_HPP
