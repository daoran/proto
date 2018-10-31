/**
 * @file
 * @defgroup core core
 */
#ifndef PROTOTYPE_CORE_HPP
#define PROTOTYPE_CORE_HPP

#include "prototype/core/config.hpp"
#include "prototype/core/data.hpp"
#include "prototype/core/euler.hpp"
#include "prototype/core/file.hpp"
#include "prototype/core/gps.hpp"
#include "prototype/core/linalg.hpp"
#include "prototype/core/log.hpp"
#include "prototype/core/math.hpp"
#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/core/stats.hpp"
#include "prototype/core/time.hpp"
#include "prototype/core/vision.hpp"

namespace prototype {
/**
 * @addtogroup util
 * @{
 */

// MACROS
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

// MACROS
#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

/** @} group util */
} //  namespace prototype
#endif // PROTOTYPE_CORE_HPP
