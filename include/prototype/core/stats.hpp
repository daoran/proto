/**
 * @file
 * @defgroup stats stats
 * @ingroup core
 */
#ifndef PROTOTYPE_CORE_STATS_HPP
#define PROTOTYPE_CORE_STATS_HPP

#include "prototype/core/math.hpp"

namespace prototype {
/**
 * @addtogroup stats
 * @{
 */

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

/** @} group stats */
} //  namespace prototype
#endif // PROTOTYPE_CORE_STATS_HPP
