/** @file
 * @ingroup control
 */
#ifndef PROTOTYPE_CONTROL_CARROT_CTRL_HPP
#define PROTOTYPE_CONTROL_CARROT_CTRL_HPP

#include <vector>

#include "prototype/core.hpp"

using namespace prototype;

namespace prototype {
/**
 * @addtogroup control
 * @{
 */

/**
 * Carrot control
 */
struct carrot_ctrl {
  std::vector<Vec3> waypoints;
  Vec3 wp_start = Vec3::Zero();
  Vec3 wp_end = Vec3::Zero();
  size_t wp_index = 0;
  double look_ahead_dist = 0.0;
};

/**
 * Configure
 *
 * @param[in,out] cc Carrot control
 * @param[in] waypoints Waypoints
 * @param[in] look_ahead_dist Look ahead distance (m)
 * @returns 0 for success, -1 for failure
 */
int carrot_ctrl_configure(struct carrot_ctrl &cc,
                          const std::vector<Vec3> &waypoints,
                          const double look_ahead_dist);

/**
 * Calculate closest point
 *
 * @param[in,out] cc Carrot control
 * @param[in] pos Position of robot
 * @param[out] result Result is a the closest point between `wp_start` and
 * `wp_end`
 *
 * @returns A number to denote progress along the waypoint,
 * - t == -1: Position is before `wp_start`
 * - t == 0: Position is between `wp_start` and `wp_end`
 * - t == 1: Position is after `wp_end`
 */
int carrot_ctrl_closest_point(const struct carrot_ctrl &cc,
                              const Vec3 &pos,
                              Vec3 &result);

/**
  * Calculate carrot point
  *
  * @param[in] cc Carrot control
  * @param[in] pos Position of robot
  * @param[out] result Result is a the carrot point between `wp_start` and
  * `wp_end`
  *
  * @returns A number to denote progress along the waypoint,
  * - t == -1: Position is before `wp_start`
  * - t == 0: Position is between `wp_start` and `wp_end`
  * - t == 1: Position is after `wp_end`
  */
int carrot_ctrl_carrot_point(const struct carrot_ctrl &cc,
                             const Vec3 &pos,
                             Vec3 &result);

/**
  * Update carrot controller
  *
  * @param[in,out] cc Carrot control
  * @param[in] pos Current position
  * @param[out] carrot_pt Carrot point
  * @returns 0 for success, 1 for all waypoints reached and -1 for failure
  */
int carrot_ctrl_update(struct carrot_ctrl &cc, const Vec3 &pos, Vec3 &carrot_pt);

/** @} group control */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_CARROT_CTRL_HPP
