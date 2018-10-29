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
struct carrot_ctrl_t {
  std::vector<vec3_t> waypoints;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
  size_t wp_index = 0;
  double look_ahead_dist = 0.0;

  carrot_ctrl_t();
  ~carrot_ctrl_t();
};

/**
 * Configure
 *
 * @param[in,out] cc Carrot control
 * @param[in] waypoints Waypoints
 * @param[in] look_ahead_dist Look ahead distance (m)
 * @returns 0 for success, -1 for failure
 */
int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const std::vector<vec3_t> &waypoints,
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
int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result);

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
int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result);

/**
  * Update carrot controller
  *
  * @param[in,out] cc Carrot control
  * @param[in] pos Current position
  * @param[out] carrot_pt Carrot point
  * @returns 0 for success, 1 for all waypoints reached and -1 for failure
  */
int carrot_ctrl_update(carrot_ctrl_t &cc, const vec3_t &pos, vec3_t &carrot_pt);

/** @} group control */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_CARROT_CTRL_HPP
