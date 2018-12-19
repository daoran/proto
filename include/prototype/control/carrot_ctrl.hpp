#ifndef PROTOTYPE_CONTROL_CARROT_CTRL_HPP
#define PROTOTYPE_CONTROL_CARROT_CTRL_HPP

#include <vector>

#include "prototype/core/core.hpp"

namespace proto {

/**
 * Carrot control
 */
struct carrot_ctrl_t {
  vec3s_t waypoints;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
  size_t wp_index = 0;
  double look_ahead_dist = 0.0;

  carrot_ctrl_t();
  ~carrot_ctrl_t();
};

/**
 * Configure carrot control using a list of position `waypoints` (x, y, z), and
 * a `look_ahead` distance in [m].
 *
 * @returns 0 for success, -1 for failure
 */
int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const double look_ahead_dist);

/**
 * Calculate closest point along current trajectory using current position
 * `pos`, and outputs the closest point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result);

/**
 * Calculate carrot point using current position `pos`, and outputs the carrot
 * point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result);

/**
 * Update carrot controller using current position `pos` and outputs the carrot
 * point in `result`.
 *
 * @returns 0 for success, 1 for all waypoints reached and -1 for failure
 */
int carrot_ctrl_update(carrot_ctrl_t &cc, const vec3_t &pos, vec3_t &carrot_pt);

} //  namespace proto
#endif // PROTOTYPE_CONTROL_CARROT_CTRL_HPP
