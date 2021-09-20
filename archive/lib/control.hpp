#ifndef PROTO_CONTROL_HPP
#define PROTO_CONTROL_HPP

#include "core.hpp"

namespace proto {

/**
 * PID Controller
 */
struct pid_t {
  real_t error_prev = 0.0;
  real_t error_sum = 0.0;

  real_t error_p = 0.0;
  real_t error_i = 0.0;
  real_t error_d = 0.0;

  real_t k_p = 0.0;
  real_t k_i = 0.0;
  real_t k_d = 0.0;

  pid_t();
  pid_t(const real_t k_p, const real_t k_i, const real_t k_d);
  ~pid_t();
};

/**
 * `pid_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const pid_t &pid);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p,
                  const real_t setpoint,
                  const real_t actual,
                  const real_t dt);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p, const real_t error, const real_t dt);

/**
 * Reset controller
 */
void pid_reset(pid_t &p);

/**
 * Carrot control
 */
struct carrot_ctrl_t {
  vec3s_t waypoints;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
  size_t wp_index = 0;
  real_t look_ahead_dist = 0.0;

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
                          const real_t look_ahead_dist);

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

} // namespace proto
#endif // PROTO_CONTROL_HPP
