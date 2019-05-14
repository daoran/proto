#ifndef PROTOTYPE_MAV_ATT_CTRL_HPP
#define PROTOTYPE_MAV_ATT_CTRL_HPP

#include "prototype/core/core.hpp"
#include "prototype/control/pid.hpp"

namespace proto {

/**
 * Attitude controller
 */
struct att_ctrl_t {
  bool ok = false;

  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double max_thrust = 5.0;

  pid_t roll_ctrl;
  pid_t pitch_ctrl;
  pid_t yaw_ctrl;
};

/**
 * Configure attitude controller with `config_file`.
 * @returns 0 or -1 for success or failure
 */
int att_ctrl_configure(att_ctrl_t &ctrl,
                       const std::string &config_file,
                       const std::string &prefix="");

/**
 * Update attitude controller with `setpoints` (roll, pitch, yaw, z), actual
 * pose of the robot `T_WB` and time step `dt` in seconds [s].
 * @returns Motor command (m1, m2, m3, m4)
 */
vec4_t att_ctrl_update(att_ctrl_t &ctrl,
                       const vec4_t &setpoints,
                       const mat4_t &T_WB,
                       const double dt);

/**
 * Reset position control
 */
void att_ctrl_reset(att_ctrl_t &ctrl);

} //  namespace proto
#endif // PROTOTYPE_MAV_ATT_CTRL_HPP
