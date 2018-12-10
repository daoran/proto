#ifndef PROTOTYPE_MAV_ATT_CTRL_HPP
#define PROTOTYPE_MAV_ATT_CTRL_HPP

#include "prototype/core/core.hpp"
#include "prototype/control/pid.hpp"

namespace prototype {

/**
 * Attitude controller
 */
struct att_ctrl_t {
  bool configured = false;

  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  double roll_limit[2] = {-30, 30};
  double pitch_limit[2] = {-30, 30};
  double max_thrust = 1.0;

  pid_t roll_ctrl;
  pid_t pitch_ctrl;
  pid_t yaw_ctrl;

  att_ctrl_t();
  ~att_ctrl_t();
};

/**
 * Configure attitude controller with `config_file`.
 * @returns 0 or -1 for success or failure
 */
int att_ctrl_configure(att_ctrl_t &ac, const std::string &config_file);

/**
 * Update attitude controller with `setpoints` (roll, pitch, yaw, z), `actual`
 * (roll, pitch, yaw, z) and time step `dt` in seconds [s].
 * @returns Motor command (m1, m2, m3, m4)
 */
vec4_t att_ctrl_update(att_ctrl_t &controller,
                       const vec4_t &setpoints,
                       const vec4_t &actual,
                       const double dt);

} //  namespace prototype
#endif // PROTOTYPE_MAV_ATT_CTRL_HPP
