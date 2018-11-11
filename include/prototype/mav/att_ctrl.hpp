#ifndef PROTOTYPE_MAV_ATT_CTRL_HPP
#define PROTOTYPE_MAV_ATT_CTRL_HPP

#include "prototype/core/core.hpp"
#include "prototype/control/pid.hpp"

using namespace prototype;

namespace prototype {

/**
 * Attitude controller
 */
struct att_ctrl_t {
  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  pid_t roll{200.0, 0.001, 1.0};
  pid_t pitch{200.0, 0.001, 1.0};
  pid_t yaw{10.0, 0.001, 1.0};

  att_ctrl_t();
  att_ctrl_t(const vec3_t &roll_pid,
             const vec3_t &pitch_pid,
             const vec3_t &yaw_pid);
  ~att_ctrl_t();
};

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
