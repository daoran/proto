/**
 * @file
 * @ingroup control
 */
#ifndef PROTOTYPE_CONTROL_ATT_CTRL_HPP
#define PROTOTYPE_CONTROL_ATT_CTRL_HPP

#include "prototype/core.hpp"
#include "prototype/control/pid.hpp"

using namespace prototype;

namespace prototype {
/**
 * @addtogroup control
 * @{
 */

/**
 * Attitude controller
 */
struct att_ctrl {
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  struct pid roll = pid_setup(200.0, 0.001, 1.0);
  struct pid pitch = pid_setup(200.0, 0.001, 1.0);
  struct pid yaw = pid_setup(10.0, 0.001, 1.0);
};

/**
 * Setup attitude control
 */
struct att_ctrl att_ctrl_setup(const Vec3 &roll_pid,
                               const Vec3 &pitch_pid,
                               const Vec3 &yaw_pid);

/**
 * Update attitude controller
 *
 * @param[in] controller Attitude controller
 * @param[in] setpoints Setpoints (roll, pitch, yaw, z)
 * @param[in] actual Actual (roll, pitch, yaw, z)
 * @param[in] dt Time difference (s)
 *
 * @returns Motor command (m1, m2, m3, m4)
 */
Vec4 att_ctrl_update(struct att_ctrl &controller,
                     const Vec4 &setpoints,
                     const Vec4 &actual,
                     const double dt);

/** @} group quadrotor */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_ATT_CTRL_HPP
