#include "prototype/control/att_ctrl.hpp"

namespace prototype {

struct att_ctrl att_ctrl_setup(const Vec3 &roll_pid,
                               const Vec3 &pitch_pid,
                               const Vec3 &yaw_pid) {
  struct att_ctrl ac;

  ac.roll = pid_setup(roll_pid(0), roll_pid(1), roll_pid(2));
  ac.pitch = pid_setup(pitch_pid(0), pitch_pid(1), pitch_pid(2));
  ac.yaw = pid_setup(yaw_pid(0), yaw_pid(1), yaw_pid(2));

  return ac;
}

Vec4 att_ctrl_update(struct att_ctrl &ac,
                     const Vec4 &setpoints,
                     const Vec4 &actual,
                     const double dt) {
  // Check rate
  ac.dt += dt;
  if (ac.dt < 0.001) {
    return ac.outputs;
  }

  // Calculate yaw error
  double actual_yaw = rad2deg(actual(2));
  double setpoint_yaw = rad2deg(setpoints(2));
  double error_yaw = setpoint_yaw - actual_yaw;
  if (error_yaw > 180.0) {
    error_yaw -= 360.0;
  } else if (error_yaw < -180.0) {
    error_yaw += 360.0;
  }
  error_yaw = deg2rad(error_yaw);

  // Roll, pitch and yaw
  double r = pid_update(ac.roll, setpoints(0), actual(0), ac.dt);
  double p = pid_update(ac.pitch, setpoints(1), actual(1), ac.dt);
  double y = pid_update(ac.yaw, error_yaw, 0.0, ac.dt);

  // Thrust
  double max_thrust = 5.0;
  double t = max_thrust * setpoints(3);  // convert relative to true thrust
  t = (t > max_thrust) ? max_thrust : t; // limit thrust
  t = (t < 0) ? 0.0 : t;                 // limit thrust

  // Map roll, pitch, yaw and thrust to motor outputs
  // clang-format off
  Vec4 outputs{-p - y + t,
               -r + y + t,
               p - y + t,
               r + y + t};
  // clang-format on

  // Limit outputs
  for (int i = 0; i < 4; i++) {
    if (outputs(i) > max_thrust) {
      outputs(i) = max_thrust;
    } else if (outputs(i) < 0.0) {
      outputs(i) = 0.0;
    }
  }

  // Keep track of outputs
  ac.outputs = outputs;

  // Reset time
  ac.dt = 0.0;

  return outputs;
}

} //  namespace prototype
