#include "prototype/control/att_ctrl.hpp"

namespace prototype {

att_ctrl_t::att_ctrl_t() {}

att_ctrl_t::att_ctrl_t(const vec3_t &roll_pid,
                       const vec3_t &pitch_pid,
                       const vec3_t &yaw_pid) {
  roll = pid_t{roll_pid(0), roll_pid(1), roll_pid(2)};
  pitch = pid_t{pitch_pid(0), pitch_pid(1), pitch_pid(2)};
  yaw = pid_t{yaw_pid(0), yaw_pid(1), yaw_pid(2)};
}

att_ctrl_t::~att_ctrl_t() {}

vec4_t att_ctrl_update(att_ctrl_t &ac,
                       const vec4_t &setpoints,
                       const vec4_t &actual,
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
  vec4_t outputs{-p - y + t,
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
