#include "prototype/mav/att_ctrl.hpp"

namespace prototype {

att_ctrl_t::att_ctrl_t() {}

att_ctrl_t::~att_ctrl_t() {}

int att_ctrl_configure(att_ctrl_t &ac, const std::string &config_file) {
  // Load config
  config_t config{config_file};
  if (config.ok == false) {
    return -1;
  }
  parse(config, "roll_ctrl.k_p", ac.roll_ctrl.k_p);
  parse(config, "roll_ctrl.k_i", ac.roll_ctrl.k_i);
  parse(config, "roll_ctrl.k_d", ac.roll_ctrl.k_d);
  parse(config, "roll_ctrl.min", ac.roll_limit[0]);
  parse(config, "roll_ctrl.max", ac.roll_limit[1]);

  parse(config, "pitch_ctrl.k_p", ac.pitch_ctrl.k_p);
  parse(config, "pitch_ctrl.k_i", ac.pitch_ctrl.k_i);
  parse(config, "pitch_ctrl.k_d", ac.pitch_ctrl.k_d);
  parse(config, "pitch_ctrl.min", ac.pitch_limit[0]);
  parse(config, "pitch_ctrl.max", ac.pitch_limit[1]);

  parse(config, "yaw_ctrl.k_p", ac.yaw_ctrl.k_p);
  parse(config, "yaw_ctrl.k_i", ac.yaw_ctrl.k_i);
  parse(config, "yaw_ctrl.k_d", ac.yaw_ctrl.k_d);

  // Convert roll and pitch limits from degrees to radians
  ac.roll_limit[0] = deg2rad(ac.roll_limit[0]);
  ac.roll_limit[1] = deg2rad(ac.roll_limit[1]);
  ac.pitch_limit[0] = deg2rad(ac.pitch_limit[0]);
  ac.pitch_limit[1] = deg2rad(ac.pitch_limit[1]);

  ac.configured = true;
  return 0;
}

vec4_t att_ctrl_update(att_ctrl_t &ac,
                       const vec4_t &setpoints,
                       const vec4_t &actual,
                       const double dt) {
  assert(ac.configured);

  // Check rate
  ac.dt += dt;
  if (ac.dt < 0.0011) {
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
  double r = pid_update(ac.roll_ctrl, setpoints(0), actual(0), ac.dt);
  double p = pid_update(ac.pitch_ctrl, setpoints(1), actual(1), ac.dt);
  double y = pid_update(ac.yaw_ctrl, error_yaw, 0.0, ac.dt);

  // Thrust
  double t = setpoints(3);
  t = (t > ac.max_thrust) ? ac.max_thrust : t;
  t = (t < 0.0) ? 0.0 : t;

  // Map roll, pitch, yaw and thrust to motor outputs
  const double m1 = -p - y + t;
  const double m2 = -r + y + t;
  const double m3 = p - y + t;
  const double m4 = r + y + t;
  vec4_t outputs{m1, m2, m3, m4};

  // Limit outputs
  for (int i = 0; i < 4; i++) {
    outputs(i) = (outputs(i) > ac.max_thrust) ? ac.max_thrust : outputs(i);
    outputs(i) = (outputs(i) < 0.0) ? 0.0 : outputs(i);
  }

  ac.outputs = outputs;
  ac.dt = 0.0;
  return outputs;
}

} //  namespace prototype
