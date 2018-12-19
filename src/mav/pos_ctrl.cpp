#include "prototype/mav/pos_ctrl.hpp"

namespace proto {

int pos_ctrl_configure(pos_ctrl_t &pc, const std::string &config_file) {
  // Load config
  config_t config{config_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load config [%s]!", config_file.c_str());
    return -1;
  }

  parse(config, "roll_ctrl.k_p", pc.x_ctrl.k_p);
  parse(config, "roll_ctrl.k_i", pc.x_ctrl.k_i);
  parse(config, "roll_ctrl.k_d", pc.x_ctrl.k_d);
  parse(config, "roll_ctrl.min", pc.roll_limit[0]);
  parse(config, "roll_ctrl.max", pc.roll_limit[1]);

  parse(config, "pitch_ctrl.k_p", pc.y_ctrl.k_p);
  parse(config, "pitch_ctrl.k_i", pc.y_ctrl.k_i);
  parse(config, "pitch_ctrl.k_d", pc.y_ctrl.k_d);
  parse(config, "pitch_ctrl.min", pc.pitch_limit[0]);
  parse(config, "pitch_ctrl.max", pc.pitch_limit[1]);

  parse(config, "throttle_ctrl.k_p", pc.z_ctrl.k_p);
  parse(config, "throttle_ctrl.k_i", pc.z_ctrl.k_i);
  parse(config, "throttle_ctrl.k_d", pc.z_ctrl.k_d);
  parse(config, "throttle_ctrl.hover_throttle", pc.hover_throttle);

  // Convert roll and pitch limits from degrees to radians
  pc.roll_limit[0] = deg2rad(pc.roll_limit[0]);
  pc.roll_limit[1] = deg2rad(pc.roll_limit[1]);
  pc.pitch_limit[0] = deg2rad(pc.pitch_limit[0]);
  pc.pitch_limit[1] = deg2rad(pc.pitch_limit[1]);

  return 0;
}

vec4_t pos_ctrl_update(pos_ctrl_t &pc,
                       const vec3_t &setpoints,
                       const vec4_t &actual,
                       const double yaw,
                       const double dt) {
  // Check rate
  pc.dt += dt;
  if (pc.dt < 0.01) {
    return pc.outputs;
  }

  // Calculate RPY errors relative to quadrotor by incorporating yaw
  vec3_t errors{setpoints(0) - actual(0),
                setpoints(1) - actual(1),
                setpoints(2) - actual(2)};
  const vec3_t euler{0.0, 0.0, actual(3)};
  const mat3_t R = euler123ToRot(euler);
  errors = R * errors;

  // Roll, pitch, yaw and thrust
  double r = -pid_update(pc.x_ctrl, errors(1), dt);
  double p = pid_update(pc.y_ctrl, errors(0), dt);
  double y = yaw;
  double t = 0.5 + pid_update(pc.z_ctrl, errors(2), dt);
  vec4_t outputs{r, p, y, t};

  // Limit roll, pitch
  for (int i = 0; i < 2; i++) {
    if (outputs(i) > deg2rad(30.0)) {
      outputs(i) = deg2rad(30.0);
    } else if (outputs(i) < deg2rad(-30.0)) {
      outputs(i) = deg2rad(-30.0);
    }
  }

  // Limit yaw
  while (outputs(2) > deg2rad(360.0)) {
    outputs(2) -= deg2rad(360.0);
  }
  while (outputs(2) < deg2rad(0.0)) {
    outputs(2) += deg2rad(360.0);
  }

  // Limit thrust
  if (outputs(3) > 1.0) {
    outputs(3) = 1.0;
  } else if (outputs(3) < 0.0) {
    outputs(3) = 0.0;
  }

  // Yaw first if threshold reached
  if (fabs(yaw - actual(3)) > deg2rad(2)) {
    outputs(0) = 0.0;
    outputs(1) = 0.0;
  }

  // Keep track of outputs
  pc.outputs = outputs;
  pc.dt = 0.0;

  return outputs;
}

void pos_ctrl_reset(pos_ctrl_t &pc) {
  pid_reset(pc.x_ctrl);
  pid_reset(pc.y_ctrl);
  pid_reset(pc.z_ctrl);
}

} //  namespace proto
