#include "proto/mav/pos_ctrl.hpp"

namespace proto {

int pos_ctrl_configure(pos_ctrl_t &ctrl,
                       const std::string &config_file,
                       const std::string &prefix) {
  // Load config
  config_t config{config_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load config [%s]!", config_file.c_str());
    return -1;
  }

  const std::string p = (prefix == "") ? "" : prefix + ".";
  {
    vec3_t gains;
    parse(config, p + "roll_ctrl.gains", gains);
    parse(config, p + "roll_ctrl.limits", ctrl.roll_limits);
    ctrl.x_ctrl.k_p = gains(0);
    ctrl.x_ctrl.k_i = gains(1);
    ctrl.x_ctrl.k_d = gains(2);
  }
  {
    vec3_t gains;
    parse(config, p + "pitch_ctrl.gains", gains);
    parse(config, p + "pitch_ctrl.limits", ctrl.pitch_limits);
    ctrl.y_ctrl.k_p = gains(0);
    ctrl.y_ctrl.k_i = gains(1);
    ctrl.y_ctrl.k_d = gains(2);
  }
  {
    vec3_t gains;
    parse(config, p + "throttle_ctrl.gains", gains);
    parse(config, p + "throttle_ctrl.hover_throttle", ctrl.hover_throttle);
    ctrl.z_ctrl.k_p = gains(0);
    ctrl.z_ctrl.k_i = gains(1);
    ctrl.z_ctrl.k_d = gains(2);
  }

  // Convert roll and pitch limits from degrees to radians
  ctrl.roll_limits(0) = deg2rad(ctrl.roll_limits(0));
  ctrl.roll_limits(1) = deg2rad(ctrl.roll_limits(1));
  ctrl.pitch_limits(0) = deg2rad(ctrl.pitch_limits(0));
  ctrl.pitch_limits(1) = deg2rad(ctrl.pitch_limits(1));

  ctrl.ok = true;
  return 0;
}

vec4_t pos_ctrl_update(pos_ctrl_t &ctrl,
                       const vec3_t &setpoints,
                       const mat4_t &T_WB,
                       const double desired_yaw,
                       const double dt) {
  assert(ctrl.ok);

  // Check rate
  // ctrl.dt = dt;
  ctrl.dt += dt;
  if (ctrl.dt < 0.01) {
    return ctrl.outputs;
  }

  // Form actual position and yaw
  const vec3_t actual_pos = tf_trans(T_WB);
  const double actual_yaw = quat2euler(tf_quat(T_WB))(2);

  // Transform errors in world frame to body frame (excluding roll and pitch)
  const vec3_t errors_W{setpoints - actual_pos};
  const vec3_t rpy_WB{0.0, 0.0, actual_yaw};
  const mat3_t C_BW = euler123(rpy_WB);
  const vec3_t errors_B = C_BW * errors_W;

  // Roll, pitch, yaw and thrust
  double r = -pid_update(ctrl.x_ctrl, errors_B(1), dt);
  double p = pid_update(ctrl.y_ctrl, errors_B(0), dt);
  double y = desired_yaw;
  double t = ctrl.hover_throttle + pid_update(ctrl.z_ctrl, errors_B(2), dt);

  // Limit roll, pitch
  r = (r < ctrl.roll_limits(0)) ? ctrl.roll_limits(0) : r;
  r = (r > ctrl.roll_limits(1)) ? ctrl.roll_limits(1) : r;
  p = (p < ctrl.pitch_limits(0)) ? ctrl.pitch_limits(0) : p;
  p = (p > ctrl.pitch_limits(1)) ? ctrl.pitch_limits(1) : p;

  // Limit yaw
  y = (y > M_PI) ? (y - 2 * M_PI) : y;
  y = (y < -M_PI) ? (y + 2 * M_PI) : y;

  // Limit thrust
  t = (t > 1.0) ? 1.0 : t;
  t = (t < 0.0) ? 0.0 : t;

  // Keep track of outputs
  const vec4_t outputs{r, p, y, t};
  ctrl.outputs = outputs;
  ctrl.dt = 0.0;

  return outputs;
}

void pos_ctrl_reset(pos_ctrl_t &ctrl) {
  assert(ctrl.ok);
  pid_reset(ctrl.x_ctrl);
  pid_reset(ctrl.y_ctrl);
  pid_reset(ctrl.z_ctrl);
}

} //  namespace proto
