#include "proto/mav/att_ctrl.hpp"

namespace proto {

int att_ctrl_configure(att_ctrl_t &ctrl,
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
    ctrl.roll_ctrl.k_p = gains(0);
    ctrl.roll_ctrl.k_i = gains(1);
    ctrl.roll_ctrl.k_d = gains(2);
  }
  {
    vec3_t gains;
    parse(config, p + "pitch_ctrl.gains", gains);
    parse(config, p + "pitch_ctrl.limits", ctrl.pitch_limits);
    ctrl.pitch_ctrl.k_p = gains(0);
    ctrl.pitch_ctrl.k_i = gains(1);
    ctrl.pitch_ctrl.k_d = gains(2);
  }
  {
    vec3_t gains;
    parse(config, p + "yaw_ctrl.gains", gains);
    ctrl.yaw_ctrl.k_p = gains(0);
    ctrl.yaw_ctrl.k_i = gains(1);
    ctrl.yaw_ctrl.k_d = gains(2);
  }

  // Convert roll and pitch limits from degrees to radians
  ctrl.roll_limits(0) = deg2rad(ctrl.roll_limits(0));
  ctrl.roll_limits(1) = deg2rad(ctrl.roll_limits(1));
  ctrl.pitch_limits(0) = deg2rad(ctrl.pitch_limits(0));
  ctrl.pitch_limits(1) = deg2rad(ctrl.pitch_limits(1));

  ctrl.ok = true;
  return 0;
}

vec4_t att_ctrl_update(att_ctrl_t &ctrl,
                       const vec4_t &setpoints,
                       const mat4_t &T_WB,
                       const double dt) {
  assert(ctrl.ok);

  // Check rate
  ctrl.dt += dt;
  if (ctrl.dt < 0.001) {
    return ctrl.outputs;
  }

  // Form actual
  const double z = tf_trans(T_WB)(2);
  const vec3_t rpy = quat2euler(tf_quat(T_WB));
  const vec4_t actual{rpy(0), rpy(1), rpy(2), z};

  // Calculate yaw error
  double actual_yaw = rad2deg(actual(2));
  double setpoint_yaw = rad2deg(setpoints(2));
  double error_yaw = setpoint_yaw - actual_yaw;

  // Wrap yaw
  if (error_yaw > 180.0) {
    error_yaw -= 360.0;
  } else if (error_yaw < -180.0) {
    error_yaw += 360.0;
  }
  error_yaw = deg2rad(error_yaw);

  // Roll, pitch and yaw
  double r = pid_update(ctrl.roll_ctrl, setpoints(0), actual(0), ctrl.dt);
  double p = pid_update(ctrl.pitch_ctrl, setpoints(1), actual(1), ctrl.dt);
  double y = pid_update(ctrl.yaw_ctrl, error_yaw, 0.0, ctrl.dt);
  r = (r < ctrl.roll_limits(0)) ? ctrl.roll_limits(0) : r;
  r = (r > ctrl.roll_limits(1)) ? ctrl.roll_limits(1) : r;
  p = (p < ctrl.pitch_limits(0)) ? ctrl.pitch_limits(0) : p;
  p = (p > ctrl.pitch_limits(1)) ? ctrl.pitch_limits(1) : p;

  // Thrust
  double t = ctrl.max_thrust * setpoints(3);
  t = (t > ctrl.max_thrust) ? ctrl.max_thrust : t;
  t = (t < 0.0) ? 0.0 : t;

  // Map roll, pitch, yaw and thrust to motor outputs
  const double m1 = -p - y + t;
  const double m2 = -r + y + t;
  const double m3 = p - y + t;
  const double m4 = r + y + t;
  vec4_t outputs{m1, m2, m3, m4};

  // Limit outputs
  for (int i = 0; i < 4; i++) {
    outputs(i) = (outputs(i) > ctrl.max_thrust) ? ctrl.max_thrust : outputs(i);
    outputs(i) = (outputs(i) < 0.0) ? 0.0 : outputs(i);
  }

  ctrl.outputs = outputs;
  ctrl.dt = 0.0;
  return outputs;
}

void att_ctrl_reset(att_ctrl_t &ctrl) {
  assert(ctrl.ok);
  pid_reset(ctrl.roll_ctrl);
  pid_reset(ctrl.pitch_ctrl);
  pid_reset(ctrl.yaw_ctrl);
}

} //  namespace proto
