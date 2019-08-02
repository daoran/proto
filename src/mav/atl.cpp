#include "proto/mav/atl.hpp"

namespace proto {

atl_t::atl_t() {}

atl_t::atl_t(const std::string &config_file) {
  atl_configure(*this, config_file);
}

int atl_configure(atl_t &atl, const std::string &config_file) {
  config_t config{config_file};
  vec4_t cam0_intrinsics;
  mat4_t T_BC0 = I(4);
  parse(config, "settings.recover_height", atl.recover_height);
  parse(config, "settings.auto_track", atl.auto_track);
  parse(config, "settings.auto_land", atl.auto_land);
  parse(config, "settings.auto_disarm", atl.auto_disarm);
  parse(config, "settings.target_lost_threshold", atl.target_lost_threshold);
  parse(config, "settings.min_discover_time", atl.min_discover_time);
  parse(config, "settings.min_tracking_time", atl.min_tracking_time);
  parse(config, "settings.home_point", atl.home_point);
  parse(config, "cam0.intrinsics", cam0_intrinsics);
  parse(config, "T_BC0", T_BC0);

  atl.cam0_pinhole = pinhole_t{cam0_intrinsics};
  atl.T_BC0 = T_BC0;

  if (att_ctrl_configure(atl.att_ctrl, config_file, "att_ctrl") != 0) {
    LOG_ERROR("Failed to configure attitude controller!");
    return -1;
  }

  if (pos_ctrl_configure(atl.pos_ctrl, config_file, "pos_ctrl") != 0) {
    LOG_ERROR("Failed to configure position controller!");
    return -1;
  }

  if (tk_ctrl_configure(atl.tk_ctrl, config_file, "tk_ctrl") != 0) {
    LOG_ERROR("Failed to configure tracking controller!");
    return -1;
  }

  if (lz_detector_configure(atl.lz_detector, config_file, "landing_zone") !=
      0) {
    LOG_ERROR("Failed to configure landing zone detector!");
    return -1;
  }

  atl.position_setpoint << 0.0, 0.0, atl.recover_height;
  atl.yaw_setpoint = 0.0;

  return 0;
}

int atl_detect_lz(atl_t &atl, const cv::Mat &image) {
  return lz_detector_detect(atl.lz_detector,
                            image,
                            atl.cam0_pinhole,
                            atl.T_BC0,
                            atl.lz);
}

vec4_t atl_step_hover_mode(atl_t &atl, const mat4_t &T_WB, const double dt) {
  // Position control
  const vec4_t att_setpoint = pos_ctrl_update(atl.pos_ctrl,
                                              atl.position_setpoint,
                                              T_WB,
                                              atl.yaw_setpoint,
                                              dt);

  // Attitude control
  const vec4_t u = att_ctrl_update(atl.att_ctrl, att_setpoint, T_WB, dt);

  return u;
}

vec4_t atl_step_discover_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const double dt) {
  // Hover in place
  const vec4_t u = atl_step_hover_mode(atl, T_WB, dt);
  if (atl.discover_tic.tv_sec == 0) {
    atl.discover_tic = tic();
  }

  // Check conditions for transition
  std::vector<bool> conditions{false, false};
  conditions[0] = lz.detected;
  conditions[1] = toc(&atl.discover_tic) > atl.min_discover_time;

  // Transition to tracking mode
  if (all_true(conditions) && atl.auto_track) {
    LOG_INFO("Transitioning to [TRACKING MODE]!");
    atl.mode = TRACKING_MODE;
    atl.discover_tic = (struct timespec){0, 0};
    tk_ctrl_reset(atl.tk_ctrl);
    pos_ctrl_reset(atl.pos_ctrl);
  }

  return u;
}

vec4_t atl_step_tracking_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const double dt) {
  vec4_t att_setpoint;

  if (lz.detected) {
    // Tracking control
    const mat4_t T_BZ = lz.T_BC * lz.T_CZ;
    att_setpoint = tk_ctrl_update(atl.tk_ctrl,
                                  T_BZ,
                                  T_WB,
                                  atl.recover_height,
                                  atl.yaw_setpoint,
                                  dt);

  } else {
    // Position control
    att_setpoint = pos_ctrl_update(atl.pos_ctrl,
                                   atl.position_setpoint,
                                   T_WB,
                                   atl.yaw_setpoint,
                                   dt);
  }

  // Attitude control
  const vec4_t u = att_ctrl_update(atl.att_ctrl, att_setpoint, T_WB, dt);

  // Update position setpoint and start tracking timer
  if (lz.detected) {
    atl.position_setpoint = tf_trans(T_WB);
  }
  if (atl.tracking_tic.tv_sec == 0) {
    atl.tracking_tic = tic();
  }

  // Check conditions
  std::vector<bool> conditions = {false, false};
  conditions[0] = lz.detected;
  conditions[1] = mtoc(&atl.tracking_tic) > atl.min_tracking_time;

  // Transition
  if (all_true(conditions) && atl.auto_land) {
    // Transition to landing mode
    LOG_INFO("Transitioning to [LANDING MODE]!");
    atl.mode = LANDING_MODE;
    atl.tracking_tic = (struct timespec){0, 0};
    tk_ctrl_reset(atl.tk_ctrl);
    pos_ctrl_reset(atl.pos_ctrl);

  } else if (lz.detected == false) {
    // Transition back to discover mode
    LOG_INFO("Landing Target is lost!");
    LOG_INFO("Transitioning back to [DISCOVER_MODE]!");
    atl.mode = DISCOVER_MODE;
    atl.tracking_tic = (struct timespec){0, 0};
    atl.position_setpoint(2) = atl.recover_height;
    tk_ctrl_reset(atl.tk_ctrl);
    pos_ctrl_reset(atl.pos_ctrl);
  }

  return u;
}

// vec4_t atl_step_landing_mode(atl_t &atl, const double dt) {
//   // // land on target
//   // atl.landing_controller.update(atl.landing_target.position_B,
//   //                               atl.velocity,
//   //                               atl.yaw_setpoint,
//   //                               dt);
//   const vec3_t r_WB = tf_trans(atl.T_WB);
//   const double yaw = quat2euler(tf_quat(atl.T_WB))(2);
//   const vec3_t setpoints = atl.r_CZ;
//   const vec4_t actual{r_WB(0), r_WB(1), r_WB(2), yaw};
//
//   vec4_t u;
//
//   // update hover position and tracking timer
//   atl.position_setpoint = r_WB;
//   if (atl.landing_tic.tv_sec == 0) {
//     atl.landing_tic = tic();
//   }
//
//   // check conditions
//   std::vector<bool> conditions = {false, false};
//   conditions[0] = atl.tracking_lz;
//   conditions[1] = atl.r_CZ.norm() < 0.1;
//
//   // Transition
//   if (all_true(conditions) && atl.auto_disarm) {
//     // Transition to disarm mode
//     atl.mode = DISARM_MODE;
//     atl.landing_tic = (struct timespec){0, 0};
//
//   } else if (atl.tracking_lz == false) {
//     // Transition back to discovery mode
//     LOG_INFO("Landing Target is losted!");
//     LOG_INFO("Transitioning back to [DISCOVER_MODE]!");
//     atl.mode = DISCOVER_MODE;
//     atl.landing_tic = (struct timespec){0, 0};
//     atl.position_setpoint(2) = atl.recover_height;
//   }
//
//   return u;
// }

int atl_step(
    atl_t &atl, const mat4_t T_WB, const lz_t &lz, const double dt, vec4_t &u) {
  switch (atl.mode) {
  case DISARM_MODE: return -1; break;
  case IDLE_MODE: return 0; break;
  case HOVER_MODE: u = atl_step_hover_mode(atl, T_WB, dt); break;
  case DISCOVER_MODE: u = atl_step_discover_mode(atl, T_WB, lz, dt); break;
  case TRACKING_MODE: u = atl_step_tracking_mode(atl, T_WB, lz, dt); break;
  // case LANDING_MODE: u = atl_step_landing_mode(atl, lz, dt); break;
  default:
    LOG_ERROR("Invalid mode [%d]!", atl.mode);
    LOG_ERROR("Falling back into [HOVER_MODE]!");
    atl_step_hover_mode(atl, T_WB, dt);
    break;
  }

  return 0;
}

} //  namespace proto
