#include "proto/mav/atl.hpp"

namespace proto {

/*****************************************************************************
 *                       ATTITUDE CONTROLLER
 ****************************************************************************/

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
                       const real_t dt) {
  assert(ctrl.ok);

  // Check rate
  ctrl.dt += dt;
  if (ctrl.dt < 0.001) {
    return ctrl.outputs;
  }

  // Form actual
  const real_t z = tf_trans(T_WB)(2);
  const vec3_t rpy = quat2euler(tf_quat(T_WB));
  const vec4_t actual{rpy(0), rpy(1), rpy(2), z};

  // Calculate yaw error
  real_t actual_yaw = rad2deg(actual(2));
  real_t setpoint_yaw = rad2deg(setpoints(2));
  real_t error_yaw = setpoint_yaw - actual_yaw;

  // Wrap yaw
  if (error_yaw > 180.0) {
    error_yaw -= 360.0;
  } else if (error_yaw < -180.0) {
    error_yaw += 360.0;
  }
  error_yaw = deg2rad(error_yaw);

  // Roll, pitch and yaw
  real_t r = pid_update(ctrl.roll_ctrl, setpoints(0), actual(0), ctrl.dt);
  real_t p = pid_update(ctrl.pitch_ctrl, setpoints(1), actual(1), ctrl.dt);
  real_t y = pid_update(ctrl.yaw_ctrl, error_yaw, 0.0, ctrl.dt);
  r = (r < ctrl.roll_limits(0)) ? ctrl.roll_limits(0) : r;
  r = (r > ctrl.roll_limits(1)) ? ctrl.roll_limits(1) : r;
  p = (p < ctrl.pitch_limits(0)) ? ctrl.pitch_limits(0) : p;
  p = (p > ctrl.pitch_limits(1)) ? ctrl.pitch_limits(1) : p;

  // Thrust
  real_t t = ctrl.max_thrust * setpoints(3);
  t = (t > ctrl.max_thrust) ? ctrl.max_thrust : t;
  t = (t < 0.0) ? 0.0 : t;

  // Map roll, pitch, yaw and thrust to motor outputs
  const real_t m1 = -p - y + t;
  const real_t m2 = -r + y + t;
  const real_t m3 = p - y + t;
  const real_t m4 = r + y + t;
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

/*****************************************************************************
 *                        POSITION CONTROLLER
 ****************************************************************************/

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
                       const real_t desired_yaw,
                       const real_t dt) {
  assert(ctrl.ok);

  // Check rate
  // ctrl.dt = dt;
  ctrl.dt += dt;
  if (ctrl.dt < 0.01) {
    return ctrl.outputs;
  }

  // Form actual position and yaw
  const vec3_t actual_pos = tf_trans(T_WB);
  const real_t actual_yaw = quat2euler(tf_quat(T_WB))(2);

  // Transform errors in world frame to body frame (excluding roll and pitch)
  const vec3_t errors_W{setpoints - actual_pos};
  const vec3_t rpy_WB{0.0, 0.0, actual_yaw};
  const mat3_t C_BW = euler123(rpy_WB);
  const vec3_t errors_B = C_BW * errors_W;

  // Roll, pitch, yaw and thrust
  real_t r = -pid_update(ctrl.x_ctrl, errors_B(1), dt);
  real_t p = pid_update(ctrl.y_ctrl, errors_B(0), dt);
  real_t y = desired_yaw;
  real_t t = ctrl.hover_throttle + pid_update(ctrl.z_ctrl, errors_B(2), dt);

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

/*****************************************************************************
 *                          TRACKING CONTROLLER
 ****************************************************************************/

int tk_ctrl_configure(tk_ctrl_t &ctrl,
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

  return 0;
}

vec4_t tk_ctrl_update(tk_ctrl_t &ctrl,
                      const mat4_t &T_BZ,
                      const mat4_t &T_WB,
                      const real_t desired_height,
                      const real_t desired_yaw,
                      const real_t dt) {
  // Check rate
  ctrl.dt += dt;
  if (ctrl.dt < 0.01) {
    return ctrl.outputs;
  }

  // Form landing zone to body relative error
  // Note: Key important thing here is we don't want to land just yet, so lets
  // keep at a desired height at the moment
  const real_t actual_height = T_WB(2, 3);
  vec3_t r_BZ = tf_trans(T_BZ);
  r_BZ(2) = desired_height - actual_height;

  // Adjust relative errors by incorporating yaw in world frame
  const real_t actual_yaw = quat2euler(tf_quat(T_WB))(2);
  const vec3_t rpy_WB{0.0, 0.0, actual_yaw};
  const mat3_t C_BW = euler123(rpy_WB);
  const vec3_t errors_B = C_BW * r_BZ;

  // Roll, pitch, yaw and thrust
  real_t r = -pid_update(ctrl.x_ctrl, errors_B(1), dt);
  real_t p = pid_update(ctrl.y_ctrl, errors_B(0), dt);
  real_t y = desired_yaw;
  real_t t = ctrl.hover_throttle + pid_update(ctrl.z_ctrl, errors_B(2), dt);

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

void tk_ctrl_reset(tk_ctrl_t &ctrl) {
  // assert(ctrl.ok);
  pid_reset(ctrl.x_ctrl);
  pid_reset(ctrl.y_ctrl);
  pid_reset(ctrl.z_ctrl);
}

// /*****************************************************************************
//  *                          WAYPOINT CONTROLLER
//  ****************************************************************************/
//
// int wp_ctrl_update(wp_ctrl_t &wc,
//                    wp_mission_t &m,
//                    const vec3_t &p_G,
//                    const vec3_t &v_G,
//                    const vec3_t &rpy_G,
//                    const real_t dt) {
//   // Check rate
//   wc.dt += dt;
//   if (wc.dt < 0.01) {
//     return 0;
//   }
//
//   // Current waypoint
//   vec3_t wp_G = vec3_t::Zero();
//   int retval = wp_mission_update(m, p_G, wp_G);
//   if (retval != 0) {
//     return retval;
//   }
//
//   // // Calculate waypoint relative to quadrotor
//   // mat4_t T_P_W = zeros(4, 4);
//   // T_P_W.block(0, 0, 3, 3) = euler123(yaw(rpy_G));
//   // T_P_W(3, 3) = 1.0;
//   // const vec4_t wp_B_homo{wp_G(0) - p_G(0),
//   //                      wp_G(1) - p_G(1),
//   //                      wp_G(2) - p_G(2),
//   //                      1.0};
//   // const vec4_t errors = T_P_W * wp_B_homo;
//
//   // std::cout << "T_P_W:\n" << T_P_W << std::endl;
//   // std::cout << "wp_G: " << wp_G.transpose() << std::endl;
//   // std::cout << "p_G: " << p_G.transpose() << std::endl;
//   // std::cout << "wp_B_homo: " << wp_B_homo.transpose() << std::endl;
//   // std::cout << std::endl;
//
//   // // Calculate velocity relative to quadrotor
//   // const vec4_t v_G_homo{v_G(0), v_G(1), v_G(2), 1.0};
//   // const vec4_t v_B = T_P_W * v_G_homo;
//
//   // Calculate RPY errors relative to quadrotor by incorporating yaw
//   vec3_t errors{wp_G(0) - p_G(0), wp_G(1) - p_G(1), wp_G(2) - p_G(2)};
//   const vec3_t euler{0.0, 0.0, rpy_G(2)};
//   const mat3_t R = euler123(euler);
//   errors = R * errors;
//
//   // Roll
//   real_t r = -pid_update(wc.ct_controller, errors(1), wc.dt);
//
//   // Pitch
//   // real_t error_forward = m.desired_velocity - v_B(0);
//   // real_t p = wc.at_controller.update(error_forward, wc.dt);
//   real_t p = pid_update(wc.at_controller, errors(0), wc.dt);
//
//   // Yaw
//   // real_t y = 0.2 * mission_waypoint_heading(m);
//   real_t y = 0.0;
//
//   // Throttle
//   real_t t = wc.hover_throttle;
//   t += pid_update(wc.z_controller, errors(2), wc.dt);
//   t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch
//
//   // Limit roll, pitch and throttle
//   r = (r < wc.roll_limit[0]) ? wc.roll_limit[0] : r;
//   r = (r > wc.roll_limit[1]) ? wc.roll_limit[1] : r;
//   p = (p < wc.pitch_limit[0]) ? wc.pitch_limit[0] : p;
//   p = (p > wc.pitch_limit[1]) ? wc.pitch_limit[1] : p;
//   t = (t < 0.0) ? 0.0 : t;
//   t = (t > 1.0) ? 1.0 : t;
//
//   // Keep track of setpoints and outputs
//   wc.setpoints = wp_G;
//   wc.outputs << r, p, y, t;
//   wc.dt = 0.0;
//
//   return 0;
// }
//
// void wp_ctrl_reset(wp_ctrl_t &wc) {
//   pid_reset(wc.at_controller);
//   pid_reset(wc.ct_controller);
//   pid_reset(wc.z_controller);
//   pid_reset(wc.yaw_controller);
// }

/*****************************************************************************
 *                              LANDING ZONE
 ****************************************************************************/

lz_detector_t::lz_detector_t() {}

lz_detector_t::lz_detector_t(const std::vector<int> &tag_ids,
                             const std::vector<real_t> &tag_sizes) {
  if (lz_detector_configure(*this, tag_ids, tag_sizes) != 0) {
    FATAL("Failed to configure landing zone!");
  }
}

lz_detector_t::lz_detector_t(const std::string &config_file,
                             const std::string &prefix) {
  if (lz_detector_configure(*this, config_file, prefix) != 0) {
    FATAL("Failed to configure landing zone!");
  }
}

lz_detector_t::~lz_detector_t() {
  if (det != nullptr) {
    delete det;
  }
}

void lz_print(const lz_t &lz) {
  printf("detected: %s\n", (lz.detected) ? "true" : "false");
  print_matrix("T_CZ", lz.T_CZ);
  printf("\n");
}

int lz_detector_configure(lz_detector_t &lz,
                          const std::vector<int> &tag_ids,
                          const std::vector<real_t> &tag_sizes) {
  if (tag_ids.size() != tag_sizes.size()) {
    LOG_ERROR("tag_ids.size() != tag_sizes.size()");
    return -1;
  }

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const real_t tag_sz = tag_sizes[i];
    lz.targets[tag_id] = tag_sz;
  }
  lz.det = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
  lz.ok = true;

  return 0;
}

int lz_detector_configure(lz_detector_t &lz,
                          const std::string &config_file,
                          const std::string &prefix) {
  std::vector<int> tag_ids;
  std::vector<real_t> tag_sizes;

  const std::string p = (prefix == "") ? "" : prefix + ".";
  config_t config{config_file};
  parse(config, p + "tag_ids", tag_ids);
  parse(config, p + "tag_sizes", tag_sizes);

  return lz_detector_configure(lz, tag_ids, tag_sizes);
}

int lz_detector_detect(const lz_detector_t &lz,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       mat4_t &T_CZ) {
  // Convert colour image to gray
  cv::Mat gray_image;
  cvtColor(image, gray_image, CV_BGR2GRAY);

  // Extract camera intrinsics
  const real_t fx = pinhole.fx;
  const real_t fy = pinhole.fy;
  const real_t cx = pinhole.cx;
  const real_t cy = pinhole.cy;

  // Calculate relative pose
  bool detected = false;
  const auto detections = lz.det->extractTags(gray_image);
  for (const AprilTags::TagDetection &tag : detections) {
    if (lz.targets.count(tag.id) == 0) {
      continue;
    }

    const real_t tag_size = lz.targets.at(tag.id);
    T_CZ = tag.getRelativeTransform(tag_size, fx, fy, cx, cy).cast<real_t>();
    detected = true;
    break;
  }

  return (detected) ? 1 : 0;
}

int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       const mat4_t &T_BC,
                       lz_t &lz) {
  mat4_t T_CZ = I(4);
  if (lz_detector_detect(det, image, pinhole, T_CZ) == 0) {
    lz = lz_t{false, T_BC, T_CZ};
    return 0;
  }

  lz = lz_t{true, T_BC, T_CZ};
  return 1;
}

int lz_calc_corners(const lz_detector_t &lz,
                    const pinhole_t &pinhole,
                    const cv::Mat &image,
                    const mat4_t &T_CZ,
                    const int tag_id,
                    const real_t padding,
                    vec2_t &top_left,
                    vec2_t &btm_right) {
  // Tag size and camera intrinsics
  const real_t tag_size = lz.targets.at(tag_id);
  const real_t fx = pinhole.fx;
  const real_t fy = pinhole.fy;
  const real_t cx = pinhole.cx;
  const real_t cy = pinhole.cy;

  // Tag position in camera frame
  const vec3_t r_CZ = tf_trans(T_CZ);
  const real_t x = r_CZ(0);
  const real_t y = r_CZ(1);
  const real_t z = r_CZ(2);

  // Calculate top left and bottom right corners of tag in object frame
  top_left(0) = x - (tag_size / 2.0) - padding;
  top_left(1) = y - (tag_size / 2.0) - padding;
  btm_right(0) = x + (tag_size / 2.0) + padding;
  btm_right(1) = y + (tag_size / 2.0) + padding;

  // Project back to image frame (what it would look like in image)
  top_left(0) = (fx * top_left(0) / z) + cx;
  top_left(1) = (fy * top_left(1) / z) + cy;
  btm_right(0) = (fx * btm_right(0) / z) + cx;
  btm_right(1) = (fy * btm_right(1) / z) + cy;

  // Check corner bounds
  top_left(0) = (top_left(0) > image.cols) ? image.cols : top_left(0);
  top_left(1) = (top_left(1) > image.rows) ? image.rows : top_left(1);
  top_left(0) = (top_left(0) < 0) ? 0 : top_left(0);
  top_left(1) = (top_left(1) < 0) ? 0 : top_left(1);
  btm_right(0) = (btm_right(0) > image.cols) ? image.cols : btm_right(0);
  btm_right(1) = (btm_right(1) > image.rows) ? image.rows : btm_right(1);

  return 0;
}

/*****************************************************************************
 *                              MISSION
 ****************************************************************************/

waypoint_t waypoint_setup(const real_t latitude, const real_t longitude) {
  waypoint_t wp;
  wp.latitude = latitude;
  wp.longitude = longitude;
  return wp;
}

waypoint_t waypoint_setup(const real_t latitude,
                          const real_t longitude,
                          const real_t altitude,
                          const real_t staytime,
                          const real_t heading) {
  waypoint_t wp;
  wp.latitude = latitude;
  wp.longitude = longitude;
  wp.altitude = altitude;
  wp.staytime = staytime;
  wp.heading = heading;
  return wp;
}

real_t waypoint_distance(const waypoint_t &wp_a, const waypoint_t &wp_b) {
  return latlon_dist(wp_a.latitude,
                     wp_a.longitude,
                     wp_b.latitude,
                     wp_b.longitude);
}

std::ostream &operator<<(std::ostream &out, const waypoint_t &wp) {
  out << "latitude: " << wp.latitude << std::endl;
  out << "longitude: " << wp.longitude << std::endl;
  out << "altitude: " << wp.altitude << std::endl;
  out << "staytime: " << wp.staytime << std::endl;
  out << "heading: " << wp.heading;
  return out;
}

int wp_mission_configure(wp_mission_t &m, const std::string &config_file) {
  // Load config
  config_t config{config_file};
  if (config.ok == false) {
    return -1;
  }
  // clang-format off
  std::string wp_type;
  std::vector<real_t> wp_data;
  parse(config, "desired_velocity", m.desired_velocity);
  parse(config, "look_ahead_dist", m.look_ahead_dist);
  parse(config, "threshold_waypoint_gap", m.threshold_waypoint_gap);
  parse(config, "threshold_waypoint_reached", m.threshold_waypoint_reached);
  parse(config, "waypoint_type", wp_type);
  parse(config, "waypoints", wp_data);
  // clang-format on

  // Check number waypoint data
  if (wp_data.size() % 3 != 0) {
    LOG_ERROR("Invalid number of waypoint data!");
    return -1;
  }

  // Load waypoints
  if (wp_type == "GPS" &&
      wp_mission_load_waypoints(m, wp_data, GPS_WAYPOINTS) != 0) {
    LOG_ERROR("Failed to load GPS waypoints!");
    return -1;
  } else if (wp_type == "LOCAL" &&
             wp_mission_load_waypoints(m, wp_data, LOCAL_WAYPOINTS) != 0) {
    LOG_ERROR("Failed to load local waypoints!");
    return -1;
  } else if (wp_type != "GPS" && wp_type != "LOCAL") {
    LOG_ERROR("Invalid waypoint type [%s]!", wp_type.c_str());
    return -1;
  }

  // Update
  m.configured = true;
  return 0;
}

int wp_mission_load_waypoints(wp_mission_t &m,
                              const std::vector<real_t> &wps,
                              const int type) {
  if (type == GPS_WAYPOINTS) {
    // Convert waypoint data into waypoints in the local frame
    for (size_t i = 0; i < wps.size(); i += 3) {
      const real_t lat = wps[i];
      const real_t lon = wps[i + 1];
      const real_t alt = wps[i + 2];

      // Check lat, lon
      if (fltcmp(lat, 0.0) == 0.0 || fltcmp(lon, 0.0) == 0.0) {
        LOG_ERROR(EINVLATLON, lat, lon);
        return -1;
      }

      // Check alt
      if (fltcmp(alt, 0.0) == 0.0) {
        LOG_ERROR(EINVALT, alt);
        return -1;
      }

      m.gps_waypoints.emplace_back(lat, lon, alt);
    }

    // Check waypoints
    if (m.check_waypoints && wp_mission_check_gps_waypoints(m) != 0) {
      return -2;
    }

  } else if (type == LOCAL_WAYPOINTS) {
    // Load local waypoints
    for (size_t i = 0; i < wps.size(); i += 3) {
      const vec3_t wp{wps[i], wps[i + 1], wps[i + 2]};
      std::cout << "Adding local waypoint: " << wp.transpose() << std::endl;
      m.local_waypoints.emplace_back(wp);
    }

    // Set first pair of waypoints
    m.wp_start = m.local_waypoints[0];
    m.wp_end = m.local_waypoints[1];

  } else {
    return -3;
  }

  return 0;
}

int wp_mission_check_gps_waypoints(wp_mission_t &m) {
  // Pre-check
  if (m.gps_waypoints.size() <= 2) {
    return -1;
  }

  // Check waypoint gaps
  vec3_t last_wp = m.gps_waypoints.front();
  for (size_t i = 1; i < m.gps_waypoints.size(); i++) {
    // Calculate distance between current and last waypoint
    vec3_t wp = m.gps_waypoints[i];

    // Check distance
    real_t dist = latlon_dist(last_wp(0), last_wp(1), wp(0), wp(1));
    if (dist > m.threshold_waypoint_gap) {
      LOG_ERROR(EDISTLATLON,
                (int) i + 1,
                wp(0),
                wp(1),
                m.threshold_waypoint_gap);
      return -2;
    }

    // Update last waypoint
    last_wp = wp;
  }

  return 0;
}

int wp_mission_set_gps_homepoint(wp_mission_t &m,
                                 const real_t home_lat,
                                 const real_t home_lon) {
  // Pre-check
  if (m.gps_waypoints.size() == 0) {
    return -1;
  }

  // Convert
  for (auto gps : m.gps_waypoints) {
    // Convert lat lon to local frame
    const real_t lat = gps(0);
    const real_t lon = gps(1);
    const real_t alt = gps(2);
    real_t dist_N, dist_E;
    latlon_diff(home_lat, home_lon, lat, lon, &dist_N, &dist_E);

    // Add to local waypoints in NWU
    const vec3_t nwu{dist_N, -dist_E, alt};
    std::cout << "Adding local waypoint (nwu): " << nwu.transpose();
    std::cout << std::endl;
    m.local_waypoints.push_back(nwu);
  }

  // Set first pair of waypoints
  m.wp_start = m.local_waypoints[0];
  m.wp_end = m.local_waypoints[1];

  return 0;
}

vec3_t wp_mission_closest_point(const wp_mission_t &m, const vec3_t &p_G) {
  // Calculate closest point
  const vec3_t v1 = p_G - m.wp_start;
  const vec3_t v2 = m.wp_end - m.wp_start;
  const real_t t = v1.dot(v2) / v2.squaredNorm();

  // Make sure the point is between wp_start and wp_end
  if (t < 0) {
    return m.wp_start;
  } else if (t > 1) {
    return m.wp_end;
  }

  return m.wp_start + t * v2;
}

int wp_mission_point_line_side(const wp_mission_t &m, const vec3_t &p_G) {
  const vec3_t a = m.wp_start;
  const vec3_t b = m.wp_end;
  const vec3_t c = p_G;
  const real_t s =
      ((b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0)));

  // Position is colinear with waypoint track
  if (fltcmp(s, 0.0) == 0) {
    return 0;
  }

  // Position is left of waypoint track
  if (s > 0.0) {
    return 1;
  }

  // Position is right of waypoint track
  return -1;
}

real_t wp_mission_crosstrack_error(const wp_mission_t &m,
                                   const vec3_t &p_G,
                                   int mode) {
  vec3_t BA = m.wp_start - p_G;
  vec3_t BC = m.wp_start - m.wp_end;

  // Only calculate horizontal crosstrack error by setting z to 0
  if (mode == CTRACK_HORIZ) {
    BA(2) = 0.0;
    BC(2) = 0.0;
  }

  // Crosstrack error
  const real_t error = (BA.cross(BC)).norm() / BC.norm();

  // Check which side the point is on
  const int side = wp_mission_point_line_side(m, p_G);

  return error * side;
}

real_t wp_mission_waypoint_heading(const wp_mission_t &m) {
  const real_t dx = m.wp_end(0) - m.wp_start(0);
  const real_t dy = m.wp_end(1) - m.wp_start(1);

  // Calculate heading
  real_t heading = atan2(dy, dx);
  if (heading > M_PI) {
    heading -= 2 * M_PI;
  } else if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  return heading;
}

vec3_t wp_mission_waypoint_interpolate(const wp_mission_t &m,
                                       const vec3_t &p_G,
                                       const real_t r) {
  // Get closest point
  const vec3_t pt_on_line = wp_mission_closest_point(m, p_G);

  // Calculate waypoint between wp_start and wp_end
  const vec3_t v = m.wp_end - m.wp_start;
  const vec3_t u = v / v.norm();
  return pt_on_line + r * u;
}

int wp_mission_waypoint_reached(const wp_mission_t &m, const vec3_t &p_G) {
  // Pre-check
  if (m.configured == false) {
    return -1;
  }

  // Calculate distance to waypoint
  const vec3_t x = m.wp_end - p_G;
  const real_t dist = x.norm();

  // Waypoint reached?
  if (dist > m.threshold_waypoint_reached) {
    return 0;
  } else {
    LOG_INFO("Waypoint (%f, %f, %f) reached!",
             m.wp_end(0),
             m.wp_end(1),
             m.wp_end(2));
    return 1;
  }
}

int wp_mission_update(wp_mission_t &m, const vec3_t &p_G, vec3_t &waypoint) {
  // Pre-check
  if (m.configured == false) {
    return -1;
  } else if (m.local_waypoints.size() == 0) {
    return -3;
  }

  // Interpolate new waypoint
  if (m.completed != true) {
    waypoint = wp_mission_waypoint_interpolate(m, p_G, m.look_ahead_dist);
  }

  // Waypoint reached? get new wp_start and wp_end
  if (wp_mission_waypoint_reached(m, waypoint)) {
    if ((m.waypoint_index + 2) == (int) m.local_waypoints.size()) {
      m.completed = true;
      m.waypoint_index = 0;
      return -2;

    } else {
      m.wp_start = m.local_waypoints[m.waypoint_index + 1];
      m.wp_end = m.local_waypoints[m.waypoint_index + 2];
      m.waypoint_index++;
    }
  }

  return 0;
}

/*****************************************************************************
 *                                 ATL
 ****************************************************************************/

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

vec4_t atl_step_hover_mode(atl_t &atl, const mat4_t &T_WB, const real_t dt) {
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
                              const real_t dt) {
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
                              const real_t dt) {
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

// vec4_t atl_step_landing_mode(atl_t &atl, const real_t dt) {
//   // // land on target
//   // atl.landing_controller.update(atl.landing_target.position_B,
//   //                               atl.velocity,
//   //                               atl.yaw_setpoint,
//   //                               dt);
//   const vec3_t r_WB = tf_trans(atl.T_WB);
//   const real_t yaw = quat2euler(tf_quat(atl.T_WB))(2);
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

int atl_step(atl_t &atl,
             const mat4_t &T_WB,
             const lz_t &lz,
             const real_t dt,
             vec4_t &u) {
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
