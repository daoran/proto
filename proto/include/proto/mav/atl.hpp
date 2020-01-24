#ifndef PROTO_MAV_ATL_HPP
#define PROTO_MAV_ATL_HPP

// Order matters with the AprilTags lib. The detector has to be first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include "proto/core/core.hpp"
#include "proto/control/pid.hpp"
#include "proto/vision/camera/pinhole.hpp"

namespace proto {

/*****************************************************************************
 *                          ATTITUDE CONTROLLER
 ****************************************************************************/

/**
 * Attitude controller
 */
struct att_ctrl_t {
  bool ok = false;

  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double max_thrust = 5.0;

  pid_t roll_ctrl;
  pid_t pitch_ctrl;
  pid_t yaw_ctrl;
};

/**
 * Configure attitude controller with `config_file`.
 * @returns 0 or -1 for success or failure
 */
int att_ctrl_configure(att_ctrl_t &ctrl,
                       const std::string &config_file,
                       const std::string &prefix = "");

/**
 * Update attitude controller with `setpoints` (roll, pitch, yaw, z), actual
 * pose of the robot `T_WB` and time step `dt` in seconds [s].
 * @returns Motor command (m1, m2, m3, m4)
 */
vec4_t att_ctrl_update(att_ctrl_t &ctrl,
                       const vec4_t &setpoints,
                       const mat4_t &T_WB,
                       const double dt);

/**
 * Reset position control
 */
void att_ctrl_reset(att_ctrl_t &ctrl);

/*****************************************************************************
 *                          POSITION CONTROLLER
 ****************************************************************************/

/**
 * Position control
 */
struct pos_ctrl_t {
  bool ok = false;

  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double hover_throttle = 0.5;

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  pid_t x_ctrl{0.5, 0.0, 0.35};
  pid_t y_ctrl{0.5, 0.0, 0.35};
  pid_t z_ctrl{0.3, 0.0, 0.1};
};

/**
 * Configure position control
 *
 * @returns 0 or -1 for success or failure
 */
int pos_ctrl_configure(pos_ctrl_t &ctrl,
                       const std::string &config_file,
                       const std::string &prefix = "");

/**
 * Update position control
 *
 * - `setpoints`: Setpoints (x, y, z)
 * - `actual`: Actual Pose T_WB
 * - `yaw`: [rads]
 * - `dt`: Time step [s]
 *
 * @returns Attitude command (roll, pitch, yaw, thrust)
 */
vec4_t pos_ctrl_update(pos_ctrl_t &ctrl,
                       const vec3_t &setpoints,
                       const mat4_t &T_WB,
                       const double yaw_setpoint,
                       const double dt);

/**
 * Reset position control
 */
void pos_ctrl_reset(pos_ctrl_t &ctrl);

/*****************************************************************************
 *                          TRACKING CONTROLLER
 ****************************************************************************/

/**
 * Tracking control
 */
struct tk_ctrl_t {
  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double hover_throttle = 0.5;

  pid_t x_ctrl;
  pid_t y_ctrl;
  pid_t z_ctrl;
};

/**
 * Configure tracking control
 *
 * @returns 0 or -1 for success or failure
 */
int tk_ctrl_configure(tk_ctrl_t &ctrl,
                      const std::string &config_file,
                      const std::string &prefix = "");

/**
 * Update tracking control
 *
 * - `setpoints`: Setpoints (x, y, z)
 * - `actual`: Actual Pose T_WB
 * - `dt`: Time step [s]
 *
 * @returns Attitude command (roll, pitch, yaw, thrust)
 */
vec4_t tk_ctrl_update(tk_ctrl_t &ctrl,
                      const mat4_t &T_WZ,
                      const mat4_t &T_WB,
                      const double desired_height,
                      const double desired_yaw,
                      const double dt);

/**
 * Reset tk control
 */
void tk_ctrl_reset(tk_ctrl_t &ctrl);

// /*****************************************************************************
//  *                          WAYPOINT CONTROLLER
//  ****************************************************************************/
//
// /**
//  * Waypoint Control
//  */
// struct wp_ctrl_t {
//   bool configured = false;
//   double dt = 0.0;
//
//   pid_t at_controller{0.5, 0.0, 0.035};
//   pid_t ct_controller{0.5, 0.0, 0.035};
//   pid_t z_controller{0.3, 0.0, 0.1};
//   pid_t yaw_controller{2.0, 0.0, 0.1};
//
//   double roll_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
//   double pitch_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
//   double hover_throttle = 0.5;
//
//   vec3_t setpoints{0.0, 0.0, 0.0};
//   vec4_t outputs{0.0, 0.0, 0.0, 0.0};
// };
//
// /**
//  * Configure waypoint control
//  *
//  * @param[in,out] wc Waypoint controller
//  * @param[in] config_file Path to config file
//  * @return
//  *    - 0: Success
//  *    - -1: Failed to load config file
//  *    - -2: Failed to load mission file
//  */
// int wp_ctrl_configure(wp_ctrl_t &wc, const std::string &config_file);
//
// /**
//  * Update controller
//  *
//  * @param[in,out] wc Waypoint controller
//  * @param[in,out] m Mission
//  * @param[in] p_G Actual position in global frame
//  * @param[in] v_G Actual velocity in global frame
//  * @param[in] rpy_G Actual roll, pitch and yaw in global frame
//  * @param[in] dt Time difference in seconds
//  *
//  * @return
//  *   - 0: Success
//  *   - -1: Not configured
//  *   - -2: No more waypoints
//  */
// int wp_ctrl_update(wp_ctrl_t &wc,
//                    wp_mission_t &m,
//                    const vec3_t &p_G,
//                    const vec3_t &v_G,
//                    const vec3_t &rpy_G,
//                    const double dt);
//
// /**
//  * Reset controller errors to 0
//  */
// void wp_ctrl_reset(wp_ctrl_t &wc);

/*****************************************************************************
 *                            LANDING ZONE
 ****************************************************************************/

/**
 * Landing zone
 */
struct lz_t {
  bool detected = false;
  mat4_t T_BC = I(4);
  mat4_t T_CZ = I(4);

  lz_t() {}
  ~lz_t() {}
  lz_t(const bool detected_, const mat4_t &T_BC_, const mat4_t &T_CZ_)
      : detected{detected_}, T_BC{T_BC_}, T_CZ{T_CZ_} {}
};

/**
 * Landing zone detector
 */
struct lz_detector_t {
  bool ok = false;

  AprilTags::TagDetector *det = nullptr;
  std::map<int, double> targets;

  lz_detector_t();
  lz_detector_t(const std::vector<int> &tag_ids,
                const std::vector<double> &tag_sizes);
  lz_detector_t(const std::string &config, const std::string &prefix = "");
  ~lz_detector_t();
};

/**
 * Print landing zone
 */
void lz_print(const lz_t &lz);

/**
 * Configure landing zone
 */
int lz_detector_configure(lz_detector_t &lz,
                          const std::vector<int> &tag_ids,
                          const std::vector<double> &tag_sizes);

/**
 * Configure landing zone
 */
int lz_detector_configure(lz_detector_t &lz,
                          const std::string &config_file,
                          const std::string &prefix = "");

/**
 * Detect landing zone
 */
int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       mat4_t &T_CZ);

/**
 * Detect landing zone
 */
int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       const mat4_t &T_BC,
                       lz_t &lz);

/**
 * Calculate landing zone corners in pixels.
 */
int lz_detector_calc_corners(const lz_detector_t &lz,
                             const pinhole_t &pinhole,
                             const cv::Mat &image,
                             const mat4_t &T_CZ,
                             const int tag_id,
                             const double padding,
                             vec2_t &top_left,
                             vec2_t &btm_right);

/*****************************************************************************
 *                                 ATL
 ****************************************************************************/

#define DISARM_MODE -1
#define IDLE_MODE 0
#define HOVER_MODE 1
#define DISCOVER_MODE 2
#define TRACKING_MODE 3
#define LANDING_MODE 4
#define WAYPOINT_MODE 5

struct atl_t {
  bool ok = false;
  int mode = DISCOVER_MODE;

  double recover_height = 0.0;
  bool auto_track = false;
  bool auto_land = false;
  bool auto_disarm = false;
  double target_lost_threshold = 0.5;
  double min_discover_time = FLT_MAX;
  double min_tracking_time = FLT_MAX;
  vec2_t home_point{0.0, 0.0};

  pinhole_t cam0_pinhole;
  mat4_t T_BC0 = I(4);

  att_ctrl_t att_ctrl;
  pos_ctrl_t pos_ctrl;
  tk_ctrl_t tk_ctrl;
  lz_detector_t lz_detector;

  vec3_t position_setpoint{0.0, 0.0, 0.0};
  double yaw_setpoint = 0.0;
  lz_t lz;

  struct timespec discover_tic;
  struct timespec tracking_tic;
  struct timespec landing_tic;
  struct timespec waypoint_tic;

  atl_t();
  atl_t(const std::string &config_file);
};

int atl_configure(atl_t &atl, const std::string &config_file);
int atl_detect_lz(atl_t &atl, const cv::Mat &image);
vec4_t atl_step_hover_mode(atl_t &atl, const mat4_t &T_WB, const double dt);
vec4_t atl_step_discover_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const double dt);
vec4_t atl_step_tracking_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const double dt);
vec4_t atl_step_landing_mode(atl_t &atl, const double dt);
int atl_step(
    atl_t &atl, const mat4_t T_WB, const lz_t &lz, const double dt, vec4_t &u);

} //  namespace proto
#endif // PROTO_MAV_ATL_HPP
