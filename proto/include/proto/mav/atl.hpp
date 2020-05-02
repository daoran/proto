#ifndef PROTO_MAV_ATL_HPP
#define PROTO_MAV_ATL_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

// Order matters with the AprilTags lib. The detector has to be first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include "proto/core/core.hpp"

namespace proto {

/*****************************************************************************
 *                          ATTITUDE CONTROLLER
 ****************************************************************************/

/**
 * Attitude controller
 */
struct att_ctrl_t {
  bool ok = false;

  real_t dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  real_t max_thrust = 5.0;

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
                       const real_t dt);

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

  real_t dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  real_t hover_throttle = 0.5;

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
                       const real_t yaw_setpoint,
                       const real_t dt);

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
  real_t dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  real_t hover_throttle = 0.5;

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
                      const real_t desired_height,
                      const real_t desired_yaw,
                      const real_t dt);

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
//   real_t dt = 0.0;
//
//   pid_t at_controller{0.5, 0.0, 0.035};
//   pid_t ct_controller{0.5, 0.0, 0.035};
//   pid_t z_controller{0.3, 0.0, 0.1};
//   pid_t yaw_controller{2.0, 0.0, 0.1};
//
//   real_t roll_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
//   real_t pitch_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
//   real_t hover_throttle = 0.5;
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
//                    const real_t dt);
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
  std::map<int, real_t> targets;

  lz_detector_t();
  lz_detector_t(const std::vector<int> &tag_ids,
                const std::vector<real_t> &tag_sizes);
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
                          const std::vector<real_t> &tag_sizes);

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
                             const real_t padding,
                             vec2_t &top_left,
                             vec2_t &btm_right);

/*****************************************************************************
 *                              MISSION
 ****************************************************************************/

// ERROR MESSAGES
#define EDISTLATLON "Waypoint %d: (%f, %f) has dist > %f from prev waypoint!"
#define EINVLATLON "Invalid latlon (%f, %f)!"
#define EINVALT "Invalid altitude %f!"
#define EINVSTAY "Invalid staytime %f!"
#define EINVHEADING                                                            \
  "Invalid heading %f! Should be between -180.0 to 180.0 degrees"

// CONSTANTS
#define GPS_WAYPOINTS 0
#define LOCAL_WAYPOINTS 1
#define CTRACK_HORIZ 0
#define CTRACK_3D 1

/**
 * Waypoint
 */
struct waypoint_t {
  real_t latitude = 0.0;
  real_t longitude = 0.0;
  real_t altitude = 0.0;
  real_t staytime = 0.0;
  real_t heading = 0.0;
};

/**
 * Setup waypoint with `latitude` and `longitude`.
 *
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const real_t latitude, const real_t longitude);

/**
 * Setup waypoint.
 *
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const real_t latitude,
                          const real_t longitude,
                          const real_t altitude,
                          const real_t staytime,
                          const real_t heading);

/**
 * Calculate distance between waypoint `wp_a` and `wp_b` in [m].
 * @returns Distance between waypoint `wp_a` and waypoint `wp_b`
 */
real_t waypoint_distance(const waypoint_t &wp_a, const waypoint_t &wp_b);

/**
 * `waypoint_t` to output stream
 */
std::ostream &operator<<(std::ostream &out, const waypoint_t &wp);

/**
 * Mission
 */
struct wp_mission_t {
  bool configured = false;
  bool completed = false;

  real_t home_lat = 0.0;
  real_t home_lon = 0.0;
  real_t home_alt = 0.0;

  bool check_waypoints = true;
  real_t threshold_waypoint_gap = 20.0;
  real_t threshold_waypoint_reached = 0.2;
  real_t desired_velocity = 0.5;
  real_t look_ahead_dist = 0.5;

  vec3s_t gps_waypoints;
  vec3s_t local_waypoints;
  int waypoint_index = 0;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
};

/**
 * Configure mission with configuration file `config_file`.
 *
 * @returns 0 for success, -1 for failure to load / parse config file, and -2
 * for invalid GPS waypoints
 */
int wp_mission_configure(wp_mission_t &m, const std::string &config_file);

/**
 * Load GPS waypoints with the list of waypoints `wps`, where waypoint type is
 * denoted by `type`.
 *
 * @returns 0 for success, -1 for failure
 */
int wp_mission_load_waypoints(wp_mission_t &m,
                              const std::vector<real_t> &wps,
                              const int type);

/**
 * Check GPS waypoints to make sure the distance between waypoints does not
 * exceed `mission_t.waypoint_threshold`.
 *
 * @returns 0 for success, -1 for no waypoints loaded, and finally -2 for
 * invalid GPS waypoints.
 */
int wp_mission_check_gps_waypoints(wp_mission_t &m);

/**
 * Set GPS home point latitude `home_lat`, longitude `home_lon`, and calculate
 * local waypoints by converting GPS to local waypoints.
 *
 * @returns 0 for success, -1 for failure.
 */
int wp_mission_set_gps_homepoint(wp_mission_t &m,
                                 const real_t home_lat,
                                 const real_t home_lon);

/**
 * Calculate closest point.
 *
 * @returns Closest point
 */
vec3_t wp_mission_closest_point(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate which side the point is compared to waypoint track.
 *
 * @returns 0 if position is colinear with waypoint track, 1 if position is
 * left of waypoint track, and -1 if position is right of waypoint track.
 */
int wp_mission_point_line_side(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate crosstrack error.
 *
 * @returns Cross track error.
 */
real_t wp_mission_crosstrack_error(const wp_mission_t &m,
                                   const vec3_t &p_G,
                                   int mode = CTRACK_HORIZ);

/**
 * Calculate waypoint yaw. This function assumes we are operating in the NWU
 * frame, where a 0 heading starts from the x-axis and goes counter-clock-wise.
 *
 * @returns Waypoint yaw
 */
real_t wp_mission_waypoint_heading(const wp_mission_t &m);

/**
 * Interpolate mission waypoint point.
 *
 * @returns Interpolated mission waypoint.
 */
vec3_t wp_mission_waypoint_interpolate(const wp_mission_t &m,
                                       const vec3_t &p_G,
                                       const real_t r);

/**
 * Check whether waypoint is reached.
 *
 * @returns 0 if waypoint not reached, 1 if waypoint is reached, and -1 if
 * mission is not configured.
 */
int wp_mission_waypoint_reached(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Update mission with current position in the global frame `p_G` and output
 * interpolated `waypoint`.
 *
 * @returns 0 for success, -1 if mission is not configured, and -2 for no more
 * waypoints.
 */
int wp_mission_update(wp_mission_t &m, const vec3_t &p_G, vec3_t &waypoint);

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

  real_t recover_height = 0.0;
  bool auto_track = false;
  bool auto_land = false;
  bool auto_disarm = false;
  real_t target_lost_threshold = 0.5;
  real_t min_discover_time = FLT_MAX;
  real_t min_tracking_time = FLT_MAX;
  vec2_t home_point{0.0, 0.0};

  pinhole_t cam0_pinhole;
  mat4_t T_BC0 = I(4);

  att_ctrl_t att_ctrl;
  pos_ctrl_t pos_ctrl;
  tk_ctrl_t tk_ctrl;
  lz_detector_t lz_detector;

  vec3_t position_setpoint{0.0, 0.0, 0.0};
  real_t yaw_setpoint = 0.0;
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
vec4_t atl_step_hover_mode(atl_t &atl, const mat4_t &T_WB, const real_t dt);
vec4_t atl_step_discover_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const real_t dt);
vec4_t atl_step_tracking_mode(atl_t &atl,
                              const mat4_t &T_WB,
                              const lz_t &lz,
                              const real_t dt);
vec4_t atl_step_landing_mode(atl_t &atl, const real_t dt);
int atl_step(atl_t &atl,
             const mat4_t &T_WB,
             const lz_t &lz,
             const real_t dt,
             vec4_t &u);

} //  namespace proto
#endif // PROTO_MAV_ATL_HPP
