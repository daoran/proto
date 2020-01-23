#ifndef PROTO_MAV_ATL_HPP
#define PROTO_MAV_ATL_HPP

#include "proto/core/core.hpp"
#include "proto/control/pid.hpp"
#include "proto/mav/lz.hpp"
#include "proto/mav/tk_ctrl.hpp"
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
