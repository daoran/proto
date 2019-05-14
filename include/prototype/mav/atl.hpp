#ifndef PROTOTYPE_MAV_ATL_HPP
#define PROTOTYPE_MAV_ATL_HPP

#include "prototype/core/core.hpp"
#include "prototype/mav/lz.hpp"
#include "prototype/mav/att_ctrl.hpp"
#include "prototype/mav/pos_ctrl.hpp"
#include "prototype/mav/tk_ctrl.hpp"
#include "prototype/vision/camera/pinhole.hpp"

namespace proto {

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

  struct timespec discover_tic = {0, 0};
  struct timespec tracking_tic = {0, 0};
  struct timespec landing_tic = {0, 0};
  struct timespec waypoint_tic = {0, 0};

  atl_t();
  atl_t(const std::string &config_file);
};

int atl_configure(atl_t &atl, const std::string &config_file);
int atl_detect_lz(atl_t &atl, const cv::Mat &image);
vec4_t atl_step_hover_mode(atl_t &atl, const mat4_t &T_WB, const double dt);
vec4_t atl_step_discover_mode(atl_t &atl, const mat4_t &T_WB, const lz_t &lz, const double dt);
vec4_t atl_step_tracking_mode(atl_t &atl, const mat4_t &T_WB, const lz_t &lz, const double dt);
vec4_t atl_step_landing_mode(atl_t &atl, const double dt);
int atl_step(atl_t &atl, const mat4_t T_WB, const lz_t &lz, const double dt, vec4_t &u);

} //  namespace proto
#endif // PROTOTYPE_MAV_ATL_HPP
