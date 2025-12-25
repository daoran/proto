#include <gtest/gtest.h>

#include "LissajousTrajectory.hpp"
#include "AprilGridConfig.hpp"
#include "../core/Logger.hpp"

namespace xyz {

static AprilGridConfig setup_target_config() {
  const int target_id = 0;
  const int tag_rows = 8;
  const int tag_cols = 10;
  const double tag_size = 0.08;
  const double tag_spacing = 0.3;
  const int tag_id_offset = 0;
  const AprilGridConfig target_config{
      target_id,
      tag_rows,
      tag_cols,
      tag_size,
      tag_spacing,
      tag_id_offset,
  };
  return target_config;
}

static Mat4 setup_target_pose(const AprilGridConfig &config) {
  const auto target_center = config.getCenter();
  const Vec3 target_pos{1.0, target_center.x(), 0.0};
  const Vec3 target_euler{M_PI / 2.0, 0.0, -M_PI / 2.0};
  const Mat3 target_rot = euler321(target_euler);
  const Mat4 T_WTj = tf(target_rot, target_pos);
  return T_WTj;
}

static Mat4 setup_target_center_pose(const AprilGridConfig &config) {
  const double calib_width = config.getWidthHeight().x();
  const double calib_height = config.getWidthHeight().y();
  const Mat3 C_TO = I(3);
  const Vec3 r_TO{calib_width / 2.0, calib_height / 2.0, 1.0};
  const Mat4 T_TO = tf(C_TO, r_TO);
  return T_TO;
}

TEST(LissajousTrajectory, construct) {
  // Setup
  const std::string traj_type = "fig8-horiz";
  const AprilGridConfig config = setup_target_config();
  const Mat4 &T_WT = setup_target_pose(config);
  const Mat4 &T_TO = setup_target_center_pose(config);
  const timestamp_t ts_start = 0;
  const double calib_width = config.getWidthHeight().x();
  const double calib_height = config.getWidthHeight().y();
  const double R = 0.5;
  const double T = 1.0;
  LissajousTrajectory
      traj{traj_type, ts_start, T_WT, T_TO, calib_width, calib_height, R, T};

  // Simulate
  Logger log;
  log.init_series_line("vel/x", Vec3{255.0, 0.0, 0.0}, 1.0f);
  log.init_series_line("vel/y", Vec3{0.0, 255.0, 0.0}, 1.0f);
  log.init_series_line("vel/z", Vec3{0.0, 0.0, 255.0}, 1.0f);
  log.init_series_line("acc/x", Vec3{255.0, 0.0, 0.0}, 1.0f);
  log.init_series_line("acc/y", Vec3{0.0, 255.0, 0.0}, 1.0f);
  log.init_series_line("acc/z", Vec3{0.0, 0.0, 255.0}, 1.0f);
  log.init_series_line("angvel/x", Vec3{255.0, 0.0, 0.0}, 1.0f);
  log.init_series_line("angvel/y", Vec3{0.0, 255.0, 0.0}, 1.0f);
  log.init_series_line("angvel/z", Vec3{0.0, 0.0, 255.0}, 1.0f);

  double time = 0;
  const double dt = 0.01;
  std::map<timestamp_t, Mat4> poses;

  while (time <= T) {
    const timestamp_t ts = sec2ts(time);
    poses[ts] = traj.get_pose(ts);
    const Vec3 v_WS = traj.get_velocity(ts);
    const Vec3 a_WS = traj.get_acceleration(ts);
    const Vec3 w_WS = traj.get_angular_velocity(ts);

    log.log_scalar("vel/x", ts, v_WS.x());
    log.log_scalar("vel/y", ts, v_WS.y());
    log.log_scalar("vel/z", ts, v_WS.z());

    log.log_scalar("acc/x", ts, a_WS.x());
    log.log_scalar("acc/y", ts, a_WS.y());
    log.log_scalar("acc/z", ts, a_WS.z());

    log.log_scalar("angvel/x", ts, w_WS.x());
    log.log_scalar("angvel/y", ts, w_WS.y());
    log.log_scalar("angvel/z", ts, w_WS.z());

    time += dt;
  }

  // Log poses, trajectory and target
  log.log_poses("pose", poses, 0.1);
  log.log_trajectory("trajectory", poses);
  log.log_target("calib_target", config, T_WT);
}

} // namespace xyz
