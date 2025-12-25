#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "sim/SimCalibCameraImu.hpp"

namespace xyz {

TEST(SimCalibCameraImu, construct) {
  SimCalibCameraImu sim;

  std::vector<Vec3> pose_points;
  std::vector<Vec3> pose_colors;
  std::vector<double> pose_radii;
  for (const auto &[ts, pose] : sim.camera_poses) {
    pose_points.push_back(tf_trans(pose));
    pose_colors.emplace_back(255.0, 0.0, 0.0);
    pose_radii.emplace_back(0.01);
  }
  Logger log;
  log.log_poses("/world/camera_poses", sim.camera_poses, 0.1);
  log.log_target("/world/calib_target",
                 sim.target_configs.at(0),
                 sim.target_poses.at(0));
  // log.log_points("/world/candidate_camera_poses",
  //                pose_points,
  //                pose_colors,
  //                pose_radii);

  // sim.save("/tmp/sim_calib_camera_imu");
}

} // namespace xyz
