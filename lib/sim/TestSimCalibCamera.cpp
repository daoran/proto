#include <gtest/gtest.h>
#include "sim/SimCalibCamera.hpp"
#include "Logger.hpp"

namespace xyz {

TEST(SimCalibCamera, construct) {
  SimCalibCamera sim;

  // Logger log;
  // auto config = sim.target_configs.at(0);
  // const auto num_tags = config.tag_rows * config.tag_cols;
  // auto T_WF = sim.target_poses.at(0);
  //
  // std::vector<Vec3> points_data;
  // std::vector<Vec3> points_colors;
  // std::vector<double> points_radii;
  // for (int tag_id = 0; tag_id < num_tags; ++tag_id) {
  //   for (int corner_index = 0; corner_index < 4; ++corner_index) {
  //     const Vec3 p_F = config.getObjectPoint(tag_id, corner_index);
  //     const Vec3 p_W = tf_point(T_WF, p_F);
  //     points_data.push_back(p_W);
  //     points_colors.emplace_back(255.0, 0.0, 0.0);
  //     points_radii.emplace_back(0.01);
  //   }
  // }
  // log.log_pose("/world/calib_target", T_WF, config.tag_size);
  // log.log_points("/world/calib_points",
  //                points_data,
  //                points_colors,
  //                points_radii);
  //
  // std::vector<Vec3> pose_points;
  // std::vector<Vec3> pose_colors;
  // std::vector<double> pose_radii;
  // for (const auto &[ts, pose] : sim.camera_poses) {
  //   pose_points.push_back(tf_trans(pose));
  //   pose_colors.emplace_back(255.0, 0.0, 0.0);
  //   pose_radii.emplace_back(0.01);
  //   log.log_pose("/world/camera_poses", ts, pose, 0.1);
  // }
  // log.log_points("/world/candidate_camera_poses", pose_points, pose_colors, pose_radii);

  sim.save("/tmp/sim_calib_camera");
}

} // namespace xyz
