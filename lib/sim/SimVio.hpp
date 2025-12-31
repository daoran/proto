#pragma once
#include "../core/Core.hpp"
#include "../timeline/timeline.hpp"

namespace cartesian {

struct SimVio {
  // Settings
  double sensor_velocity = 0.3;
  double cam_rate = 30;
  double imu_rate = 400;

  // Scene data
  Vec3s features;

  // Camera data
  timestamps_t cam_ts;
  Vec3s cam_pos_gnd;
  Quats cam_rot_gnd;
  Mat4s cam_poses_gnd;
  Vec3s cam_pos;
  Quats cam_rot;
  Mat4s cam_poses;
  std::vector<std::vector<size_t>> observations;
  std::vector<Vec2s> keypoints;

  // IMU data
  timestamps_t imu_ts;
  Vec3s imu_acc;
  Vec3s imu_gyr;
  Vec3s imu_pos;
  Quats imu_rot;
  Mat4s imu_poses;
  Vec3s imu_vel;

  // Simulation timeline
  Timeline timeline;

  // Sim
  SimVio(const double circle_r = 5.0);
  virtual ~SimVio() = default;

  // Save
  void save(const std::string &dir);
};

} // namespace cartesian
