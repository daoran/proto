#pragma once
#include "../core/Core.hpp"
#include "../calib/calib.hpp"
#include "../camera/camera.hpp"
#include "../timeline/timeline.hpp"

namespace cartesian {

using AprilGridMap = std::map<int, std::shared_ptr<AprilGrid>>;
using CameraMap = std::map<int, AprilGridMap>;

struct SimCalib {
  // Settings
  const double camera_rate; // [Hz]
  const double sample_x;
  const double sample_y;
  const double sample_z;
  const double sample_z_offset;
  const int sample_num_x;
  const int sample_num_y;
  const int sample_num_z;

  // Data
  std::map<int, AprilGridConfig> target_configs;
  std::map<int, Mat4> target_poses;
  std::map<int, CameraGeometry> cameras;
  std::map<timestamp_t, Mat4> camera_poses;
  std::map<timestamp_t, CameraMap> camera_views;

  /** Constructor / destructor */
  SimCalib() = delete;
  SimCalib(const double camera_rate_ = 10.0,
           const double sample_x_ = 1.0,
           const double sample_y_ = 1.0,
           const double sample_z_ = 0.3,
           const double sample_z_offset_ = 1.0,
           const int sample_num_x_ = 5,
           const int sample_num_y_ = 7,
           const int sample_num_z_ = 3);
  virtual ~SimCalib() = default;

  /** Setup */
  void setup_calib_targets();
  void setup_camera_geometries();
  void setup_camera_poses();

  /** Simulate */
  void simulate_camera_views();

  /** Form timeline */
  Timeline get_timeline() const;

  /** Save */
  int save_target_configs(const fs::path &yaml_path) const;
  int save_target_poses(const fs::path &csv_path) const;
  int save_camera_geometries(const fs::path &yaml_path) const;
  int save_camera_poses(const fs::path &csv_path) const;
  int save_camera_views(const fs::path &save_dir) const;
  int save(const fs::path &save_dir) const;
};

} // namespace cartesian
