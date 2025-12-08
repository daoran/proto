#pragma once
#include "../Core.hpp"
#include "../calib/calib.hpp"
#include "../camera/camera.hpp"
#include "../timeline/timeline.hpp"

namespace xyz {

using AprilGridMap = std::map<int, std::shared_ptr<AprilGrid>>;
using CameraMap = std::map<int, AprilGridMap>;

struct SimCalibCamera {
  // Settings
  const double camera_rate = 10.0; // [Hz]
  const double sample_x = 1.0;
  const double sample_y = 1.0;
  const double sample_z = 0.3;
  const double sample_z_offset = 1.0;
  const int sample_num_x = 5;
  const int sample_num_y = 7;
  const int sample_num_z = 3;

  // Data
  std::map<int, AprilGridConfig> target_configs;
  std::map<int, Mat4> target_poses;
  std::map<int, CameraGeometry> cameras;
  std::map<timestamp_t, Mat4> camera_poses;
  std::map<timestamp_t, CameraMap> camera_views;

  /** Constructor / destructor */
  SimCalibCamera();
  virtual ~SimCalibCamera() = default;

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

} // namespace xyz
