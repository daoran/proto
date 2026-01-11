#pragma once

#include <string>
#include <libgen.h>
#include <inttypes.h>

#include "../core/Core.hpp"
#include "../timeline/timeline.hpp"

namespace cartesian {

/**
 * Euroc IMU data
 */
struct EurocImu {
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  Vec3s w_B;
  Vec3s a_B;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  Mat4 T_BS = I(4);
  double rate_hz = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;

  EurocImu() = default;
  EurocImu(const std::string &data_dir_);
  ~EurocImu() = default;
  void print() const;
};

/**
 * Euroc camera data
 */
struct EurocCamera {
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  std::vector<std::string> image_paths;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  Mat4 T_BS = I(4);
  double rate_hz = 0.0;
  Vec2i resolution;
  std::string camera_model;
  Vec4 intrinsics = zeros(4, 1);
  std::string distortion_model;
  Vec4 distortion_coefficients = zeros(4, 1);

  EurocCamera() = default;
  EurocCamera(const std::string &data_dir_, bool is_calib_data = false);
  ~EurocCamera() = default;
  void print() const;
};

/**
 * Euroc ground truth
 */
struct EurocGroundTruth {
  bool ok = false;

  // Data
  std::string data_dir;
  timestamps_t timestamps;
  Vec3s p_RS_R;
  Vec4s q_RS;
  Vec3s v_RS_R;
  Vec3s b_w_RS_S;
  Vec3s b_a_RS_S;

  EurocGroundTruth() = default;
  EurocGroundTruth(const std::string &data_dir_);
  ~EurocGroundTruth() = default;
};

/**
 * Euroc data
 */
struct EurocData {
  bool ok = false;
  std::string data_path;

  EurocImu imu_data;
  EurocCamera cam0_data;
  EurocCamera cam1_data;
  EurocGroundTruth ground_truth;
  cv::Size image_size;

  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  timestamp_t ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::map<timestamp_t, double> time;
  Timeline timeline;

  EurocData() = default;
  EurocData(const std::string &data_path);
  ~EurocData() = default;

  void reset();
  timestamp_t min_timestamp() const;
  timestamp_t max_timestamp() const;
};

/**
 * Euroc calibration target
 */
struct EurocTarget {
  bool ok = false;
  std::string file_path;

  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  EurocTarget() = default;
  EurocTarget(const std::string &target_file);
  ~EurocTarget() = default;

  void print() const;
};

/**
 * Euroc calibration data
 */
struct EurocCalib {
  bool ok = false;

  // Settings
  std::string data_path;
  bool imshow = false;

  // Data
  EurocImu imu_data;
  EurocCamera cam0_data;
  EurocCamera cam1_data;
  EurocTarget calib_target;
  cv::Size image_size;

  EurocCalib(const std::string &data_path);
  ~EurocCalib() = default;
  Timeline timeline();
};

} // namespace yac
