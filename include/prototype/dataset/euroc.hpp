#ifndef PROTOTYPE_DATASET_EUROC_HPP
#define PROTOTYPE_DATASET_EUROC_HPP

#include <string>
#include <libgen.h>

#include "prototype/core/core.hpp"
#include "prototype/dataset/timeline.hpp"

namespace prototype {

/*****************************************************************************
 * euroc_imu_t
 ****************************************************************************/

struct euroc_imu_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> w_B;
  std::vector<vec3_t> a_B;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;

  euroc_imu_t();
  euroc_imu_t(const std::string &data_dir_);
  ~euroc_imu_t();
};

/**
 * Load IMU data
 *
 * @param[in] data IMU data
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int euroc_imu_load(euroc_imu_t &data, const std::string &data_dir);

/**
 * `euroc_imu_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_imu_t &data);

/*****************************************************************************
 * euroc_camera_t
 ****************************************************************************/

struct euroc_camera_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<std::string> image_paths;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  vec2_t resolution = zeros(2, 1);
  std::string camera_model;
  vec4_t intrinsics = zeros(4, 1);
  std::string distortion_model;
  vec4_t distortion_coefficients = zeros(4, 1);

  euroc_camera_t();
  euroc_camera_t(const std::string &data_dir_);
  ~euroc_camera_t();
};

/**
 * Load Camera data
 *
 * @param[in,out] cd Camera data
 * @param[in] data_dir Data directory
 * @param[in] is_calib_data Is camera data for calibration?
 * @returns 0 for success, -1 for failure
 */
int euroc_camera_load(euroc_camera_t &cd,
                      const std::string &data_dir,
                      const bool is_calib_data = false);

/**
 * `euroc_camera_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_camera_t &data);

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

struct euroc_ground_truth_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> p_RS_R;
  std::vector<vec4_t> q_RS;
  std::vector<vec3_t> v_RS_R;
  std::vector<vec3_t> b_w_RS_S;
  std::vector<vec3_t> b_a_RS_S;

  euroc_ground_truth_t();
  euroc_ground_truth_t(const std::string data_dir_);
  ~euroc_ground_truth_t();
};

/**
 * Load ground truth data
 *
 * @param[in,out] data Ground truth data
 * @returns 0 for success, -1 for failure
 */
int euroc_ground_truth_load(euroc_ground_truth_t &data,
                            const std::string &data_dir);

/*****************************************************************************
 * euroc_data_t
 ****************************************************************************/

struct euroc_data_t {
  bool ok = false;
  std::string data_path;

  euroc_imu_t imu_data;
  euroc_camera_t cam0_data;
  euroc_camera_t cam1_data;
  euroc_ground_truth_t ground_truth;
  cv::Size image_size;

  long ts_start = 0;
  long ts_end = 0;
  long ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::set<long> timestamps;
  std::map<long, double> time;
  std::multimap<long, timeline_event_t> timeline;

  euroc_data_t();
  euroc_data_t(const std::string &data_path);
  ~euroc_data_t();
};

/**
 * Load dataset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
int euroc_data_load(euroc_data_t &ds);

/**
 * Reset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
void euroc_data_reset(euroc_data_t &ds);

/**
 * Return min timestamp
 *
 * @param[in] ds Dataset
 * @returns Minimum timestamp
 */
long euroc_data_min_timestamp(const euroc_data_t &ds);

/**
 * Return max timestamp
 * @returns Maximum timestamp
 */
long euroc_data_max_timestamp(const euroc_data_t &ds);

/*****************************************************************************
 * euroc_target_t
 ****************************************************************************/

struct euroc_target_t {
  bool ok = false;
  std::string file_path;

  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  euroc_target_t();
  euroc_target_t(const std::string &file_path);
  ~euroc_target_t();
};

/**
 * Load calibration target settings
 *
 * @param target_file Path to target yaml file
 * @returns 0 for success, -1 for failure
 */
int euroc_target_load(euroc_target_t &target, const std::string &target_file);

/**
 * `calib_target_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const euroc_target_t &target);

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

class euroc_calib_t {
public:
  bool ok = false;

  // Settings
  std::string data_path;
  bool imshow = false;

  // Data
  euroc_imu_t imu_data;
  euroc_camera_t cam0_data;
  euroc_camera_t cam1_data;
  euroc_target_t calib_target;
  cv::Size image_size;

  euroc_calib_t();
  euroc_calib_t(const std::string &data_path);
  ~euroc_calib_t();
};

/**
 * Load calibration data
 * @returns 0 for success, -1 for failure
 */
int euroc_calib_load(euroc_calib_t &data, const std::string &data_path);

} // namespace prototype
#endif /* PROTOTYPE_DATASET_EUROC_HPP */
