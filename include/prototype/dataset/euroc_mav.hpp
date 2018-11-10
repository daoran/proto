#ifndef PROTOTYPE_DATASET_EUROC_MAV_HPP
#define PROTOTYPE_DATASET_EUROC_MAV_HPP

#include <string>
#include <libgen.h>

#include "prototype/core/core.hpp"

namespace prototype {
namespace euroc {

/*****************************************************************************
 * calib_target_t
 ****************************************************************************/

struct calib_target_t {
  bool ok = false;
  std::string file_path;

  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  calib_target_t();
  calib_target_t(const std::string &file_path);
  ~calib_target_t();
};

/**
  * Load calibration target settings
  *
  * @param target_file Path to target yaml file
  * @returns 0 for success, -1 for failure
  */
int calib_target_load(calib_target_t &target, const std::string &target_file);

/**
 * `calib_target_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);



/*****************************************************************************
 * imu_data_t
 ****************************************************************************/

struct imu_data_t {
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

  imu_data_t();
  imu_data_t(const std::string &data_dir_);
  ~imu_data_t();
};

/**
 * Load IMU data
 *
 * @param[in] data IMU data
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int imu_data_load(imu_data_t &data, const std::string &data_dir);

/**
 * `imu_data_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const imu_data_t &data);



/*****************************************************************************
 * camera_data_t
 ****************************************************************************/

struct camera_data_t {
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
  vec2_t resolution;
  std::string camera_model;
  vec4_t intrinsics;
  std::string distortion_model;
  vec4_t distortion_coefficients;

  camera_data_t();
  camera_data_t(const std::string &data_dir_);
  ~camera_data_t();
};

/**
 * Load Camera data
 *
 * @param[in,out] cd Camera data
 * @param[in] data_dir Data directory
 * @returns 0 for success, -1 for failure
 */
int camera_data_load(camera_data_t &cd, const std::string &data_dir);

/**
 * `camera_data_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const camera_data_t &data);



/*****************************************************************************
 * ground_truth_t
 ****************************************************************************/

struct ground_truth_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> p_RS_R;
  std::vector<vec4_t> q_RS;
  std::vector<vec3_t> v_RS_R;
  std::vector<vec3_t> b_w_RS_S;
  std::vector<vec3_t> b_a_RS_S;

  ground_truth_t();
  ground_truth_t(const std::string data_dir_);
  ~ground_truth_t();
};

/**
 * Load ground truth data
 *
 * @param[in,out] data Ground truth data
 * @returns 0 for success, -1 for failure
 */
int ground_truth_load(ground_truth_t &data, const std::string &data_dir);



/*****************************************************************************
 * dataset_event_t
 ****************************************************************************/

/* Dataset Event Type */
#define NOT_SET 0
#define IMU_EVENT 1
#define CAMERA_EVENT 2

struct dataset_event_t {
  // Event Type
  int type;

  // IMU data
  vec3_t a_m;
  vec3_t w_m;

  // Camera data
  int camera_index;
  std::string image_path;

  dataset_event_t();
  dataset_event_t(const vec3_t &a_m, const vec3_t &w_m);
  dataset_event_t(const int camera_index, const std::string &image_path);
  ~dataset_event_t();
};



/*****************************************************************************
 * mav_dataset_t
 ****************************************************************************/

struct mav_dataset_t {
  bool ok = false;
  std::string data_path;

  imu_data_t imu_data;
  camera_data_t cam0_data;
  camera_data_t cam1_data;
  ground_truth_t ground_truth;
  cv::Size image_size;

  long ts_start = 0;
  long ts_end = 0;
  long ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::vector<long> timestamps;
  std::map<long, double> time;
  std::multimap<long, dataset_event_t> timeline;

  mav_dataset_t();
  mav_dataset_t(const std::string &data_path);
  ~mav_dataset_t();
};

/**
 * Load dataset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
int mav_dataset_load(mav_dataset_t &ds);

/**
 * Reset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
void mav_dataset_reset(mav_dataset_t &ds);

/**
 * Return min timestamp
 *
 * @param[in] ds Dataset
 * @returns Minimum timestamp
 */
long mav_dataset_min_timestamp(const mav_dataset_t &ds);

/**
 * Return max timestamp
 * @returns Maximum timestamp
 */
long mav_dataset_max_timestamp(const mav_dataset_t &ds);



/*****************************************************************************
 * calib_data_t
 ****************************************************************************/

class calib_data_t {
public:
  bool ok = false;

  // Settings
  std::string data_path;
  bool imshow = false;

  // Data
  imu_data_t imu_data;
  camera_data_t cam0_data;
  camera_data_t cam1_data;
  calib_target_t calib_target;
  cv::Size image_size;

  calib_data_t();
  calib_data_t(const std::string &data_path);
  ~calib_data_t();
};

/**
 * Load calibration data
 * @returns 0 for success, -1 for failure
 */
int calib_data_load(calib_data_t &data, const std::string &data_path);

} // namespace euroc
} // namespace prototype
#endif /* PROTOTYPE_DATASET_EUROC_MAV_HPP */
