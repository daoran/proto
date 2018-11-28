#ifndef PROTOTYPE_DATASET_KITTI_HPP
#define PROTOTYPE_DATASET_KITTI_HPP

#include "prototype/core/core.hpp"

namespace prototype {

/**
 * Camera to Camera calibration
 */
struct calib_cam2cam_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  double corner_dist = 0.0;
  std::array<vec2_t, 4> S;
  std::array<mat3_t, 4> K;
  std::array<vecx_t, 4> D;
  std::array<mat3_t, 4> R;
  std::array<vec3_t, 4> T;
  std::array<vec2_t, 4> S_rect;
  std::array<mat3_t, 4> R_rect;
  std::array<mat34_t, 4> P_rect;

  calib_cam2cam_t() {}
  calib_cam2cam_t(const std::string file_path_) : file_path{file_path_} {}
};

/**
 * IMU to Velo calibration
 */
struct calib_imu2velo_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  mat3_t R;
  vec3_t t;
  mat4_t T_velo_imu;

  calib_imu2velo_t() {}
  calib_imu2velo_t(const std::string file_path_) : file_path{file_path_} {}
};

/**
 * Velo to Camera calibration
 */
struct calib_velo2cam_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  mat3_t R;
  vec3_t t;
  vec2_t df;
  vec2_t dc;
  mat4_t T_cam_velo;

  calib_velo2cam_t() {}
  calib_velo2cam_t(const std::string file_path_) : file_path{file_path_} {}
};

/**
 * OXTS entry
 */
struct oxts_entry_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string file_path;

  vec3_t gps = zeros(3, 1);
  vec3_t rpy = zeros(3, 1);
  vec3_t v_G = zeros(3, 1);
  vec3_t v_B = zeros(3, 1);
  vec3_t a_G = zeros(3, 1);
  vec3_t a_B = zeros(3, 1);
  vec3_t w_G = zeros(3, 1);
  vec3_t w_B = zeros(3, 1);
  double pos_accuracy = 0.0;
  double vel_accuracy = 0.0;

  oxts_entry_t() {}
  oxts_entry_t(const std::string &file_path_) : file_path{file_path_} {}
};

/**
 * OXTS
 */
struct oxts_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string oxts_dir;

  std::vector<long> timestamps;
  std::vector<double> time;
  vec3s_t gps;
  vec3s_t rpy;
  vec3s_t p_G;
  vec3s_t v_G;
  vec3s_t v_B;
  vec3s_t a_G;
  vec3s_t a_B;
  vec3s_t w_G;
  vec3s_t w_B;
  std::vector<double> pos_accuracy;
  std::vector<double> vel_accuracy;

  oxts_t() {}
  oxts_t(const std::string &oxts_dir_) : oxts_dir{oxts_dir_} {}
};

/**
 * KITTI Raw Dataset
 */
struct kitti_raw_t {
  bool ok = false;

  std::string raw_dir;
  std::string date;
  std::string seq;
  std::string date_dir;
  std::string drive_dir;

  calib_cam2cam_t cam2cam;
  calib_imu2velo_t imu2velo;
  calib_velo2cam_t velo2cam;

  oxts_t oxts;
  std::vector<std::string> cam0;
  std::vector<std::string> cam1;
  std::vector<std::string> cam2;
  std::vector<std::string> cam3;

  kitti_raw_t();
  kitti_raw_t(const std::string &raw_dir,
              const std::string &date,
              const std::string &seq);
  kitti_raw_t(const std::string &raw_dir,
              const std::string &date,
              const std::string &seq,
              const std::string &type);
  ~kitti_raw_t();
};

/**
 * Parse string
 *
 * @param[in] line Line containing a string
 * @returns A string
 */
std::string kitti_parse_string(const std::string &line);

/**
 * Parse double
 *
 * @param[in] line Line containing a double
 * @returns A double
 */
double kitti_parse_double(const std::string &line);

/**
 * Parse array
 *
 * @param[in] line Line containing an array
 * @returns Array
 */
std::vector<double> kitti_parse_array(const std::string &line);

/**
 * Parse vector of size 2
 *
 * @param[in] line Line containing a vector of size 2
 * @returns Vector of size 2
 */
vec2_t kitti_parse_vec2(const std::string &line);

/**
 * Parse vector of size 3
 *
 * @param[in] line Line containing a vector of size 3
 * @returns Vector of size 3
 */
vec3_t kitti_parse_vec3(const std::string &line);

/**
 * Parse vector
 *
 * @param[in] line Line containing a vector
 * @returns Vector
 */
vecx_t kitti_parse_vecx(const std::string &line);

/**
 * Parse 3x3 matrix
 *
 * @param[in] line Line containing a 3x3 matrix
 * @returns 3x3 matrix
 */
mat3_t kitti_parse_mat3(const std::string &line);

/**
 * Parse 3x4 matrix
 *
 * @param[in] line Line containing a 3x4 matrix
 * @returns 3x4 matrix
 */
mat34_t kitti_parse_mat34(const std::string &line);

/**
 * Parse timestamp
 *
 * @param[in] line Line containing a timestamp
 * @param[out] s Output string
 * @returns 0 or -1 for success or failure
 */
int kitti_parse_timestamp(const std::string &line, long *s);

/**
 * Load camera to camera calibration
 *
 * @param[in,out] calib Camera to camera calibration
 * @returns 0 or -1 for success or failure
 */
int calib_cam2cam_load(calib_cam2cam_t &calib);

/**
 * Load imu to velo calibration
 *
 * @param[in,out] calib IMU to VELO calibration
 * @returns 0 or -1 for success or failure
 */
int calib_imu2velo_load(calib_imu2velo_t &calib);

/**
 * Load velo to camera calibration
 *
 * @param[in,out] calib VELO to camera calibration
 * @returns 0 or -1 for success or failure
 */
int calib_velo2cam_load(calib_velo2cam_t &calib);

/**
 * Load OXTS entry
 *
 * @param[in,out] entry OXTS entry
 * @returns 0 for success, -1 for failure
 */
int oxts_entry_load(oxts_entry_t &entry);

/**
 * load OXTS
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_entries(oxts_t &oxts);

/**
 * Load timestamps
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_timestamps(oxts_t &oxts);

/**
 * Load OXTS
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load(oxts_t &oxts);

/**
 * Load KITTI raw dataset
 *
 * @param[in,out] dataset KITTI raw dataset
 * @returns 0 or -1 for success or failure
 */
int kitti_raw_load(kitti_raw_t &dataset);

} //  namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_HPP
