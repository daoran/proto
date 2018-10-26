/**
 * @file
 * @defgroup kitti kitti
 * @ingroup dataset
 */
#ifndef PROTOTYPE_DATASET_KITTI_RAW_CALIB_HPP
#define PROTOTYPE_DATASET_KITTI_RAW_CALIB_HPP

#include "prototype/core.hpp"
#include "prototype/dataset/kitti/raw/calib.hpp"
#include "prototype/dataset/kitti/raw/parse.hpp"

namespace prototype {
/**
 * @addtogroup kitti
 * @{
 */

/**
 * Camera to Camera calibration
 */
struct calib_cam2cam_t {
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
 * Load camera to camera calibration
 *
 * @param[in] file_path
 * @returns 0 or -1 for success or failure
 */
int calib_cam2cam_load(calib_cam2cam_t &calib);

/**
 * Load imu to velo calibration
 *
 * @param[in] file_path
 * @returns 0 or -1 for success or failure
 */
int calib_imu2velo_load(calib_imu2velo_t &calib);

/**
 * Load velo to camera calibration
 *
 * @param[in] file_path
 * @returns 0 or -1 for success or failure
 */
int calib_velo2cam_load(calib_velo2cam_t &calib);

/** @} group kitti */
} //  namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_RAW_CALIB_HPP
