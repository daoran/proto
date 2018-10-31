/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
#define PROTOTYPE_CALIB_GIMBAL_CALIB_HPP

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/model/gimbal.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Gimbal calibration data
 */
struct gimbal_calib_data_t {
  bool ok = false;

  int nb_measurements = 0;
  std::vector<matx_t> P_s;
  std::vector<matx_t> P_d;
  std::vector<matx_t> Q_s;
  std::vector<matx_t> Q_d;
  matx_t joint_data;

  gimbal_calib_data_t();
  virtual ~gimbal_calib_data_t();
};

/**
 * Calibration parameters
 */
struct gimbal_calib_params_t {
  bool ok = false;
  // Camchain camchain;

  double *tau_s = nullptr;
  double *tau_d = nullptr;
  double *w1 = nullptr;
  double *w2 = nullptr;
  double *theta1_offset = nullptr;
  double *theta2_offset = nullptr;
  double *Lambda1 = nullptr;
  double *Lambda2 = nullptr;
  int nb_measurements = 0;

  gimbal_calib_params_t();
  virtual ~gimbal_calib_params_t();
};

/**
 * Gimbal calibrator
 */
struct gimbal_calibrator_t {
  std::string data_dir;
  gimbal_calib_data_t data;
  gimbal_calib_params_t params;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  gimbal_calibrator_t();
  virtual ~gimbal_calibrator_t();
};

/**
 * gimbal_calib_data_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_calib_data_t &m);

/**
 * gimbal_calib_params_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_calib_params_t &m);

/**
 * Load gimbal calibration data
 *
 * @param[in,out] data Gimbal calibration data
 * @param[in] data_dir Data directory
 * @return 0 for success, -1 for failure
 */
int gimbal_calib_data_load(gimbal_calib_data_t &data,
                           const std::string &data_dir);

/**
 * Load initial optimization params
 *
 * @param[in,out] data Gimbal calibration data
 * @param[in] camchain_file Path to camchain file
 * @param[in] joint_file Path to joint angles file
 * @returns 0 for success, -1 for failure
 */
int gimbal_calib_params_load(gimbal_calib_data_t &data,
                             const std::string &camchain_file,
                             const std::string &joint_file);

/**
 * Load gimbal calibrator and data
 *
 * @param[in,out] calib Gimbal calibrator
 * @param[in] data_dir Path to where camchain file and calib data are
 * @returns 0 for success, -1 for failure
 */
int gimbal_calibrator_load(gimbal_calibrator_t &calib,
                           const std::string &data_dir);

/**
 * Calculate reprojection errors
 *
 * @param[in,out] calib Gimbal calibrator
 * @return 0 for success, -1 for failure
 */
int gimbal_calibrator_calc_reprojection_errors(gimbal_calibrator_t &calib);

/**
 * Calibrate calibration
 *
 * @param[in,out] calib Gimbal calibrator
 * @return 0 for success, -1 for failure
 */
int gimbal_calibrator_solve(gimbal_calibrator_t &calib);

/**
 * Gimbal calibration residual
 */
struct GimbalCalibResidual {
  // Observed 3d point in static camera
  double P_s[3] = {0};

  // Observed 3d point in dynamic camera
  double P_d[3] = {0};

  // Observed pixel in static camera
  double Q_s[2] = {0};

  // Observed pixel in dynamic camera
  double Q_d[2] = {0};

  // Theta1 and Theta2 offset
  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

  // Static camera intrinsics
  double fx_s = 0.0;
  double fy_s = 0.0;
  double cx_s = 0.0;
  double cy_s = 0.0;

  // Dynamic camera intrinsics
  double fx_d = 0.0;
  double fy_d = 0.0;
  double cx_d = 0.0;
  double cy_d = 0.0;

  // Static camera distortion coefficients
  double k1_s = 0.0;
  double k2_s = 0.0;
  double k3_s = 0.0;
  double k4_s = 0.0;

  // Dynamic camera distortion coefficients
  double k1_d = 0.0;
  double k2_d = 0.0;
  double k3_d = 0.0;
  double k4_d = 0.0;

  GimbalCalibResidual();
  GimbalCalibResidual(const vec3_t &P_s,
                      const vec3_t &P_d,
                      const vec2_t &Q_s,
                      const vec2_t &Q_d,
                      const mat3_t &K_s,
                      const mat3_t &K_d,
                      const vec4_t &D_s,
                      const vec4_t &D_d,
                      double theta1_offset,
                      double theta2_offset);

  /// Form DH-Transform
  template <typename T>
  Eigen::Matrix<T, 4, 4>
  dhTransform(const T theta, const T alpha, const T a, const T d) const;

  /// Euler 3-2-1 to rotation matrix
  template <typename T>
  Eigen::Matrix<T, 3, 3> euler321ToRot(const T phi,
                                       const T theta,
                                       const T psi) const;

  /// Form camera intrinsics matrix K
  template <typename T>
  Eigen::Matrix<T, 3, 3>
  K(const T fx, const T fy, const T cx, const T cy) const;

  /// Form camera distortion vector D
  template <typename T>
  Eigen::Matrix<T, 4, 1>
  D(const T k1, const T k2, const T k3, const T k4) const;

  /// Form transform from static to dynamic camera
  template <typename T>
  Eigen::Matrix<T, 4, 4> T_ds(const T *const tau_s,
                              const T *const tau_d,
                              const T *const w1,
                              const T *const w2,
                              const T *const Lambda1,
                              const T *const Lambda2) const;

  /// Project 3D point using pinhole-equi
  template <typename T>
  Eigen::Matrix<T, 2, 1>
  project_pinhole_equi(const Eigen::Matrix<T, 3, 3> &K,
                       const Eigen::Matrix<T, 4, 1> &D,
                       const Eigen::Matrix<T, 3, 1> &X) const;

  /// Project 3D point using pinhole-radtan
  template <typename T>
  Eigen::Matrix<T, 2, 1>
  project_pinhole_radtan(const Eigen::Matrix<T, 3, 3> &K,
                         const Eigen::Matrix<T, 4, 1> &D,
                         const Eigen::Matrix<T, 3, 1> &X) const;

  /// Project 3D point using pinhole
  template <typename T>
  Eigen::Matrix<T, 2, 1> project_pinhole(const Eigen::Matrix<T, 3, 3> &K,
                                         const Eigen::Matrix<T, 3, 1> &X) const;

  /// Calculate residual
  template <typename T>
  bool operator()(const T *const tau_s,
                  const T *const tau_d,
                  const T *const w1,
                  const T *const w2,
                  const T *const Lambda1,
                  const T *const Lambda2,
                  T *residual) const;
};

/**
 * GimbalCalibResidual to string
 */
std::ostream &operator<<(std::ostream &os, const GimbalCalibResidual &residual);

/**
 * GimbalCalibResidual to string
 */
std::ostream &operator<<(std::ostream &os, const GimbalCalibResidual *residual);

/** @} group gimbal */
} //  namespace prototype
#include "prototype/calib/gimbal_calib_impl.hpp"
#endif // PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
