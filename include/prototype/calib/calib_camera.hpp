/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_CAMERA_HPP
#define PROTOTYPE_CALIB_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/calib/calib.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Pose parameter block
 */
struct pose_param_t {
  quat_t q;
  vec3_t t;

  pose_param_t(const mat4_t &T)
    : q{T.block<3, 3>(0, 0)}, t{T.block<3, 1>(0, 3)} {}

  ~pose_param_t() {}
};

/**
 * Pinhole Radial-tangential calibration residual
 */
struct pinhole_radtan4_residual_t {
  double obj_point_[3] = {0.0, 0.0, 0.0};
  double measurement_[2] = {0.0, 0.0};

  pinhole_radtan4_residual_t(const vec2_t &measurement, const vec3_t &obj_point) {
    measurement_[0] = measurement(0);
    measurement_[1] = measurement(1);

    obj_point_[0] = obj_point(0);
    obj_point_[1] = obj_point(1);
    obj_point_[2] = obj_point(2);
  }

  /// Calculate residual
  template <typename T>
  bool operator()(const T *const intrinsics,
                  const T *const distortion,
                  const T *const q_CF,
                  const T *const t_CF,
                  T *residual) const;
};

template <typename T>
Eigen::Matrix<T, 3, 3> pinhole_K(const T *intrinsics) {
  const T fx = intrinsics[0];
  const T fy = intrinsics[1];
  const T cx = intrinsics[2];
  const T cy = intrinsics[3];

  // clang-format off
  Eigen::Matrix<T, 3, 3> K;
  K << fx, T(0.0), cx,
       T(0.0), fy, cy,
       T(0.0), T(0.0), T(1.0);
  // clang-format on
  return K;
}

template <typename T>
Eigen::Matrix<T, 4, 1> radtan4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T p1 = distortion[2];
  const T p2 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, p1, p2};
  return D;
}

template <typename T>
Eigen::Matrix<T, 2, 1> pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                                               const Eigen::Matrix<T, 4, 1> &D,
                                               const Eigen::Matrix<T, 3, 1> &X) {

  // Project
  const T x_dash = X(0) / X(2);
  const T y_dash = X(1) / X(2);

  // Radial distortion factor
  const T k1 = D(0);
  const T k2 = D(1);
  const T r2 = (x_dash * x_dash) + (y_dash * y_dash);
  const T r4 = r2 * r2;
  const T r6 = r2 * r4;
  const T temp = T(1) + (k1 * r2) + (k2 * r4);

  // Tangential distortion factor
  const T p1 = D(2);
  const T p2 = D(3);
  // clang-format off
  const T x_ddash = x_dash * temp + (T(2) * p1 * x_dash * y_dash + p2 * (r2 + T(2) * (x_dash * x_dash)));
  const T y_ddash = y_dash * temp + (p1 * (r2 + T(2) * (y_dash * y_dash)) + T(2) * p2 * x_dash * y_dash);
  // clang-format on

  // Scale distorted point
  Eigen::Matrix<T, 2, 1> x_distorted{x_ddash, y_ddash};
  const Eigen::Matrix<T, 2, 1> pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

template <typename T>
bool pinhole_radtan4_residual_t::operator()(const T *const intrinsics,
                                            const T *const distortion,
                                            const T *const q_CF_data,
                                            const T *const t_CF_data,
                                            T *residual) const {
  // Map variables to Eigen
  const Eigen::Matrix<T, 3, 3> K = pinhole_K(intrinsics);
  const Eigen::Matrix<T, 4, 1> D = radtan4_D(distortion);
  const Eigen::Matrix<T, 3, 1> obj_point{T(obj_point_[0]), T(obj_point_[1]), T(obj_point_[2])};

  // Form transform
  const Eigen::Quaternion<T> q_CF(q_CF_data[3], q_CF_data[0], q_CF_data[1], q_CF_data[2]);
  const Eigen::Matrix<T, 3, 3> R_CF = q_CF.toRotationMatrix();
  const Eigen::Matrix<T, 3, 1> t_CF{t_CF_data[0], t_CF_data[1], t_CF_data[2]};
  Eigen::Matrix<T, 4, 4> T_CF = Eigen::Matrix<T, 4, 4>::Identity();
  T_CF.block(0, 0, 3, 3) = R_CF;
  T_CF.block(0, 3, 3, 1) = t_CF;

  // Project
  const Eigen::Matrix<T, 3, 1> point = (T_CF * obj_point.homogeneous()).head(3);
  const Eigen::Matrix<T, 2, 1> z = pinhole_radtan4_project(K, D, point);

  // Residual
  residual[0] = T(measurement_[0]) - z(0);
  residual[1] = T(measurement_[1]) - z(1);

  return true;
}

/**
 * Setup camera calibration problem
 *
 * @param[in] aprilgrids AprilGrids
 * @returns 0 or -1 for success or failure
 */
int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan);

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_CAMERA_HPP
