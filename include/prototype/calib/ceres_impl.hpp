#ifndef PROTOTYPE_CALIB_CERES_IMPL_HPP
#define PROTOTYPE_CALIB_CERES_IMPL_HPP

#include "prototype/core/core.hpp"

namespace proto {

template <typename T>
Eigen::Matrix<T, 4, 4> tf(const Eigen::Matrix<T, 3, 3> &C,
                          const Eigen::Matrix<T, 3, 1> &r) {
  Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
  transform.block(0, 0, 3, 3) = C;
  transform.block(0, 3, 3, 1) = r;
  return transform;
}

template <typename T>
static Eigen::Matrix<T, 3, 3> pinhole_K(const T *intrinsics) {
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
Eigen::Matrix<T, 4, 1> equi4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T k3 = distortion[2];
  const T k4 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, k3, k4};
  return D;
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
static Eigen::Matrix<T, 2, 1>
pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                        const Eigen::Matrix<T, 4, 1> &D,
                        const Eigen::Matrix<T, 3, 1> &point) {
  const T k1 = D(0);
  const T k2 = D(1);
  const T p1 = D(2);
  const T p2 = D(3);

  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

  // Radial distortion factor
  const T x2 = x * x;
  const T y2 = y * y;
  const T r2 = x2 + y2;
  const T r4 = r2 * r2;
  const T radial_factor = T(1) + (k1 * r2) + (k2 * r4);
  const T x_dash = x * radial_factor;
  const T y_dash = y * radial_factor;

  // Tangential distortion factor
  const T xy = x * y;
  const T x_ddash = x_dash + (T(2) * p1 * xy + p2 * (r2 + T(2) * x2));
  const T y_ddash = y_dash + (p1 * (r2 + T(2) * y2) + T(2) * p2 * xy);

  // Scale distorted point
  Eigen::Matrix<T, 2, 1> x_distorted{x_ddash, y_ddash};
  const Eigen::Matrix<T, 2, 1> pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

}  // namespace proto
#include "ceres_impl.hpp"
#endif // PROTOTYPE_CALIB_CERES_IMPL_HPP
