/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_CAMERA_IMPL_HPP
#define PROTOTYPE_CALIB_CALIB_CAMERA_IMPL_HPP

#include "prototype/calib/calib_camera.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

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
static Eigen::Matrix<T, 4, 1> radtan4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T p1 = distortion[2];
  const T p2 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, p1, p2};
  return D;
}

template <typename T>
static Eigen::Matrix<T, 4, 1> equi4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T k3 = distortion[2];
  const T k4 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, k3, k4};
  return D;
}

template <typename T>
static Eigen::Matrix<T, 2, 1> pinhole_radtan4_project(
    const Eigen::Matrix<T, 3, 3> &K,
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

template <typename T>
bool pinhole_radtan4_residual_t::operator()(const T *const intrinsics,
                                            const T *const distortion,
                                            const T *const q_CF_,
                                            const T *const t_CF_,
                                            T *residual) const {
  // Map variables to Eigen
  const Eigen::Matrix<T, 3, 3> K = pinhole_K(intrinsics);
  const Eigen::Matrix<T, 4, 1> D = radtan4_D(distortion);
  const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};

  // Form transform
  const Eigen::Quaternion<T> q_CF(q_CF_[3], q_CF_[0], q_CF_[1], q_CF_[2]);
  const Eigen::Matrix<T, 3, 3> R_CF = q_CF.toRotationMatrix();
  const Eigen::Matrix<T, 3, 1> t_CF{t_CF_[0], t_CF_[1], t_CF_[2]};
  Eigen::Matrix<T, 4, 4> T_CF = Eigen::Matrix<T, 4, 4>::Identity();
  T_CF.block(0, 0, 3, 3) = R_CF;
  T_CF.block(0, 3, 3, 1) = t_CF;

  // Project
  const Eigen::Matrix<T, 3, 1> p_C = (T_CF * p_F.homogeneous()).head(3);
  const Eigen::Matrix<T, 2, 1> z_hat = pinhole_radtan4_project(K, D, p_C);

  // Residual
  residual[0] = T(z_[0]) - z_hat(0);
  residual[1] = T(z_[1]) - z_hat(1);

  return true;
}

template <typename RESIDUAL>
int calib_camera_stats(const std::vector<aprilgrid_t> &aprilgrids,
                       const double *intrinsics,
                       const double *distortion,
                       const std::vector<mat4_t> &poses,
                       const std::string &output_path) {
  std::vector<vec2_t> residuals;

  // Obtain residuals using optimized params
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    const quat_t q_CF{T_CF.block<3, 3>(0, 0)};
    const vec3_t t_CF{T_CF.block<3, 1>(0, 3)};

    // Iterate over all tags in AprilGrid
    for (const auto &tag_id: aprilgrid.ids) {
      // Get keypoints
      std::vector<vec2_t> keypoints;
      if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
        LOG_ERROR("Failed to get AprilGrid keypoints!");
        return -1;
      }

      // Get object points
      std::vector<vec3_t> object_points;
      if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
        LOG_ERROR("Failed to calculate AprilGrid object points!");
        return -1;
      }

      // Form residual and call the functor for four corners of the tag
      for (size_t j = 0; j < 4; j++) {
        const RESIDUAL residual{keypoints[j], object_points[j]};
        double res[2] = {0.0, 0.0};
        residual(intrinsics, distortion, q_CF.coeffs().data(), t_CF.data(), res);
        residuals.emplace_back(res[0], res[1]);
      }
    }
  }

  // Calculate RMSE reprojection error
  double sum = 0.0;
  for (auto &residual : residuals) {
    const double norm = residual.norm();
    const double err_sq = norm * norm;
    sum += err_sq;
  }
  const double rmse = sqrt(sum / residuals.size());
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return 0;
}

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_CAMERA_IMPL_HPP
