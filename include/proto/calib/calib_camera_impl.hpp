#ifndef PROTO_CALIB_CALIB_CAMERA_IMPL_HPP
#define PROTO_CALIB_CALIB_CAMERA_IMPL_HPP

#include "proto/calib/calib_camera.hpp"

namespace proto {

template <typename T>
bool pinhole_radtan4_residual_t::operator()(const T *const intrinsics_,
                                            const T *const distortion_,
                                            const T *const q_CF_,
                                            const T *const t_CF_,
                                            T *residuals_) const {
  // Map variables to Eigen
  const Eigen::Matrix<T, 3, 3> K = pinhole_K(intrinsics_);
  const Eigen::Matrix<T, 4, 1> D = radtan4_D(distortion_);
  const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};

  // Form tf
  const Eigen::Quaternion<T> q_CF(q_CF_[3], q_CF_[0], q_CF_[1], q_CF_[2]);
  const Eigen::Matrix<T, 3, 3> R_CF = q_CF.toRotationMatrix();
  const Eigen::Matrix<T, 3, 1> t_CF{t_CF_[0], t_CF_[1], t_CF_[2]};
  Eigen::Matrix<T, 4, 4> T_CF = tf(R_CF, t_CF);

  // Project
  const Eigen::Matrix<T, 3, 1> p_C = (T_CF * p_F.homogeneous()).head(3);
  const Eigen::Matrix<T, 2, 1> z_hat = pinhole_radtan4_project(K, D, p_C);

  // Residual
  residuals_[0] = T(z_[0]) - z_hat(0);
  residuals_[1] = T(z_[1]) - z_hat(1);

  return true;
}

template <typename RESIDUAL>
int calib_camera_stats(const std::vector<aprilgrid_t> &aprilgrids,
                       const double *intrinsics,
                       const double *distortion,
                       const mat4s_t &poses,
                       const std::string &output_path) {
  UNUSED(output_path);
  vec2s_t residuals;

  // Obtain residuals using optimized params
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    const auto aprilgrid = aprilgrids[i];

    // Form relative pose
    const mat4_t T_CF = poses[i];
    const quat_t q_CF{T_CF.block<3, 3>(0, 0)};
    const vec3_t t_CF{T_CF.block<3, 1>(0, 3)};

    // Iterate over all tags in AprilGrid
    for (const auto &tag_id : aprilgrid.ids) {
      // Get keypoints
      vec2s_t keypoints;
      if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
        LOG_ERROR("Failed to get AprilGrid keypoints!");
        return -1;
      }

      // Get object points
      vec3s_t object_points;
      if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
        LOG_ERROR("Failed to calculate AprilGrid object points!");
        return -1;
      }

      // Form residual and call the functor for four corners of the tag
      for (size_t j = 0; j < 4; j++) {
        const RESIDUAL residual{keypoints[j], object_points[j]};
        double res[2] = {0.0, 0.0};
        residual(intrinsics,
                 distortion,
                 q_CF.coeffs().data(),
                 t_CF.data(),
                 res);
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
  const double mean = sum / (double) residuals.size();
  const double rmse = sqrt(mean);
  std::cout << "nb_residuals: " << residuals.size() << std::endl;
  std::cout << "sum: " << sum << std::endl;
  std::cout << "mean: " << mean << std::endl;
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return 0;
}

} //  namespace proto
#endif // PROTO_CALIB_CALIB_CAMERA_IMPL_HPP
