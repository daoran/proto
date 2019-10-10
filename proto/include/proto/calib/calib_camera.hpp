#ifndef PROTO_CALIB_CALIB_CAMERA_HPP
#define PROTO_CALIB_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/calib/calib_data.hpp"
#include "proto/vision/camera/radtan.hpp"

namespace proto {

/**
 * Pinhole Radial-tangential calibration residual
 */
struct pinhole_radtan4_residual_t {
  double z_[2] = {0.0, 0.0};        ///< Measurement
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  pinhole_radtan4_residual_t(const vec2_t &z, const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

  ~pinhole_radtan4_residual_t() {}

  /**
   * Calculate residual (auto diff version)
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
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
};

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration.
 *
 * @returns 0 or -1 for success or failure
 */
template <typename RESIDUAL>
int calib_camera_stats(const aprilgrids_t &aprilgrids,
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
  double err_sum = 0.0;
  for (auto &residual : residuals) {
    const double err = residual.norm();
    const double err_sq = err * err;
    err_sum += err_sq;
  }
  const double err_mean = err_sum / (double) residuals.size();
  const double rmse = sqrt(err_mean);
  std::cout << "nb_residuals: " << residuals.size() << std::endl;
  std::cout << "RMSE Reprojection Error [px]: " << rmse << std::endl;

  return 0;
}

/**
 * Generate camera poses for Next-Best-View.
 */
mat4s_t calib_generate_poses(const calib_target_t &target);

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target.
 *
 * @returns 0 or -1 for success or failure
 */
int calib_camera_solve(const aprilgrids_t &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan,
                       mat4s_t &poses);

/**
 * Calibrate camera intrinsics and relative pose between camera and fiducial
 * calibration target. This function assumes that the path to `config_file`
 * is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *       imshow: true
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.085           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 *
 * @returns 0 or -1 for success or failure
 */
int calib_camera_solve(const std::string &config_file);

} //  namespace proto
#endif // PROTO_CALIB_CALIB_CAMERA_HPP
