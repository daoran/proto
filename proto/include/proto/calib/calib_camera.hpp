#ifndef PROTO_CALIB_CALIB_CAMERA_HPP
#define PROTO_CALIB_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/calib/calib_data.hpp"
#include "proto/vision/vision.hpp"

namespace proto {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

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
                  const T *const r_CF_,
                  T *residuals_) const {
    // Map variables to Eigen
    const Eigen::Matrix<T, 3, 3> K = pinhole_K(intrinsics_);
    const Eigen::Matrix<T, 4, 1> D = radtan4_D(distortion_);
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};

    // Form tf
    const Eigen::Quaternion<T> q_CF(q_CF_[3], q_CF_[0], q_CF_[1], q_CF_[2]);
    const Eigen::Matrix<T, 3, 3> R_CF = q_CF.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_CF{r_CF_[0], r_CF_[1], r_CF_[2]};
    Eigen::Matrix<T, 4, 4> T_CF = tf(R_CF, r_CF);

    // Transform and project point to image plane
    const Eigen::Matrix<T, 3, 1> p_C = (T_CF * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_hat;
    if (pinhole_radtan4_project(K, D, p_C, z_hat) != 0) {
      return false;
    }

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
    const quat_t q_CF = tf_quat(T_CF);
    const vec3_t r_CF = tf_trans(T_CF);

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
                 r_CF.data(),
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

aprilgrid_t
nbv_create_aprilgrid(const calib_target_t &target,
                     const camera_geometry_t<pinhole_t, radtan4_t> &camera,
                     const vec2_t &resolution,
                     const mat4_t &T_CF);

void nbv_draw_aprilgrid(const aprilgrid_t grid,
                        const pinhole_t &pinhole,
                        const radtan4_t &radtan,
                        const mat4_t &T_CT,
                        cv::Mat &frame);

void nbv_find(const calib_target_t &target,
              const aprilgrids_t &aprilgrids,
              const pinhole_t &pinhole,
              const radtan4_t &radtan,
              mat4_t &nbv_pose,
              aprilgrid_t &nbv_grid);

double calib_camera_nbv_solve(aprilgrids_t &aprilgrids,
                              pinhole_t &pinhole,
                              radtan4_t &radtan,
                              mat4s_t &T_CF);

int calib_camera_nbv(const std::string &target_path,
                     const size_t max_frames = 15);

int calib_camera_batch(const std::string &target_path,
                       const size_t max_frames = 15);

/*****************************************************************************
 *                             STEREO CAMERA
 ****************************************************************************/

/**
 * Stereo camera calibration residual
 */
struct stereo_residual_t {
  double z_C0_[2] = {0.0, 0.0};     ///< Measurement from cam0
  double z_C1_[2] = {0.0, 0.0};     ///< Measurement from cam1
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  stereo_residual_t(const vec2_t &z_C0, const vec2_t &z_C1, const vec3_t &p_F)
      : z_C0_{z_C0(0), z_C0(1)}, z_C1_{z_C1(0), z_C1(1)}, p_F_{p_F(0),
                                                               p_F(1),
                                                               p_F(2)} {}

  ~stereo_residual_t() {}

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const cam0_intrinsics,
                  const T *const cam0_distortion,
                  const T *const cam1_intrinsics,
                  const T *const cam1_distortion,
                  const T *const q_C0C1_,
                  const T *const r_C0C1_,
                  const T *const q_C0F_,
                  const T *const r_C0F_,
                  T *residual) const {
    // Map variables to Eigen
    // -- Fiducial point
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
    // -- cam0 intrinsics and distortion
    const Eigen::Matrix<T, 3, 3> cam0_K = pinhole_K(cam0_intrinsics);
    const Eigen::Matrix<T, 4, 1> cam0_D = radtan4_D(cam0_distortion);
    // -- cam1 intrinsics and distortion
    const Eigen::Matrix<T, 3, 3> cam1_K = pinhole_K(cam1_intrinsics);
    const Eigen::Matrix<T, 4, 1> cam1_D = radtan4_D(cam1_distortion);

    // Form transforms
    // clang-format off
    // -- Create transform between fiducial and cam0
    const Eigen::Quaternion<T> q_C0F(q_C0F_[3], q_C0F_[0], q_C0F_[1], q_C0F_[2]);
    const Eigen::Matrix<T, 3, 3> R_C0F = q_C0F.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0F{r_C0F_[0], r_C0F_[1], r_C0F_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0F = tf(R_C0F, r_C0F);
    // -- Create transform between cam0 and cam1
    const Eigen::Quaternion<T> q_C0C1(q_C0C1_[3], q_C0C1_[0], q_C0C1_[1], q_C0C1_[2]);
    const Eigen::Matrix<T, 3, 3> R_C0C1 = q_C0C1.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_C0C1{r_C0C1_[0], r_C0C1_[1], r_C0C1_[2]};
    const Eigen::Matrix<T, 4, 4> T_C0C1 = tf(R_C0C1, r_C0C1);
    const Eigen::Matrix<T, 4, 4> T_C1C0 = T_C0C1.inverse();
    // clang-format on

    // Project
    // clang-format off
    // -- Project point observed from cam0 to cam0 image plane
    const Eigen::Matrix<T, 3, 1> p_C0 = (T_C0F * p_F.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C0_hat;
    if (pinhole_radtan4_project(cam0_K, cam0_D, p_C0, z_C0_hat) != 0) {
      return false;
    }
    // -- Project point observed from cam0 to cam1 image plane
    const Eigen::Matrix<T, 3, 1> p_C1 = (T_C1C0 * p_C0.homogeneous()).head(3);
    Eigen::Matrix<T, 2, 1> z_C1_hat;
    if (pinhole_radtan4_project(cam1_K, cam1_D, p_C1, z_C1_hat) != 0) {
      return false;
    }
    // clang-format on

    // Residual
    // -- cam0 residual
    residual[0] = T(z_C0_[0]) - z_C0_hat(0);
    residual[1] = T(z_C0_[1]) - z_C0_hat(1);
    // -- cam1 residual
    residual[2] = T(z_C1_[0]) - z_C1_hat(0);
    residual[3] = T(z_C1_[1]) - z_C1_hat(1);

    return true;
  }
};

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras.
 */
int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_aprilgrids,
                       const std::vector<aprilgrid_t> &cam1_aprilgrids,
                       pinhole_t &cam0_pinhole,
                       radtan4_t &cam0_radtan,
                       pinhole_t &cam1_pinhole,
                       radtan4_t &cam1_radtan,
                       mat4_t &T_C0C1,
                       mat4s_t &T_C0F);

/**
 * Calibrate stereo camera extrinsics and relative pose between cameras. This
 * function assumes that the path to `config_file` is a yaml file of the form:
 *
 *     settings:
 *       data_path: "/data"
 *       results_fpath: "/data/calib_results.yaml"
 *
 *     calib_target:
 *       target_type: 'aprilgrid'  # Target type
 *       tag_rows: 6               # Number of rows
 *       tag_cols: 6               # Number of cols
 *       tag_size: 0.088           # Size of apriltag, edge to edge [m]
 *       tag_spacing: 0.3          # Ratio of space between tags to tagSize
 *                                 # Example: tagSize=2m, spacing=0.5m
 *                                 # --> tagSpacing=0.25[-]
 *
 *     cam0:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 *
 *     cam1:
 *       resolution: [752, 480]
 *       lens_hfov: 98.0
 *       lens_vfov: 73.0
 *       camera_model: "pinhole"
 *       distortion_model: "radtan"
 */
int calib_stereo_solve(const std::string &config_file);


} //  namespace proto
#endif // PROTO_CALIB_CALIB_CAMERA_HPP
