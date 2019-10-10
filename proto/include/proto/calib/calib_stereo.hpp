#ifndef PROTO_CALIB_CALIB_STEREO_HPP
#define PROTO_CALIB_CALIB_STEREO_HPP

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/calib/calib_data.hpp"

namespace proto {

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
    const Eigen::Matrix<T, 2, 1> z_C0_hat = pinhole_radtan4_project(cam0_K, cam0_D, p_C0);
    // -- Project point observed from cam0 to cam1 image plane
    const Eigen::Matrix<T, 3, 1> p_C1 = (T_C1C0 * p_C0.homogeneous()).head(3);
    const Eigen::Matrix<T, 2, 1> z_C1_hat = pinhole_radtan4_project(cam1_K, cam1_D, p_C1);
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
#endif // PROTO_CALIB_CALIB_STEREO_HPP
