#ifndef PROTO_CALIB_CALIB_VICON_MARKER_HPP
#define PROTO_CALIB_CALIB_VICON_MARKER_HPP

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/calib/calib_data.hpp"
#include "proto/calib/calib_camera.hpp"

namespace proto {

/**
 * Stereo camera calibration residual
 */
struct vicon_marker_residual_t {
  double z_[2] = {0.0, 0.0};        ///< Measurement from cam0
  double p_F_[3] = {0.0, 0.0, 0.0}; ///< Object point

  vicon_marker_residual_t(const vec2_t &z, const vec3_t &p_F);
  ~vicon_marker_residual_t();

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
                  const T *const distortion_,
                  const T *const q_MC_,
                  const T *const r_MC_,
                  const T *const q_WM_,
                  const T *const r_WM_,
                  const T *const q_WF_,
                  const T *const r_WF_,
                  T *residual) const {
    // Map optimization variables to Eigen
    // -- Camera intrinsics and distortion
    Eigen::Matrix<T, 8, 1> cam_params;
    cam_params << intrinsics_[0], intrinsics_[1], intrinsics_[2], intrinsics_[3],
                  distortion_[0], distortion_[1], distortion_[2], distortion_[3];
    // -- Marker to camera extrinsics pose
    const Eigen::Quaternion<T> q_MC(q_MC_[3], q_MC_[0], q_MC_[1], q_MC_[2]);
    const Eigen::Matrix<T, 3, 3> C_MC = q_MC.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_MC{r_MC_[0], r_MC_[1], r_MC_[2]};
    const Eigen::Matrix<T, 4, 4> T_MC = tf(C_MC, r_MC);
    // -- Marker pose
    const Eigen::Quaternion<T> q_WM(q_WM_[3], q_WM_[0], q_WM_[1], q_WM_[2]);
    const Eigen::Matrix<T, 3, 3> C_WM = q_WM.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_WM{r_WM_[0], r_WM_[1], r_WM_[2]};
    const Eigen::Matrix<T, 4, 4> T_WM = tf(C_WM, r_WM);
    // -- Fiducial pose
    const Eigen::Quaternion<T> q_WF(q_WF_[3], q_WF_[0], q_WF_[1], q_WF_[2]);
    const Eigen::Matrix<T, 3, 3> C_WF = q_WF.toRotationMatrix();
    const Eigen::Matrix<T, 3, 1> r_WF{r_WF_[0], r_WF_[1], r_WF_[2]};
    const Eigen::Matrix<T, 4, 4> T_WF = tf(C_WF, r_WF);

    // Project fiducial object point to camera image plane
    const Eigen::Matrix<T, 3, 1> p_F{T(p_F_[0]), T(p_F_[1]), T(p_F_[2])};
    const Eigen::Matrix<T, 4, 4> T_CM = T_MC.inverse();
    const Eigen::Matrix<T, 4, 4> T_MW = T_WM.inverse();
    const Eigen::Matrix<T, 4, 1> hp_C = T_CM * T_MW * T_WF * p_F.homogeneous();
    const Eigen::Matrix<T, 3, 1> p_C = hp_C.head(3);
    Eigen::Matrix<T, 2, 1> z_hat;
    if (pinhole_radtan4_project(cam_params, p_C, z_hat) != 0) {
      return false;
    }

    // Residual
    residual[0] = T(z_[0]) - z_hat(0);
    residual[1] = T(z_[1]) - z_hat(1);

    return true;
  }
};

/**
 * Evaluate the vicon marker cost function
 */
double evaluate_vicon_marker_cost(const std::vector<aprilgrid_t> &aprilgrids,
                                  mat4s_t &T_WM,
                                  pinhole_t &pinhole,
                                  radtan4_t &radtan,
                                  mat4_t &T_MC);

/**
 * Calibrate vicon marker to camera extrinsics
 */
int calib_vicon_marker_solve(const aprilgrids_t &aprilgrids,
                             pinhole_t &pinhole,
                             radtan4_t &radtan,
                             mat4s_t &T_WM,
                             mat4_t &T_MC,
                             mat4_t &T_WF);

} //  namespace proto
#endif // PROTO_CALIB_CALIB_VICON_MARKER_HPP
