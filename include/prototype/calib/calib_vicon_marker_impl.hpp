#ifndef PROTOTYPE_CALIB_CALIB_VICON_MARKER_IMPL_HPP
#define PROTOTYPE_CALIB_CALIB_VICON_MARKER_IMPL_HPP

#include "prototype/calib/calib_vicon_marker.hpp"

namespace prototype {

template <typename T>
bool vicon_marker_residual_t::operator()(const T *const intrinsics_,
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
  const Eigen::Matrix<T, 3, 3> K = pinhole_K(intrinsics_);
  const Eigen::Matrix<T, 4, 1> D = radtan4_D(distortion_);
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
  const Eigen::Matrix<T, 2, 1> z_hat = pinhole_radtan4_project(K, D, p_C);

  // Residual
  residual[0] = T(z_[0]) - z_hat(0);
  residual[1] = T(z_[1]) - z_hat(1);

  return true;
}

} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_VICON_MARKER_IMPL_HPP
