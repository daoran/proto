#ifndef PROTOTYPE_CALIB_CALIB_STEREO_IMPL_HPP
#define PROTOTYPE_CALIB_CALIB_STEREO_IMPL_HPP

namespace prototype {

template <typename T>
bool stereo_residual_t::operator()(const T *const cam0_intrinsics,
                                   const T *const cam0_distortion,
                                   const T *const cam1_intrinsics,
                                   const T *const cam1_distortion,
                                   const T *const q_C0C1_,
                                   const T *const t_C0C1_,
                                   const T *const q_C0F_,
                                   const T *const t_C0F_,
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
  const Eigen::Matrix<T, 3, 1> t_C0F{t_C0F_[0], t_C0F_[1], t_C0F_[2]};
  const Eigen::Matrix<T, 4, 4> T_C0F = tf(R_C0F, t_C0F);
  // -- Create transform between cam0 and cam1
  const Eigen::Quaternion<T> q_C0C1(q_C0C1_[3], q_C0C1_[0], q_C0C1_[1], q_C0C1_[2]);
  const Eigen::Matrix<T, 3, 3> R_C0C1 = q_C0C1.toRotationMatrix();
  const Eigen::Matrix<T, 3, 1> t_C0C1{t_C0C1_[0], t_C0C1_[1], t_C0C1_[2]};
  const Eigen::Matrix<T, 4, 4> T_C0C1 = tf(R_C0C1, t_C0C1);
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

} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_STEREO_IMPL_HPP
