#include "CalibCameraError.hpp"

namespace xyz {

CalibCameraError::CalibCameraError(
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec2 &z,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraError", param_ptrs, param_types, 2},
      camera_geometry_{camera_geometry}, z_{z}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraError>
CalibCameraError::create(const std::shared_ptr<CameraGeometry> &camera,
                          double *T_C0F,
                          double *p_FFi,
                          const Vec2 &z,
                          const Mat2 &covar) {
  std::vector<double *> param_ptrs = {p_FFi,
                                      T_C0F,
                                      camera->getExtrinsicPtr(),
                                      camera->getIntrinsicPtr()};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POINT,
                                               ParamBlock::POSE,
                                               ParamBlock::EXTRINSIC,
                                               ParamBlock::INTRINSIC8};

  return std::make_shared<CalibCameraError>(camera,
                                             param_ptrs,
                                             param_types,
                                             z,
                                             covar);
}

bool CalibCameraError::getResiduals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraError::getReprojError(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool CalibCameraError::EvaluateWithMinimalJacobians(
    double const *const *params,
    double *res,
    double **jacs,
    double **min_jacs) const {
  // Map parameters out
  Eigen::Map<const Vec3> p_FFi(params[0], 3);
  const Mat4 T_C0F = tf(params[1]);
  const Mat4 T_C0Ci = tf(params[2]);
  Eigen::Map<const VecX> intrinsic(params[3], 8);

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera-n
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_CiF = T_CiC0 * T_C0F;
  const Vec3 p_Ci = tf_point(T_CiF, p_FFi);
  // -- Project point from camera frame to image plane
  const auto camera_model = camera_geometry_->getCameraModel();
  const Vec2i resolution = camera_geometry_->getResolution();

  Vec2 z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
  }

  // Residual
  Eigen::Map<Vec2> r(res);
  r = sqrt_info_ * (z_ - z_hat);
  residuals_ = z_ - z_hat;

  // Jacobians
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jh_weighted = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr) {
    return true;
  }

  // Jacobians w.r.t p_FFi
  if (jacs[0]) {
    const Mat3 C_CiF = tf_rot(T_CiF);
    MatX J_min = Jh_weighted * C_CiF;

    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[0]);
    J = (valid_) ? J_min : zeros(2, 3);

    if (min_jacs && min_jacs[0]) {
      Eigen::Map<Mat<2, 3, Eigen::RowMajor>> min_J(min_jacs[0]);
      min_J = (valid_) ? J_min : zeros(2, 3);
    }
  }

  // Jacobians w.r.t T_C0F
  if (jacs[1]) {
    // clang-format off
      const Mat3 C_CiC0 = tf_rot(T_CiC0);
      const Mat3 C_C0F = tf_rot(T_C0F);
      MatX J_min = zeros(2, 6);
      J_min.block(0, 0, 2, 3) = Jh_weighted * C_CiC0;
      J_min.block(0, 3, 2, 3) = Jh_weighted * C_CiC0 * -C_C0F * skew(p_FFi);
    // clang-format on

    Eigen::Map<Mat<2, 7, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    if (valid_) {
      J.block<2, 6>(0, 0) = J_min;
    }

    if (min_jacs && min_jacs[1]) {
      Eigen::Map<Mat<2, 6, Eigen::RowMajor>> min_J(min_jacs[1]);
      min_J = (valid_) ? J_min : zeros(2, 6);
    }
  }

  // Jacobians w.r.t T_C0Ci
  if (jacs[2]) {
    // clang-format off
    const Mat3 C_C0Ci = tf_rot(T_C0Ci);
    const Vec3 p_C0Fi = tf_point(T_C0F, p_FFi);
    const Vec3 p_C0Ci = tf_trans(T_C0Ci);
    MatX J_min = zeros(2, 6);
    J_min.block(0, 0, 2, 3) = Jh_weighted * -C_C0Ci;
    J_min.block(0, 3, 2, 3) = Jh_weighted * -C_C0Ci * skew(p_C0Fi - p_C0Ci) * -C_C0Ci;
    // clang-format on

    Eigen::Map<Mat<2, 7, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();
    if (valid_) {
      J.block<2, 6>(0, 0) = J_min;
    }

    if (min_jacs && min_jacs[2]) {
      Eigen::Map<Mat<2, 6, Eigen::RowMajor>> min_J(min_jacs[2]);
      min_J = (valid_) ? J_min : zeros(2, 6);
    }
  }

  // Jacobians w.r.t intrinsic
  if (jacs[3]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[3]);
    const MatX J_cam = camera_model->params_jacobian(intrinsic, p_Ci);
    const MatX J_min = -1 * sqrt_info_ * J_cam;
    J = (valid_) ? J_min : zeros(2, 8);

    if (min_jacs && min_jacs[3]) {
      Eigen::Map<Mat<2, 8, Eigen::RowMajor>> min_J(min_jacs[3]);
      min_J = (valid_) ? J_min : zeros(2, 8);
    }
  }

  return true;
}

} // namespace xyz
