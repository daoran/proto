#include "CalibCameraImuError.hpp"

namespace xyz {

CalibCameraImuError::CalibCameraImuError(
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec3 &p_FFi,
    const Vec2 &z,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraImuError", param_ptrs, param_types, 2},
      camera_geometry_{camera_geometry}, p_FFi_{p_FFi}, z_{z}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraImuError>
CalibCameraImuError::create(const std::shared_ptr<CameraGeometry> &camera,
                            double *T_WS,
                            double *T_WF,
                            double *T_SC0,
                            const Vec3 &p_FFi,
                            const Vec2 &z,
                            const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(T_WS);
  param_ptrs.push_back(T_WF);
  param_ptrs.push_back(T_SC0);
  param_ptrs.push_back(camera->getExtrinsicPtr());
  param_ptrs.push_back(camera->getIntrinsicPtr());

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);      // T_WS
  param_types.push_back(ParamBlock::POSE);      // T_WF
  param_types.push_back(ParamBlock::EXTRINSIC); // T_C0S
  param_types.push_back(ParamBlock::EXTRINSIC); // T_C0Ci
  param_types.push_back(ParamBlock::INTRINSIC8);

  return std::make_shared<CalibCameraImuError>(camera,
                                               param_ptrs,
                                               param_types,
                                               p_FFi,
                                               z,
                                               covar);
}

bool CalibCameraImuError::valid() const { return valid_; }

bool CalibCameraImuError::getResiduals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraImuError::getReprojError(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool CalibCameraImuError::eval(double const *const *params,
                               double *res,
                               double **jacs) const {
  // Map parameters out
  const Mat4 T_WS = tf(params[0]);                // Sensor pose
  const Mat4 T_WF = tf(params[1]);                // Fiducial pose
  const Mat4 T_C0S = tf(params[2]);               // Imu extrinsic
  const Mat4 T_C0Ci = tf(params[3]);              // Camera extrinsic
  Eigen::Map<const VecX> intrinsic(params[4], 8); // Camera parameters

  // Transform and project point to image plane
  // -- Transform point from fiducial frame to camera
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_SW = tf_inv(T_WS);
  const Mat4 T_CiF = T_CiC0 * T_C0S * T_SW * T_WF;
  const Vec3 p_Ci = tf_point(T_CiF, p_FFi_);
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
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr) {
    return true;
  }

  // -- Jacobians w.r.t imu pose T_WS
  if (jacs[0]) {
    const Vec3 r_WS = tf_trans(T_WS);
    const Mat3 C_WS = tf_rot(T_WS);
    const Mat4 T_CiS = T_CiC0 * T_C0S;
    const Mat3 C_CiW = tf_rot(T_CiS * T_SW);
    const Vec3 p_W = tf_point(T_WF, p_FFi_);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * -C_CiW;
      J.block<2, 3>(0, 3) = Jhw * -C_CiW * skew(p_W - r_WS) * -C_WS;
    }
  }

  // -- Jacobians w.r.t fiducial pose T_WF
  if (jacs[1]) {
    const Mat4 T_CiS = T_CiC0 * T_C0S;
    const Mat3 C_CiW = tf_rot(T_CiS * T_SW);
    const Mat3 C_WF = tf_rot(T_WF);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * C_CiW;
      J.block<2, 3>(0, 3) = Jhw * C_CiW * -C_WF * skew(p_FFi_);
    }
  }

  // -- Jacobians w.r.t imu extrinsic T_C0S
  if (jacs[2]) {
    const Vec3 p_S = tf_point(T_WS.inverse() * T_WF, p_FFi_);
    const Mat3 C_C0S = tf_rot(T_C0S);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * C_CiC0;
      J.block<2, 3>(0, 3) = Jhw * C_CiC0 * -C_C0S * skew(p_S);
    }
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  if (jacs[3]) {
    const Vec3 p_C0 = tf_point(T_C0S * T_WS.inverse() * T_WF, p_FFi_);
    const Vec3 r_C0Ci = tf_trans(T_C0Ci);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);
    const Mat3 C_C0Ci = C_CiC0.transpose();

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * -C_CiC0;
      J.block<2, 3>(0, 3) = Jhw * -C_CiC0 * skew(p_C0 - r_C0Ci) * -C_C0Ci;
    }
  }

  // -- Jacobians w.r.t intrinsic
  if (jacs[4]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[4]);
    J.setZero();
    if (valid_) {
      J = -1.0 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
    }
  }

  return true;
}

} // namespace xyz
