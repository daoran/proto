#include "ReprojError.hpp"

namespace xyz {

ReprojError::ReprojError(const std::shared_ptr<CameraGeometry> &camera_geometry,
                         const std::vector<double *> &param_ptrs,
                         const std::vector<ParamBlock::Type> &param_types,
                         const Vec2 &z,
                         const Mat2 &covar)
    : ResidualBlock{"ReprojError", param_ptrs, param_types, 2},
      camera_geometry_{camera_geometry}, z_{z}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<ReprojError>
ReprojError::create(const std::shared_ptr<CameraGeometry> &camera,
                    double *T_WB,
                    double *p_W,
                    const Vec2 &z,
                    const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(T_WB);
  param_ptrs.push_back(p_W);
  param_ptrs.push_back(camera->extrinsic.data());
  param_ptrs.push_back(camera->intrinsic.data());

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);
  param_types.push_back(ParamBlock::POINT);
  param_types.push_back(ParamBlock::EXTRINSIC);
  param_types.push_back(ParamBlock::INTRINSIC8);

  return std::make_shared<ReprojError>(camera,
                                       param_ptrs,
                                       param_types,
                                       z,
                                       covar);
}

bool ReprojError::valid() const { return valid_; }

bool ReprojError::getResiduals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool ReprojError::getReprojError(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool ReprojError::eval(double const *const *params,
                       double *res,
                       double **jacs) const {
  // Map parameters out
  const Mat4 T_WB = tf(params[0]);                          // Body pose
  const Vec3 p_W{params[1][0], params[1][1], params[1][2]}; // Point
  const Mat4 T_BCi = tf(params[2]);                         // Camera extrinsic
  Eigen::Map<const VecX> intrinsic(params[3], 8);           // Camera parameters

  // Transform and project point to image plane
  // -- Transform point from world frame to camera
  const Mat4 T_CiW = T_BCi.inverse() * T_WB.inverse();
  const Vec3 p_Ci = tf_point(T_CiW, p_W);
  // -- Project point from camera frame to image plane
  const auto camera_model = camera_geometry_->camera_model;
  const Vec2i resolution = camera_geometry_->resolution;
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

  // -- Jacobians w.r.t body pose T_WB
  if (jacs[0]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();

    if (valid_) {
      const Mat3 C_WB = tf_rot(T_WB);
      const Mat3 C_CiW = tf_rot(T_BCi.inverse() * T_WB.inverse());
      const Vec3 r_WB = tf_trans(T_WB);
      J.block<2, 3>(0, 0) = Jhw * -C_CiW;
      J.block<2, 3>(0, 3) = Jhw * -C_CiW * skew(p_W - r_WB) * -C_WB;
    }
  }

  // -- Jacobians w.r.t 3D point p_W
  if (jacs[1]) {
    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiW = tf_rot(T_BCi.inverse() * T_WB.inverse());
      J = Jhw * C_CiW;
    }
  }

  // -- Jacobians w.r.t camera extrinsic T_BCi
  if (jacs[2]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();

    if (valid_) {
      const Mat4 T_CiB = T_BCi.inverse();
      const Mat3 C_CiB = tf_rot(T_CiB);
      const Mat3 C_BCi = tf_rot(T_BCi);
      J.block<2, 3>(0, 0) = Jhw * -C_CiB;
      J.block<2, 3>(0, 3) = Jhw * -C_CiB * skew(C_BCi * p_Ci) * -C_BCi;
    }
  }

  // -- Jacobians w.r.t camera intrinsic
  if (jacs[3]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();

    if (valid_) {
      J = -1.0 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
    }
  }

  return true;
}

} // namespace xyz
