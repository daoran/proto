#include "CalibCameraImuError.hpp"
#include "core/SO3.hpp"

namespace cartesian {

CalibCameraImuError::CalibCameraImuError(
    const Mode mode,
    const timestamp_t ts_km1,
    const timestamp_t ts_k,
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec2 &z_k,
    const Vec2 &v_k,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraImuError", param_ptrs, param_types, 2},
      mode_{mode}, ts_km1_{ts_km1}, ts_k_{ts_k},
      camera_geometry_{camera_geometry}, z_k_{z_k}, v_k_{v_k}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraImuError>
CalibCameraImuError::create(const timestamp_t ts_km1,
                            const timestamp_t ts_k,
                            const std::shared_ptr<CameraGeometry> &camera,
                            const std::shared_ptr<ImuGeometry> &imu,
                            const std::shared_ptr<CalibTargetGeometry> &target,
                            double *sensor_pose,
                            double *target_pose,
                            double *time_delay,
                            const int point_id,
                            const Vec2 &z_km1,
                            const Vec2 &z_k,
                            const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(sensor_pose);                     // Sensor pose T_WS
  param_ptrs.push_back(target_pose);                     // Target pose T_WT0
  param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
  param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
  param_ptrs.push_back(imu->extrinsic.data());    // Imu extrinsic T_C0S
  param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
  param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic
  param_ptrs.push_back(time_delay);               // Time-delay

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS
  param_types.push_back(ParamBlock::POSE);       // Target pose T_WT0
  param_types.push_back(ParamBlock::POINT);      // Target point p_Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Target extrinsic T_T0Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Imu extrinsic T_C0S
  param_types.push_back(ParamBlock::EXTRINSIC);  // Camera extrinsic T_C0Ci
  param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic
  param_types.push_back(ParamBlock::TIME_DELAY); // Time-delay

  const Vec2 v_k = (z_k - z_km1) / ts2sec(ts_k - ts_km1);
  return std::make_shared<CalibCameraImuError>(PIXEL_VELOCITY,
                                               ts_km1,
                                               ts_k,
                                               camera,
                                               param_ptrs,
                                               param_types,
                                               z_k,
                                               v_k,
                                               covar);
}

std::shared_ptr<CalibCameraImuError>
CalibCameraImuError::create(const timestamp_t ts_km1,
                            const timestamp_t ts_k,
                            const std::shared_ptr<CameraGeometry> &camera,
                            const std::shared_ptr<ImuGeometry> &imu,
                            const std::shared_ptr<CalibTargetGeometry> &target,
                            double *sensor_pose_km1,
                            double *sensor_pose_k,
                            double *target_pose,
                            double *time_delay,
                            const int point_id,
                            const Vec2 &z_k,
                            const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(sensor_pose_km1); // Sensor pose T_WS at k-1
  param_ptrs.push_back(sensor_pose_k);   // Sensor pose T_WS at k
  param_ptrs.push_back(target_pose);     // Target pose T_WT0
  param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
  param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
  param_ptrs.push_back(imu->extrinsic.data());    // Imu extrinsic T_C0S
  param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
  param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic
  param_ptrs.push_back(time_delay);               // Time-delay

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k-1
  param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k
  param_types.push_back(ParamBlock::POSE);       // Target pose T_WT0
  param_types.push_back(ParamBlock::POINT);      // Target point p_Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Target extrinsic T_T0Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Imu extrinsic T_C0S
  param_types.push_back(ParamBlock::EXTRINSIC);  // Camera extrinsic T_C0Ci
  param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic
  param_types.push_back(ParamBlock::TIME_DELAY); // Time-delay

  const Vec2 v_k = Vec2::Zero(); // unused for POSE_INTERP
  return std::make_shared<CalibCameraImuError>(POSE_INTERP,
                                               ts_km1,
                                               ts_k,
                                               camera,
                                               param_ptrs,
                                               param_types,
                                               z_k,
                                               v_k,
                                               covar);
}

bool CalibCameraImuError::valid() const { return valid_; }

Mat2 CalibCameraImuError::get_covariance_matrix() const { return covar_; }

bool CalibCameraImuError::get_residuals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraImuError::get_reproj_error(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool CalibCameraImuError::eval(double const *const *params,
                               double *res,
                               double **jacs) const {
  valid_ = true;

  const int poff = (mode_ == PIXEL_VELOCITY) ? 0 : 1;
  const double dt = ts2sec(ts_k_ - ts_km1_);
  const double time_delay = params[7 + poff][0];
  const double alpha = time_delay / dt;

  // Compute T_WS based on mode
  Mat4 T_WS;
  Vec3 r_km1;
  Vec3 r_k;
  Vec3 w_k;
  Mat3 C_km1;
  Mat3 C_k;
  if (mode_ == PIXEL_VELOCITY) {
    T_WS = tf(params[0]);

  } else {
    const Mat4 T_WS_km1 = tf(params[0]);
    const Mat4 T_WS_k = tf(params[1]);

    r_km1 = tf_trans(T_WS_km1);
    r_k = tf_trans(T_WS_k);
    C_km1 = tf_rot(T_WS_km1);
    C_k = tf_rot(T_WS_k);

    const Vec3 r_delayed = r_k + alpha * (r_k - r_km1);
    const Mat3 C_k_km1 = C_k * C_km1.transpose();
    w_k = SO3::log(C_k_km1);
    const Mat3 C_delayed = C_k * SO3::exp(alpha * w_k);
    T_WS = tf(C_delayed, r_delayed);
  }

  // Map shared parameters
  const Mat4 T_WT0 = tf(params[1 + poff]);
  const Eigen::Map<const Vec3> p_Tj(params[2 + poff], 3);
  const Mat4 T_T0Tj = tf(params[3 + poff]);
  const Mat4 T_C0S = tf(params[4 + poff]);
  const Mat4 T_C0Ci = tf(params[5 + poff]);
  Eigen::Map<const VecX> intrinsic(params[6 + poff], 8);

  // Transform and project point to image plane
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_SW = tf_inv(T_WS);
  const Mat4 T_CiTj = T_CiC0 * T_C0S * T_SW * T_WT0 * T_T0Tj;
  const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
  const auto camera_model = camera_geometry_->camera_model;
  const Vec2i resolution = camera_geometry_->resolution;
  Vec2 z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
    Eigen::Map<Vec2> r(res);
    r.setZero();
    residuals_.setZero();
    return true;
  }

  // Residual based on mode
  Eigen::Map<Vec2> r(res);
  if (mode_ == PIXEL_VELOCITY) {
    const Vec2 z_corrected = z_k_ - (time_delay * v_k_);
    r = sqrt_info_ * (z_corrected - z_hat);
    residuals_ = z_corrected - z_hat;
  } else {
    r = sqrt_info_ * (z_k_ - z_hat);
    residuals_ = z_k_ - z_hat;
  }

  // Jacobians
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr) { return true; }

  if (!valid_) { return true; }

  // Common terms for projection Jacobian w.r.t. T_WS
  const Vec3 r_WS = tf_trans(T_WS);
  const Mat3 C_WS = tf_rot(T_WS);
  const Mat4 T_CiS = T_CiC0 * T_C0S;
  const Mat4 T_CiW = T_CiS * T_SW;
  const Mat3 C_CiW = tf_rot(T_CiW);
  const Vec3 p_W = tf_point(T_WT0 * T_T0Tj, p_Tj);

  const Mat<2, 3, Eigen::RowMajor> J_std_trans = Jhw * -C_CiW;
  const Mat<2, 3, Eigen::RowMajor> J_std_rot =
      Jhw * -C_CiW * skew(p_W - r_WS) * -C_WS;

  // -- Pose Jacobians (mode-specific)
  if (mode_ == PIXEL_VELOCITY) {
    if (jacs[0]) {
      Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
      J.setZero();
      J.block<2, 3>(0, 0) = J_std_trans;
      J.block<2, 3>(0, 3) = J_std_rot;
    }
  } else {
    const Mat3 R_alpha_mat = SO3::exp(alpha * w_k);
    const Mat3 J_r_alpha_w = SO3::right_jacobian(alpha * w_k);
    const Mat3 J_r_inv_w = SO3::right_jacobian_inv(w_k);

    if (jacs[0]) {
      Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
      J.setZero();

      const Mat3 d_rot = -J_r_alpha_w * alpha * J_r_inv_w * C_km1;
      J.block<2, 3>(0, 0) = J_std_trans * (-alpha);
      J.block<2, 3>(0, 3) = J_std_rot * d_rot;
    }
    if (jacs[1]) {
      Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[1]);
      J.setZero();

      const Mat3 d_rot =
          R_alpha_mat.transpose() + J_r_alpha_w * alpha * J_r_inv_w * C_km1;
      J.block<2, 3>(0, 0) = J_std_trans * (1.0 + alpha);
      J.block<2, 3>(0, 3) = J_std_rot * d_rot;
    }
  }

  // -- Jacobians w.r.t target pose T_WT0
  if (jacs[1 + poff]) {
    const Mat3 C_WT0 = tf_rot(T_WT0);
    const Vec3 p_T0 = tf_point(T_T0Tj, p_Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[1 + poff]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiW;
    J.block<2, 3>(0, 3) = Jhw * C_CiW * -C_WT0 * skew(p_T0);
  }

  // -- Jacobians w.r.t target point p_Tj
  if (jacs[2 + poff]) {
    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[2 + poff]);
    J.setZero();
    const Mat3 C_CiTj = tf_rot(T_CiTj);
    J = Jhw * C_CiTj;
  }

  // -- Jacobians w.r.t target extrinsic T_T0Tj
  if (jacs[3 + poff]) {
    const Mat4 T_CiT0 = T_CiC0 * T_C0S * T_SW * T_WT0;
    const Mat3 C_CiT0 = tf_rot(T_CiT0);
    const Mat3 C_T0Tj = tf_rot(T_T0Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[3 + poff]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiT0;
    J.block<2, 3>(0, 3) = Jhw * C_CiT0 * -C_T0Tj * skew(p_Tj);
  }

  // -- Jacobians w.r.t imu extrinsic T_C0S
  if (jacs[4 + poff]) {
    const Vec3 p_S = tf_point(T_WS.inverse() * T_WT0 * T_T0Tj, p_Tj);
    const Mat3 C_C0S = tf_rot(T_C0S);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[4 + poff]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiC0;
    J.block<2, 3>(0, 3) = Jhw * C_CiC0 * -C_C0S * skew(p_S);
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  if (jacs[5 + poff]) {
    const Vec3 p_C0 = tf_point(T_C0S * T_WS.inverse() * T_WT0 * T_T0Tj, p_Tj);
    const Vec3 r_C0Ci = tf_trans(T_C0Ci);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);
    const Mat3 C_C0Ci = C_CiC0.transpose();

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[5 + poff]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * -C_CiC0;
    J.block<2, 3>(0, 3) = Jhw * -C_CiC0 * skew(p_C0 - r_C0Ci) * -C_C0Ci;
  }

  // -- Jacobians w.r.t intrinsic
  if (jacs[6 + poff]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[6 + poff]);
    J.setZero();
    J = -1.0 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
  }

  // -- Time-delay Jacobian (mode-specific)
  if (jacs[7 + poff]) {
    Eigen::Map<Vec2> J(jacs[7 + poff]);
    J.setZero();
    if (mode_ == PIXEL_VELOCITY) {
      J = -sqrt_info_ * v_k_;

    } else {
      const Vec3 v_trans = (r_k - r_km1) / dt;
      const Mat3 J_r_alpha = SO3::right_jacobian(alpha * w_k);
      const Vec3 d_rot_dt = J_r_alpha * w_k / dt;
      J = J_std_trans * v_trans + J_std_rot * d_rot_dt;
    }
  }

  return true;
}

} // namespace cartesian
