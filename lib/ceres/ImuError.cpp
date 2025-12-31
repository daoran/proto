#include "ImuError.hpp"

namespace xyz {

// static double quat_norm(const Vec4 &q) {
//   const double qw2 = q[0] * q[0];
//   const double qx2 = q[1] * q[1];
//   const double qy2 = q[2] * q[2];
//   const double qz2 = q[3] * q[3];
//   const double norm = sqrt(qw2 + qx2 + qy2 + qz2);
//   return norm;
// }
//
// static Vec4 quat_normalize(const Vec4 &q) {
//   const double qw = q[0];
//   const double qx = q[1];
//   const double qy = q[2];
//   const double qz = q[3];
//   const double n = quat_norm(q);
//   return Vec4{qw / n, qx / n, qy / n, qz / n};
// }
//
// static Vec4 quat_conj(const Vec4 &q) {
//   const double qw = q[0];
//   const double qx = q[1];
//   const double qy = q[2];
//   const double qz = q[3];
//   const Vec4 q_inv{qw, -qx, -qy, -qz};
//   return q_inv;
// }
//
// static Vec4 quat_inv(const Vec4 &q) {
//   // Invert quaternion
//   return quat_conj(q);
// }
//
// static Mat4 quat_left(const Vec4 &q) {
//   const double qw = q[0];
//   const double qx = q[1];
//   const double qy = q[2];
//   const double qz = q[3];
//
//   // clang-format off
//   Mat4 Qleft;
//   Qleft <<  qw, -qx, -qy, -qz,
//             qx,  qw, -qz,  qy,
//             qy,  qz,  qw, -qx,
//             qz, -qy,  qx,  qw;
//   // clang-format on
//   return Qleft;
// }
//
// static Mat4 quat_right(const Vec4 &q) {
//   const double qw = q[0];
//   const double qx = q[1];
//   const double qy = q[2];
//   const double qz = q[3];
//
//   // clang-format off
//   Mat4 Qright;
//   Qright << qw, -qx, -qy, -qz,
//             qx,  qw,  qz, -qy,
//             qy, -qz,  qw,  qx,
//             qz,  qy, -qx,  qw;
//   // clang-format on
//   return Qright;
// }
//
// static Vec4 quat_mul(const Vec4 &p, const Vec4 &q) {
//   // P * q
//   return quat_left(p) * q;
// }
//
// static Vec3 quat_rot(const Vec4 &q, const Vec3 &pt) {
//   const double qw = q[0];
//   const double qx = q[1];
//   const double qy = q[2];
//   const double qz = q[3];
//
//   // p_new = q * p * q_conj
//   const Vec4 q_conj{qw, -qx, -qy, -qz};
//   const Vec4 p{0.0, pt.x(), pt.y(), pt.z()};
//   const Vec4 p_new = quat_mul(quat_mul(q, p), q_conj);
//   return Vec3{p_new.x(), p_new.y(), p_new.z()};
// }
//
// /**
//  * Convert quaternion to 3x3 rotation matrix.
//  * Source:
//  * Blanco, Jose-Luis. "A tutorial on se (3) transformation parameterizations
//  * and on-manifold optimization." University of Malaga, Tech. Rep 3 (2010): 6.
//  * [Page 18, Equation (2.20)]
//  */
// static Mat3 quat2rot(const Vec4 &q) {
//   const auto qw = q[0];
//   const auto qx = q[1];
//   const auto qy = q[2];
//   const auto qz = q[3];
//
//   const auto qx2 = qx * qx;
//   const auto qy2 = qy * qy;
//   const auto qz2 = qz * qz;
//   const auto qw2 = qw * qw;
//
//   const auto C11 = qw2 + qx2 - qy2 - qz2;
//   const auto C12 = 2.0 * (qx * qy - qw * qz);
//   const auto C13 = 2.0 * (qx * qz + qw * qy);
//   const auto C21 = 2.0 * (qx * qy + qw * qz);
//   const auto C22 = qw2 - qx2 + qy2 - qz2;
//   const auto C23 = 2.0 * (qy * qz - qw * qx);
//   const auto C31 = 2.0 * (qx * qz - qw * qy);
//   const auto C32 = 2.0 * (qy * qz + qw * qx);
//   const auto C33 = qw2 - qx2 - qy2 + qz2;
//
//   Mat3 rot;
//   rot << C11, C12, C13, C21, C22, C23, C31, C32, C33;
//   return rot;
// }

MatX ImuError::formQ() {
  MatX Q = zeros(18, 18);
  Q.block<3, 3>(0, 0) = pow(imu_params_.noise_acc, 2) * I(3);
  Q.block<3, 3>(3, 3) = pow(imu_params_.noise_gyr, 2) * I(3);
  Q.block<3, 3>(6, 6) = pow(imu_params_.noise_acc, 2) * I(3);
  Q.block<3, 3>(9, 9) = pow(imu_params_.noise_gyr, 2) * I(3);
  Q.block<3, 3>(12, 12) = pow(imu_params_.noise_ba, 2) * I(3);
  Q.block<3, 3>(15, 15) = pow(imu_params_.noise_bg, 2) * I(3);
  return Q;
}

MatX ImuError::formF(const int k,
                     const Quat &dq_i,
                     const Quat &dq_j,
                     const Vec3 &ba_i,
                     const Vec3 &bg_i,
                     const double dt) {
  const Vec3 w_k = imu_buffer_.getGyr(k);
  const Vec3 w_kp1 = imu_buffer_.getGyr(k + 1);
  const Vec3 a_k = imu_buffer_.getAcc(k);
  const Vec3 a_kp1 = imu_buffer_.getAcc(k + 1);
  const Mat3 gyr_x = skew(0.5 * (w_k + w_kp1) - bg_i);
  const Mat3 acc_i_x = skew(a_k - ba_i);
  const Mat3 acc_j_x = skew(a_kp1 - ba_i);
  const Mat3 dC_i = dq_i.toRotationMatrix();
  const Mat3 dC_j = dq_j.toRotationMatrix();
  const double dt_sq = dt * dt;

  // Form transition matrix F
  MatX F = zeros(15, 15);
  // -- F row block 1
  F.block<3, 3>(0, 0) = I(3);
  F.block<3, 3>(0, 3) = -0.25 * dC_i * acc_i_x * dt_sq;
  F.block<3, 3>(0, 3) += -0.25 * dC_j * acc_j_x * (I(3) - gyr_x * dt) * dt_sq;
  F.block<3, 3>(0, 6) = I(3) * dt;
  F.block<3, 3>(0, 9) = -0.25 * (dC_i + dC_j) * dt_sq;
  F.block<3, 3>(0, 12) = 0.25 * -dC_j * acc_j_x * dt_sq * -dt;
  // -- F row block 2
  F.block<3, 3>(3, 3) = I(3) - gyr_x * dt;
  F.block<3, 3>(3, 12) = -I(3) * dt;
  // -- F row block 3
  F.block<3, 3>(6, 3) = -0.5 * dC_i * acc_i_x * dt;
  F.block<3, 3>(6, 3) += -0.5 * dC_j * acc_j_x * (I(3) - gyr_x * dt) * dt;
  F.block<3, 3>(6, 6) = I(3);
  F.block<3, 3>(6, 9) = -0.5 * (dC_i + dC_j) * dt;
  F.block<3, 3>(6, 12) = 0.5 * -dC_j * acc_j_x * dt * -dt;
  // -- F row block 4
  F.block<3, 3>(9, 9) = I(3);
  // -- F row block 5
  F.block<3, 3>(12, 12) = I(3);

  return F;
}

MatX ImuError::formG(const int k,
                     const Quat &dq_i,
                     const Quat &dq_j,
                     const Vec3 &ba_i,
                     const double dt) {
  const Vec3 a_k = imu_buffer_.getAcc(k);
  const Mat3 acc_i_x = skew(a_k - ba_i);
  const Mat3 dC_i = dq_i.toRotationMatrix();
  const Mat3 dC_j = dq_j.toRotationMatrix();
  const double dt_sq = dt * dt;

  MatX G = zeros(15, 18);
  // -- G row block 1
  G.block<3, 3>(0, 0) = 0.25 * dC_i * dt_sq;
  G.block<3, 3>(0, 3) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
  G.block<3, 3>(0, 6) = 0.25 * dC_j * acc_i_x * dt_sq;
  G.block<3, 3>(0, 9) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
  // -- G row block 2
  G.block<3, 3>(3, 3) = I(3) * dt;
  G.block<3, 3>(3, 9) = I(3) * dt;
  // -- G row block 3
  G.block<3, 3>(6, 0) = 0.5 * dC_i * dt;
  G.block<3, 3>(6, 3) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
  G.block<3, 3>(6, 6) = 0.5 * dC_j * dt;
  G.block<3, 3>(6, 9) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
  // -- G row block 4
  G.block<3, 3>(9, 12) = I(3) * dt;
  // -- G row block 5
  G.block<3, 3>(12, 15) = I(3) * dt;

  return G;
}

void ImuError::propagate() {
  // Noise matrix Q
  const MatX &Q = formQ();

  // Pre-integrate imu measuremenets
  for (int k = 0; k < (imu_buffer_.getNumMeasurements() - 1); k++) {
    // Timestep
    const timestamp_t ts_i = imu_buffer_.getTimestamp(k);
    const timestamp_t ts_j = imu_buffer_.getTimestamp(k + 1);
    const double dt = ts2sec(ts_j - ts_i);
    const double dt_sq = dt * dt;

    // Setup
    const Quat dq_i = dq_;
    const Vec3 dr_i = dr_;
    const Vec3 dv_i = dv_;
    const Vec3 ba_i = ba_;
    const Vec3 bg_i = bg_;

    // Gyroscope measurement
    const Vec3 w_k = imu_buffer_.getGyr(k);
    const Vec3 w_kp1 = imu_buffer_.getGyr(k + 1);
    const Vec3 w = 0.5 * (w_k + w_kp1) - bg_i;
    const Quat dq_perturb{1.0,
                          w.x() * dt / 2.0,
                          w.y() * dt / 2.0,
                          w.z() * dt / 2.0};

    // Accelerometer measurement
    const Vec3 a_k = imu_buffer_.getAcc(k);
    const Vec3 a_kp1 = imu_buffer_.getAcc(k + 1);
    const Vec3 acc_i = dq_i * (a_k - ba_i);
    const Vec3 acc_j = (dq_i * dq_perturb) * (a_kp1 - ba_i);
    const Vec3 a = 0.5 * (acc_i + acc_j);

    // Propagate IMU state using mid-point method
    const Quat dq_j = dq_i * dq_perturb;
    const Vec3 dr_j = dr_i + dv_i * dt + 0.5 * a * dt_sq;
    const Vec3 dv_j = dv_i + a * dt;
    const Vec3 ba_j = ba_i;
    const Vec3 bg_j = bg_i;

    // Continuous time transition matrix F and input matrix G
    const MatX F = formF(k, dq_i, dq_j, ba_i, bg_i, dt);
    const MatX G = formG(k, dq_i, dq_j, ba_i, dt);

    // Map results
    dq_ = dq_j;
    dr_ = dr_j;
    dv_ = dv_j;
    ba_ = ba_j;
    bg_ = bg_j;

    // Update
    state_F_ = F * state_F_;
    state_P_ = F * state_P_ * F.transpose() + G * Q * G.transpose();
    Dt_ += dt;
  }

  // Enforce semi-positive-definite on the state covariance matrix
  state_P_ = (state_P_ + state_P_.transpose()) / 2.0;
  sqrt_info_ = state_P_.inverse().llt().matrixU();
}

ImuError::ImuError(const std::vector<double *> &param_ptrs,
                   const std::vector<ParamBlock::Type> &param_types,
                   const ImuParams &imu_params,
                   const ImuBuffer &imu_buffer)
    : ResidualBlock{"ImuError", param_ptrs, param_types, 15},
      imu_params_{imu_params}, imu_buffer_{imu_buffer} {
  assert(imu_params_.noise_acc > 0);
  assert(imu_params_.noise_gyr > 0);
  assert(imu_params_.noise_ba > 0);
  assert(imu_params_.noise_bg > 0);
  assert(imu_params_.g.norm() > 0);

  Eigen::Map<const VecX> sb_i(param_ptrs[1], 9);
  ba_ = Vec3{sb_i[3], sb_i[4], sb_i[5]};
  bg_ = Vec3{sb_i[6], sb_i[7], sb_i[8]};

  propagate();
}

void ImuError::setSqrtInfo(const MatX &sqrt_info) { sqrt_info_ = sqrt_info; }

MatX ImuError::getMatrixF() const { return state_F_; }

MatX ImuError::getMatrixP() const { return state_P_; }

Quat ImuError::getRelativeRotation() const { return dq_; }

Vec3 ImuError::getRelativePosition() const { return dr_; }

Vec3 ImuError::getRelativeVelocity() const { return dv_; }

std::shared_ptr<ImuError> ImuError::create(const ImuParams &imu_params,
                                           const ImuBuffer &imu_buffer,
                                           double *pose_i,
                                           double *sb_i,
                                           double *pose_j,
                                           double *sb_j) {
  std::vector<double *> param_ptrs = {pose_i, sb_i, pose_j, sb_j};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES,
                                               ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES};
  return std::make_shared<ImuError>(param_ptrs,
                                    param_types,
                                    imu_params,
                                    imu_buffer);
}

bool ImuError::eval(double const *const *params,
                    double *res,
                    double **jacs) const {
  // Map parameters out
  const Mat4 T_i = tf(params[0]);
  Eigen::Map<const VecX> sb_i(params[1], 9);
  const Mat4 T_j = tf(params[2]);
  Eigen::Map<const VecX> sb_j(params[3], 9);

  const Quat q_i = tf_quat(T_i);
  const Mat3 C_i = tf_rot(T_i);
  const Vec3 r_i = tf_trans(T_i);
  const Vec3 v_i = sb_i.segment<3>(0);
  const Vec3 ba_i = sb_i.segment<3>(3);
  const Vec3 bg_i = sb_i.segment<3>(6);

  const Quat q_j = tf_quat(T_j);
  const Mat3 C_j = tf_rot(T_j);
  const Vec3 r_j = tf_trans(T_j);
  const Vec3 v_j = sb_j.segment<3>(0);
  const Vec3 ba_j = sb_j.segment<3>(3);
  const Vec3 bg_j = sb_j.segment<3>(6);

  // Correct the relative position, velocity and orientation
  // -- Extract jacobians from error-state jacobian
  const Mat3 dr_dba = state_F_.block<3, 3>(0, 9);
  const Mat3 dr_dbg = state_F_.block<3, 3>(0, 12);
  const Mat3 dq_dbg = state_F_.block<3, 3>(6, 12);
  const Mat3 dv_dba = state_F_.block<3, 3>(3, 9);
  const Mat3 dv_dbg = state_F_.block<3, 3>(3, 12);
  const Vec3 dba = ba_i - ba_;
  const Vec3 dbg = bg_i - bg_;
  const Vec3 dhtheta = 0.5 * (dq_dbg * dbg);
  const Quat dq_theta{1.0, dhtheta.x(), dhtheta.y(), dhtheta.z()};

  // -- Correct the relative position, velocity and rotation
  const auto dr = dr_ + dr_dba * dba + dr_dbg * dbg;
  const auto dv = dv_ + dv_dba * dba + dv_dbg * dbg;
  const auto dq = dq_ * quat_delta(dq_dbg * dbg);

  // Form residuals
  const Vec3 g = imu_params_.g;
  const double Dt_sq = Dt_ * Dt_;
  const Mat3 C_iT = C_i.transpose();
  const Quat dq_inv = dq.inverse();
  const Quat qi_inv = q_i.inverse();

  const Vec3 dr_est = C_iT * ((r_j - r_i) - (v_i * Dt_) + (0.5 * g * Dt_sq));
  const Vec3 dv_est = C_iT * ((v_j - v_i) + (g * Dt_));
  const Quat dq_est = qi_inv * q_j;

  // <<<<<<< Updated upstream
  const Vec3 err_pos = dr_est - dr;
  const Vec3 err_vel = dv_est - dv;
  const Vec3 err_rot = 2.0 * (dq_inv * dq_est).vec();
  // =======
  //   const Vec3 err_pos = dr_meas - dr;
  //   const Vec3 err_vel = dv_meas - dv;
  //   const Vec3 err_rot = 2.0 * (dq.inverse() * (q_i.inverse() * q_j)).vec();
  // >>>>>>> Stashed changes
  const Vec3 err_ba = ba_j - ba_i;
  const Vec3 err_bg = bg_j - bg_i;

  Eigen::Map<VecX> r(res, 15);
  r.segment(0, 3) = err_pos;
  r.segment(3, 3) = err_vel;
  r.segment(6, 3) = err_rot;
  r.segment(9, 3) = err_ba;
  r.segment(12, 3) = err_bg;
  r = sqrt_info_ * r;

  // Form Jacobians
  if (jacs == nullptr) {
    return true;
  }

  // Jacobian w.r.t pose i
  if (jacs[0]) {
    const Quat q_ji{C_j.transpose() * C_i};
    const Mat4 q_left_dq_right = quat_left(q_ji) * quat_right(dq);
    const Mat3 dtheta_dCi = -(q_left_dq_right).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_iT;        // dr w.r.t r_i
    J.block<3, 3>(0, 3) = skew(dr_est); // dr w.r.t C_i
    J.block<3, 3>(3, 3) = skew(dv_est); // dv w.r.t C_i
    J.block<3, 3>(6, 3) = dtheta_dCi;   // dtheta w.r.t C_i
    J = sqrt_info_ * J;
  }

  // Jacobian w.r.t speed and biases i
  if (jacs[1]) {
    const Quat dq_ji{C_j.transpose() * C_i * dq.toRotationMatrix()};
    const Mat3 dQ_left_xyz = quat_left(dq_ji).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 9, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_i.transpose() * Dt_; // dr w.r.t v_i
    J.block<3, 3>(0, 3) = -dr_dba;                // dr w.r.t ba
    J.block<3, 3>(0, 6) = -dr_dbg;                // dr w.r.t bg
    J.block<3, 3>(3, 0) = -C_i.transpose();       // dv w.r.t v_i
    J.block<3, 3>(3, 3) = -dv_dba;                // dv w.r.t ba
    J.block<3, 3>(3, 6) = -dv_dbg;                // dv w.r.t bg
    J.block<3, 3>(6, 6) = -dQ_left_xyz * dq_dbg;  // dtheta w.r.t C_i
    J.block<3, 3>(9, 3) = -I(3);
    J.block<3, 3>(12, 6) = -I(3);
    J = sqrt_info_ * J;
  }

  // Jacobian w.r.t. pose j
  if (jacs[2]) {
    const Quat error_rot = dq.inverse() * (q_i.inverse() * q_j);
    const Mat3 dtheta_dCj = quat_left(error_rot).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 6, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();
    J.block<3, 3>(0, 0) = C_i.transpose(); // dr w.r.t r_j
    J.block<3, 3>(6, 3) = dtheta_dCj;      // dtheta w.r.t C_j
    J = sqrt_info_ * J;
  }

  //  Jacobian w.r.t. speed and biases j
  if (jacs[3]) {
    Eigen::Map<Mat<15, 9, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();
    J.block<3, 3>(3, 0) = C_i.transpose(); // dv w.r.t v_j
    J.block<3, 3>(9, 3) = I(3);
    J.block<3, 3>(12, 6) = I(3);
    J = sqrt_info_ * J;
  }

  return true;
}

} // namespace xyz
