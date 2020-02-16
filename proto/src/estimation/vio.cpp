#include "proto/estimation/vio.hpp"

namespace proto {

imu_model_t::imu_model_t() : size{15} {}

void imu_model_setup(imu_model_t &imu) {
  // // clang-format off
  // // Set estimate covariance matrix
  // vecx_t init_var = zeros(15, 1);
  // init_var << config.q_init_var,
  //             config.bg_init_var,
  //             config.v_init_var,
  //             config.ba_init_var,
  //             config.p_init_var;
  // imu.P = init_var.asDiagonal();
  //
  // // Set noise covariance matrix
  // vecx_t n_imu = zeros(12, 1);
  // n_imu << config.w_var,
  //         config.dbg_var,
  //         config.a_var,
  //         config.dba_var;
  // imu.Q = n_imu.asDiagonal();
  // // clang-format on
}

matx_t imu_model_F(const vec3_t &w_hat, const vec4_t &q_hat, const vec3_t &a_hat) {
  matx_t F = zeros(15);

  // -- First row block --
  F.block<3, 3>(0, 0) = -skew(w_hat);
  F.block<3, 3>(0, 3) = -I(3);
  // -- Third Row block --
  F.block<3, 3>(6, 0) = -quat2rot(q_hat).transpose() * skew(a_hat);
  F.block<3, 3>(6, 9) = -quat2rot(q_hat).transpose();
  // -- Fifth Row block --
  F.block<3, 3>(12, 6) = I(3);

  return F;
}

matx_t imu_model_G(imu_model_t &imu, const vec4_t &q_hat) {
  matx_t G = zeros(15, 12);

  // -- First row block --
  G.block<3, 3>(0, 0) = -I(3);
  // -- Second row block --
  G.block<3, 3>(3, 3) = I(3);
  // -- Third row block --
  G.block<3, 3>(6, 6) = -quat2rot(q_hat).transpose();
  // -- Fourth row block --
  G.block<3, 3>(9, 9) = I(3);

  return G;
}

void imu_model_propagate(imu_model_t &imu, const vec3_t &a_m, const vec3_t &w_m, const double dt) {
  // Calculate new accel and gyro estimates
  const vec3_t a_hat = a_m - imu.b_a;
  const vec3_t w_hat = w_m - imu.b_g;

  // Build the transition F and input G matrices
  const matx_t F = imu_model_F(w_hat, imu.q_IG, a_hat);
  const matx_t G = imu_model_G(imu.q_IG);

  // Propagate IMU states
  // clang-format off
  if (imu.rk4) {
    // Quaternion zeroth order integration
    const vec4_t dq_dt = quatzoi(imu.q_IG, w_hat, dt);
    const vec4_t dq_dt2 = quatzoi(imu.q_IG, w_hat, dt * 0.5);
    const Mat3 dR_dt_transpose = quat2rot(dq_dt).transpose();
    const Mat3 dR_dt2_transpose = quat2rot(dq_dt2).transpose();

    // 4th order Runge-Kutta
    // -- k1 = f(tn, yn)
    const vec3_t k1_v_dot = quat2rot(imu.q_IG).transpose() * a_hat + imu.g_G;
    const vec3_t k1_p_dot = imu.v_G;
    // -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
    const vec3_t k1_v = imu.v_G + k1_v_dot * dt / 2.0;
    const vec3_t k2_v_dot = dR_dt2_transpose * a_hat + imu.g_G;
    const vec3_t k2_p_dot = k1_v;
    // -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
    const vec3_t k2_v = imu.v_G + k2_v_dot * dt / 2;
    const vec3_t k3_v_dot = dR_dt2_transpose * a_hat + imu.g_G;
    const vec3_t k3_p_dot = k2_v;
    // -- k4 = f(tn + dt, yn + k3)
    const vec3_t k3_v = imu.v_G + k3_v_dot * dt;
    const vec3_t k4_v_dot = dR_dt_transpose * a_hat + imu.g_G;
    const vec3_t k4_p_dot = k3_v;
    // -- yn + 1 = yn + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    imu.q_IG = quatnormalize(dq_dt);
    imu.v_G = imu.v_G + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    imu.p_G = imu.p_G + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

  } else {
    // -- Orientation
    imu.q_IG += 0.5 * Omega(w_hat) * imu.q_IG * dt;
    imu.q_IG = quatnormalize(imu.q_IG);
    // -- Velocity
    imu.v_G += (quat2rot(imu.q_IG).transpose() * a_hat + imu.g_G) * dt;
    // -- Position
    imu.p_G += v_G * dt;

  }
  // clang-format on

  // Update covariance
  // clang-format off
  // -- Approximate matrix exponential to the 3rd order using the power series,
  //    which can be considered to be accurate enough assuming dt is within
  //    0.01s.
  const matx_t F_dt = F * dt;
  const matx_t F_dt_sq = F_dt * F_dt;
  const matx_t F_dt_cube = F_dt_sq * F_dt;
  imu.Phi = I(imu.size) + F_dt + 0.5 * F_dt_sq + (1.0 / 6.0) * F_dt_cube;
  // -- Update
  // imu.Q = imu.Phi * G * imu.Q * G.transpose() * imu.Phi.transpose() * dt;
  // imu.P = imu.Phi * imu.P * imu.Phi.transpose() + imu.Q;
  // imu.P = imu.Phi * imu.P * imu.Phi.transpose() + (G * imu.Q * G.transpose()) * dt;
  imu.P = imu.Phi * imu.P * imu.Phi.transpose() + (imu.Phi * G * imu.Q * G.transpose() * imu.Phi.transpose()) * dt;
  imu.P = enforce_psd(P);
  // clang-format on

  // TODO: Modify transition matrix according to OC-EKF
}

void imu_model_correct(const vecx_t &dx) {
  // Split dx into its own components
  const vec3_t dtheta_IG = dx.segment(0, 3);
  const vec3_t db_g = dx.segment(3, 3);
  const vec3_t dv_G = dx.segment(6, 3);
  const vec3_t db_a = dx.segment(9, 3);
  const vec3_t dp_G = dx.segment(12, 3);

  // Time derivative of quaternion (small angle approx)
  const vec4_t dq_IG = quatsmallangle(dtheta_IG);

  // Correct IMU state
  imu.q_IG = quatnormalize(quatlcomp(dq_IG) * imu.q_IG);
  imu.b_g = imu.b_g + db_g;
  imu.v_G = imu.v_G + dv_G;
  imu.b_a = imu.b_a + db_a;
  imu.p_G = imu.p_G + dp_G;
}

} // namespace proto
