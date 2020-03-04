#include "proto/estimation/imu.hpp"

namespace proto {

mat3_t so3_exp(const vec3_t &phi) {
  const double phi_norm = phi.norm();
  const double phi_norm_sq = phi_norm * phi_norm;
  const vec3_t phi_skew = skew(phi);
  const vec3_t phi_skew_sq = phi_skew * phi_skew;

  // Small angle approximation
  if (phi_norm < 1e-3) {
    return I(3) + phi_skew;
  }

  // Exponential Map from so(3) -> SO(3)
  mat3_t C;
  C = I(3);
  C += (sin(phi_norm) / phi_norm) * phi_skew;
  C += ((1 - cos(phi_norm)) / phi_norm_sq) * phi_skew_sq;

  return C;
}

vec3_t SO3_log(const mat3_t &C) {
  const auto phi = acos(C.trace() - 1 / 2);
  return phi * (C * C.transpose()) / (2 * sin(phi));
}

mat3_t SO3_rjac(const vec3_t &psi) {
  const double psi_norm = psi.norm();
  const double psi_norm_sq = psi_norm * psi_norm;
  const double psi_norm_cube = psi_norm_sq * psi_norm;
  const mat3_t psi_skew = skew(psi);
  const mat3_t psi_skew_sq = psi_skew * psi_skew;

  mat3_t J = I(3);
  J -= ((1 - cos(psi_norm)) / psi_norm_sq) * psi_skew;
  J += (psi_norm - sin(psi_norm)) / (psi_norm_cube) * psi_skew_sq;
  return J;
}

void imu_update(imu_t &imu,
                const vec3_t &a_S,
                const vec3_t &w_S,
                const double dt,
                const vec3_t &up) {
  const double dt_sq = dt * dt;       // Time difference [s]
  const vec3_t g = -1 * up * imu.g;   // Gravity vector [ms^-2]

  const vec3_t b_a = imu.b_a;         // Accel bias
  const vec3_t b_g = imu.b_g;         // Gyro bias
  const vec3_t n_a = zeros(3, 1);     // Accel Noise
  const vec3_t n_g = zeros(3, 1);     // Gyro Noise

  const mat3_t C_WS = imu.C_WS;       // IMU orientation
  const vec3_t v_WS = imu.v_WS;       // IMU velocity
  const vec3_t p_WS = imu.p_WS;       // IMU position

  // Nominal gyro and accel measurements
  const vec3_t w_St = w_S - b_g - n_g;
  const vec3_t a_St = a_S - b_a - n_a;

  // Update IMU state
  imu.C_WS = C_WS * so3_exp(w_St * dt);
  imu.v_WS += (C_WS * a_St * dt) + (g * dt);
  imu.p_WS += (v_WS * dt) + (0.5 * C_WS * a_St * dt_sq) + (0.5 * g * dt_sq);
}

}  // namespace proto
