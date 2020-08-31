#ifndef PROTO_ESTIMATION_FILTER_HPP
#define PROTO_ESTIMATION_FILTER_HPP

#include "core.hpp"

namespace proto {

/**
 * A simple complementary filter that uses `gyro` and `accel` measurements to
 * estimate the attitude in `roll` and `pitch`. Where `dt` is the update
 * rate of the `gyro` measurements.
 **/
void complementary_filter(const vec3_t &gyro,
                          const vec3_t &accel,
                          const real_t dt,
                          real_t &roll,
                          real_t &pitch) {

  // Calculate pitch and roll using gyroscope
  const real_t gyro_roll = (gyro(0) * dt) + roll;
  const real_t gyro_pitch = (gyro(1) * dt) + pitch;

  // Calculate pitch and roll using accelerometer
  const real_t x = accel(0);
  const real_t y = accel(1);
  const real_t z = accel(2);
  const real_t accel_pitch = (atan(x / sqrt(y * y + z * z))) * 180 / M_PI;
  const real_t accel_roll = (atan(y / sqrt(x * y + z * z))) * 180 / M_PI;

  // Complimentary filter
  pitch = (0.98 * gyro_pitch) + (0.02 * accel_pitch);
  roll = (0.98 * gyro_roll) + (0.02 * accel_roll);
}

struct imu_t {
  mat3_t C_WS = I(3);
  vec3_t p_WS{0.0, 0.0, 0.0};
  vec3_t v_WS{0.0, 0.0, 0.0};
  vec3_t b_a{0.0, 0.0, 0.0};
  vec3_t b_g{0.0, 0.0, 0.0};
};

void imu_update(imu_t &imu,
                const vec3_t &a_S,
                const vec3_t &w_S,
                const real_t dt,
                const vec3_t &g = vec3_t{0.0, 0.0, 9.81}) {
  const real_t dt_sq = dt * dt;       // Time difference [s]
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
  imu.C_WS = C_WS * lie::Exp(w_St * dt);
  imu.v_WS += (C_WS * a_St * dt) + (g * dt);
  imu.p_WS += (v_WS * dt) + (0.5 * C_WS * a_St * dt_sq) + (0.5 * g * dt_sq);
}

void imu_update(const mat3_t &C_WS_i,
                const vec3_t &v_WS_i,
                const vec3_t &p_WS_i,
                const vec3_t &b_g_i,
                const vec3_t &b_a_i,
                mat3_t &C_WS_j,
                vec3_t &v_WS_j,
                vec3_t &p_WS_j,
                vec3_t &b_g_j,
                vec3_t &b_a_j);

} // namespace proto
#endif // PROTO_ESTIMATION_FILTER_HPP
