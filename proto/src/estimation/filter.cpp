#include "proto/estimation/filter.hpp"

namespace proto {

void complementary_filter(const vec3_t &gyro,
                          const vec3_t &accel,
                          const double dt,
                          double &roll,
                          double &pitch) {

  // Calculate pitch and roll using gyroscope
  const double gyro_roll = (gyro(0) * dt) + roll;
  const double gyro_pitch = (gyro(1) * dt) + pitch;

  // Calculate pitch and roll using accelerometer
  const double x = accel(0);
  const double y = accel(1);
  const double z = accel(2);
  const double accel_pitch = (atan(x / sqrt(y * y + z * z))) * 180 / M_PI;
  const double accel_roll = (atan(y / sqrt(x * y + z * z))) * 180 / M_PI;

  // Complimentary filter
  pitch = (0.98 * gyro_pitch) + (0.02 * accel_pitch);
  roll = (0.98 * gyro_roll) + (0.02 * accel_roll);
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
  imu.C_WS = C_WS * lie::Exp(w_St * dt);
  imu.v_WS += (C_WS * a_St * dt) + (g * dt);
  imu.p_WS += (v_WS * dt) + (0.5 * C_WS * a_St * dt_sq) + (0.5 * g * dt_sq);
}

} // namespace proto
