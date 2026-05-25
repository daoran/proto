#pragma once
#include "../core/Core.hpp"
#include "imu/ImuParams.hpp"
#include "imu/ImuBuffer.hpp"

namespace cartesian {

struct ImuPreintegrate {
  // Pre-integrate relative position, velocity, rotation and biases
  Quat dq{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  Vec3 dr{0.0, 0.0, 0.0};      // Relative position
  Vec3 dv{0.0, 0.0, 0.0};      // Relative velocity
  Vec3 ba{0.0, 0.0, 0.0};      // Accel biase at i
  Vec3 bg{0.0, 0.0, 0.0};      // Gyro biase at i

  double Dt = 0.0;              // Preintegration time period [s]
  MatX state_F = eye(15);         // State jacobian
  MatX state_P = zeros(15, 15); // State covariance
  MatX sqrt_info = eye(15, 15);   // Square root information

  ImuPreintegrate() = delete;
  ImuPreintegrate(const ImuParams &imu_params, const ImuBuffer &imu_buffer);
  ~ImuPreintegrate() = default;

  /** Reset */
  void reset();
};

} // namespace cartesian
