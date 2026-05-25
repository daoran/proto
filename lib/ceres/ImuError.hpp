#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuParams.hpp"
#include "imu/ImuBuffer.hpp"

namespace cartesian {

// Forward declaration
struct ImuError;
using ImuErrorPtr = std::shared_ptr<ImuError>;

/** Imu Error */
struct ImuError : ResidualBlock {
  // Imu parameters and data
  const ImuParams imu_params;
  const ImuBuffer imu_buffer;

  // Pre-integrate relative position, velocity, rotation and biases
  Quat dq_{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  Vec3 dr_{0.0, 0.0, 0.0};      // Relative position
  Vec3 dv_{0.0, 0.0, 0.0};      // Relative velocity
  Vec3 ba_{0.0, 0.0, 0.0};      // Accel biase at i
  Vec3 bg_{0.0, 0.0, 0.0};      // Gyro biase at i

  double Dt_ = 0.0;              // Preintegration time period [s]
  MatX state_F_ = I(15);         // State jacobian
  MatX state_P_ = zeros(15, 15); // State covariance
  MatX sqrt_info_ = I(15, 15);   // Square root information

  /** Form noise matrix Q */
  MatX form_Q() const;

  /** Form transiton matrix F */
  MatX form_F(const int k,
              const Quat &dq_i,
              const Quat &dq_j,
              const Vec3 &ba_i,
              const Vec3 &bg_i,
              const double dt) const;

  /** Form matrix G */
  MatX form_G(const int k,
              const Quat &dq_i,
              const Quat &dq_j,
              const Vec3 &ba_i,
              const double dt) const;

  /** Propagate IMU measurements */
  void propagate();

  /** Constructor */
  ImuError(const std::vector<double *> &param_ptrs,
           const std::vector<ParamBlock::Type> &param_types,
           const ImuParams &imu_params_,
           const ImuBuffer &imu_buffer_);

  /** Manually set square root information */
  void set_sqrt_info(const MatX &sqrt_info);

  /** Return state transition matrix F */
  MatX get_matrix_f() const;

  /** Return state transition matrix P */
  MatX get_matrix_p() const;

  /** Return relative rotation dq */
  Quat get_relative_rotation() const;

  /** Return relative position dr */
  Vec3 get_relative_position() const;

  /** Return relative velocity dv */
  Vec3 get_relative_velocity() const;

  /** Create residual block */
  static std::shared_ptr<ImuError> create(const ImuParams &imu_params_,
                                          const ImuBuffer &imu_buffer_,
                                          double *pose_i,
                                          double *sb_i,
                                          double *pose_j,
                                          double *sb_j);

  /** Evaluate with minimial Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace cartesian
