#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuParams.hpp"
#include "imu/ImuBuffer.hpp"

namespace xyz {

// Forward declaration
class ImuError;
using ImuErrorPtr = std::shared_ptr<ImuError>;

/** Imu Error */
class ImuError : public ResidualBlock {
private:
  // Imu parameters and data
  const ImuParams imu_params_;
  const ImuBuffer imu_buffer_;

  // Pre-integrate relative position, velocity, rotation and biases
  mutable Quat dq_{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  mutable Vec3 dr_{0.0, 0.0, 0.0};      // Relative position
  mutable Vec3 dv_{0.0, 0.0, 0.0};      // Relative velocity
  Vec3 ba_{0.0, 0.0, 0.0};              // Accel biase at i
  Vec3 bg_{0.0, 0.0, 0.0};              // Gyro biase at i

  double Dt_ = 0.0;              // Preintegration time period [s]
  MatX state_F_ = I(15);         // State jacobian
  MatX state_P_ = zeros(15, 15); // State covariance
  mutable MatX sqrt_info_ = I(15, 15);   // Square root information

  /** Form noise matrix Q */
  MatX formQ();

  /** Form transiton matrix F */
  MatX formF(const int k,
             const Quat &dq_i,
             const Quat &dq_j,
             const Vec3 &ba_i,
             const Vec3 &bg_i,
             const double dt);

  /** Form matrix G */
  MatX formG(const int k,
             const Quat &dq_i,
             const Quat &dq_j,
             const Vec3 &ba_i,
             const double dt);

  /** Propagate IMU measurements */
  void propagate();

public:
  /** Constructor */
  ImuError(const std::vector<double *> &param_ptrs,
           const std::vector<ParamBlock::Type> &param_types,
           const ImuParams &imu_params,
           const ImuBuffer &imu_buffer);

  /** Manually set square root information */
  void setSqrtInfo(const MatX &sqrt_info);

  /** Return state transition matrix F */
  MatX getMatrixF() const;

  /** Return state transition matrix P */
  MatX getMatrixP() const;

  /** Return relative rotation dq */
  Quat getRelativeRotation() const;

  /** Return relative position dr */
  Vec3 getRelativePosition() const;

  /** Return relative velocity dv */
  Vec3 getRelativeVelocity() const;

  /** Create residual block */
  static std::shared_ptr<ImuError> create(const ImuParams &imu_params,
                                          const ImuBuffer &imu_buffer,
                                          double *pose_i,
                                          double *sb_i,
                                          double *pose_j,
                                          double *sb_j);

  /** Evaluate with minimial Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace xyz
