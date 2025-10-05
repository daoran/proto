#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuParams.hpp"
#include "imu/ImuBuffer.hpp"

namespace xyz {

/** Imu Error */
class ImuError : public ResidualBlock {
private:
  // Imu parameters and data
  const ImuParams imu_params_;
  const ImuBuffer imu_buffer_;

  // Pre-integrate relative position, velocity, rotation and biases
  mutable quat_t dq_{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  mutable Vec3 dr_{0.0, 0.0, 0.0};      // Relative position
  mutable Vec3 dv_{0.0, 0.0, 0.0};      // Relative velocity
  Vec3 ba_{0.0, 0.0, 0.0};              // Accel biase at i
  Vec3 bg_{0.0, 0.0, 0.0};              // Gyro biase at i

  double Dt_ = 0.0;                // Preintegration time period [s]
  matx_t state_F_ = I(15);         // State jacobian
  matx_t state_P_ = zeros(15, 15); // State covariance
  matx_t sqrt_info_ = I(15, 15);   // Square root information

  /** Form noise matrix Q */
  matx_t formQ();

  /** Form transiton matrix F */
  matx_t formF(const int k,
               const quat_t &dq_i,
               const quat_t &dq_j,
               const Vec3 &ba_i,
               const Vec3 &bg_i,
               const double dt);

  /** Form matrix G */
  matx_t formG(const int k,
               const quat_t &dq_i,
               const quat_t &dq_j,
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

  /** Return state transition matrix F */
  matx_t getMatrixF() const;

  /** Return state transition matrix P */
  matx_t getMatrixP() const;

  /** Return relative rotation dq */
  quat_t getRelativeRotation() const;

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
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const override;
};

} // namespace xyz
