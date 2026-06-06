#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "calib/CalibTargetGeometry.hpp"

namespace cartesian {

// Forward declaration
class CalibCameraImuError2;
using CalibCameraImuError2Ptr = std::shared_ptr<CalibCameraImuError2>;

/** Camera-IMU Calibration Reprojection Error */
class CalibCameraImuError2 : public ResidualBlock {
private:
  const timestamp_t ts_km1_;
  const timestamp_t ts_k_;
  std::shared_ptr<CameraGeometry> camera_geometry_;
  Vec2 z_km1_;
  Vec2 z_k_;
  Vec2 v_k_;
  Mat2 covar_;
  Mat2 info_;
  Mat2 sqrt_info_;
  mutable bool valid_ = true;
  mutable Vec2 residuals_;

public:
  /** Constructor */
  CalibCameraImuError2(const timestamp_t ts_km1,
                       const timestamp_t ts_k,
                       const std::shared_ptr<CameraGeometry> &camera_geometry,
                       const std::vector<double *> &param_ptrs,
                       const std::vector<ParamBlock::Type> &param_types,
                       const Vec2 &z_km1,
                       const Vec2 &z_k,
                       const Mat2 &covar);

  /** Create residual block */
  static std::shared_ptr<CalibCameraImuError2>
  create(const timestamp_t ts_km1,
         const timestamp_t ts_k,
         const std::shared_ptr<CameraGeometry> &camera,
         const std::shared_ptr<ImuGeometry> &imu,
         const std::shared_ptr<CalibTargetGeometry> &target,
         double *sensor_pose_k,
         double *target_pose,
         double *time_delay,
         const int point_id,
         const Vec2 &z_km1,
         const Vec2 &z_k,
         const Mat2 &covar = eye(2));

  /** Is point reprojection valid? */
  bool valid() const;

  /** Get covariance matrix */
  Mat2 get_covariance_matrix() const;

  /** Get residuals */
  bool get_residuals(Vec2 &r) const;

  /** Get reprojection error */
  bool get_reproj_error(double *error) const;

  /** Evaluate with minimial Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace cartesian
