#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "calib/CalibTargetGeometry.hpp"

namespace xyz {

// Forward declaration
class CalibCameraImuError;
using CalibCameraImuErrorPtr = std::shared_ptr<CalibCameraImuError>;

/** Camera-IMU Calibration Reprojection Error */
class CalibCameraImuError : public ResidualBlock {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;
  Vec2 z_;
  Mat2 covar_;
  Mat2 info_;
  Mat2 sqrt_info_;
  mutable bool valid_ = true;
  mutable Vec2 residuals_;

public:
  /** Constructor */
  CalibCameraImuError(const std::shared_ptr<CameraGeometry> &camera_geometry,
                      const std::vector<double *> &param_ptrs,
                      const std::vector<ParamBlock::Type> &param_types,
                      const Vec2 &z,
                      const Mat2 &covar);

  /** Create residual block */
  static std::shared_ptr<CalibCameraImuError>
  create(const std::shared_ptr<CameraGeometry> &camera,
         const std::shared_ptr<ImuGeometry> &imu,
         const std::shared_ptr<CalibTargetGeometry> &target,
         double *T_WS,
         double *T_WT0,
         const int point_id,
         const Vec2 &z,
         const Mat2 &covar = I(2));

  /** Is point reprojection valid? */
  bool valid() const;

  /** Get residuals */
  bool getResiduals(Vec2 &r) const;

  /** Get reprojection error */
  bool getReprojError(double *error) const;

  /** Evaluate with minimial Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace xyz
