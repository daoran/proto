#pragma once
#include "ResidualBlock.hpp"
#include "../camera/CameraGeometry.hpp"
#include "../calib/CalibTargetGeometry.hpp"

namespace xyz {

// Forward declaration
class CalibCameraError;
using CalibCameraErrorPtr = std::shared_ptr<CalibCameraError>;

/** Camera Calibration Reprojection Error */
class CalibCameraError : public ResidualBlock {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;
  std::shared_ptr<CalibTargetGeometry> target_geometry_;
  Vec2 z_;
  Mat2 covar_;
  Mat2 info_;
  Mat2 sqrt_info_;
  mutable bool valid_ = true;
  mutable Vec2 residuals_;

public:
  /** Constructor */
  CalibCameraError(const std::shared_ptr<CameraGeometry> &camera_geometry,
                   const std::vector<double *> &param_ptrs,
                   const std::vector<ParamBlock::Type> &param_types,
                   const Vec2 &z,
                   const Mat2 &covar = I(2));

  /** Create residual block */
  static std::shared_ptr<CalibCameraError>
  create(const std::shared_ptr<CameraGeometry> &camera,
         const std::shared_ptr<CalibTargetGeometry> &target,
         const int point_id,
         double *T_C0T0,
         const Vec2 &z,
         const Mat2 &covar = I(2));

  /** Is point reprojection valid? */
  bool valid() const;

  /** Get covariance matrix */
  Mat2 getCovarianceMatrix() const;

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
