#pragma once
#include "ResidualBlock.hpp"
#include "camera/CameraGeometry.hpp"

namespace xyz {

/** Reprojection Error */
class CalibCameraError : public ResidualBlock {
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
  CalibCameraError(const std::shared_ptr<CameraGeometry> &camera_geometry,
                    const std::vector<double *> &param_ptrs,
                    const std::vector<ParamBlock::Type> &param_types,
                    const Vec2 &z,
                    const Mat2 &covar);

  /** Create residual block */
  static std::shared_ptr<CalibCameraError>
  create(const std::shared_ptr<CameraGeometry> &camera,
         double *T_C0F,
         double *p_FFi,
         const Vec2 &z,
         const Mat2 &covar);

  /** Get residuals */
  bool getResiduals(Vec2 &r) const;

  /** Get reprojection error */
  bool getReprojError(double *error) const;

  /** Evaluate with minimial Jacobians */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const override;
};

} // namespace xyz
