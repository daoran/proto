#pragma once
#include "CameraModel.hpp"

namespace xyz {

/**
 * Unified Camera Model
 *
 * Source:
 *
 * C. Mei and P. Rives. Single view point omnidirectional
 * camera calibration from planar grids. In Proceedings 2007
 * IEEE International Conference on Robotics and Automation,
 * pages 3945–3950, April 2007
 *
 */
struct UnifiedCameraModel : CameraModel {
  UnifiedCameraModel();
  ~UnifiedCameraModel();

  MatX ucm_point_jacobian();
  MatX ucm_params_jacobian();

  virtual std::string type() const override;
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace xyz
