#pragma once

#include "../Core.hpp"

namespace xyz {

/** Camera Model */
struct CameraModel {
  CameraModel() = default;
  virtual ~CameraModel() = default;

  virtual std::string type() const = 0;
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const = 0;
  virtual MatX project_jacobian(const VecX &params, const Vec3 &p_C) const = 0;
  virtual MatX params_jacobian(const VecX &params, const Vec3 &p_C) const = 0;
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const = 0;
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const = 0;
};

} // namespace xyz
