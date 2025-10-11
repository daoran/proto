#pragma once
#include "../../Core.hpp"

namespace xyz {

struct Distortion {
  static std::string type();
  static Vec2 distort(const Vec4 &dist_params, const Vec2 &p);
  static Vec2 undistort(const Vec4 &dist_params, const Vec2 &p0);
  static Mat2 point_jacobian(const Vec4 &dist_params, const Vec2 &p);
  static MatX params_jacobian(const Vec4 &dist_params, const Vec2 &p);
};

} // namespace xyz
