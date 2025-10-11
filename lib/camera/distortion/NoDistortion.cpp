#include "NoDistortion.hpp"

namespace xyz {

std::string NoDistortion::type() { return "nodist"; }

Vec2 NoDistortion::distort(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);
  return p;
}

Vec2 NoDistortion::undistort(const Vec4 &dist_params, const Vec2 &p0) {
  UNUSED(dist_params);
  return p0;
}

Mat2 NoDistortion::point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);
  UNUSED(p);
  return I(2);
}

MatX NoDistortion::params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);
  UNUSED(p);
  return I(2);
}

} // namespace xyz
