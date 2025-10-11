#include "RadTan4.hpp"

namespace xyz {

std::string RadTan4::type() { return "radtan4"; }

Vec2 RadTan4::distort(const Vec4 &dist_params, const Vec2 &p) {
  const double x = p.x();
  const double y = p.y();

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  // Apply radial distortion
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  // Apply tangential distortion
  const double xy = x * y;
  const double x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const double y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  return Vec2{x_ddash, y_ddash};
}

Vec2 RadTan4::undistort(const Vec4 &dist_params, const Vec2 &p0) {
  int max_iter = 5;
  Vec2 p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const Vec2 p_distorted = RadTan4::distort(dist_params, p);
    const Vec2 err = (p0 - p_distorted);

    // Jacobian
    const Mat2 J = RadTan4::point_jacobian(dist_params, p);
    const Vec2 dp = (J.transpose() * J).inverse() * J.transpose() * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

Mat2 RadTan4::point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  const double x = p(0);
  const double y = p(1);

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  Mat2 J_point;
  J_point(0, 0) = 1.0 + k1 * r2 + k2 * r4;
  J_point(0, 0) += 2.0 * p1 * y + 6.0 * p2 * x;
  J_point(0, 0) += x * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(1, 0) = 2.0 * p1 * x + 2.0 * p2 * y;
  J_point(1, 0) += y * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1.0 + k1 * r2 + k2 * r4;
  J_point(1, 1) += 6.0 * p1 * y + 2.0 * p2 * x;
  J_point(1, 1) += y * (2.0 * k1 * y + 4.0 * k2 * y * r2);
  // Above is generated using sympy

  return J_point;
}

MatX RadTan4::params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);

  const double x = p.x();
  const double y = p.y();

  const double xy = x * y;
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  Mat<2, 4> J_dist = zeros(2, 4);
  J_dist(0, 0) = x * r2;
  J_dist(0, 1) = x * r4;
  J_dist(0, 2) = 2.0 * xy;
  J_dist(0, 3) = r2 + 2.0 * x2;

  J_dist(1, 0) = y * r2;
  J_dist(1, 1) = y * r4;
  J_dist(1, 2) = r2 + 2.0 * y2;
  J_dist(1, 3) = 2 * xy;

  return J_dist;
}

} // namespace xyz
