#include "Equi4.hpp"

namespace xyz {

std::string Equi4::type() { return "equi4"; }

Vec2 Equi4::distort(const Vec4 &dist_params, const Vec2 &p) {
  const double r = p.norm();
  if (r < 1e-8) {
    return p;
  }

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  // Apply equi distortion
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (thd / r) * p(0);
  const double y_dash = (thd / r) * p(1);

  return Vec2{x_dash, y_dash};
}

Vec2 Equi4::undistort(const Vec4 &dist_params, const Vec2 &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double thd = sqrt(p(0) * p(0) + p(1) * p(1));
  double th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const double th2 = th * th;
    const double th4 = th2 * th2;
    const double th6 = th4 * th2;
    const double th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const double scaling = tan(th) / thd;
  return Vec2{p(0) * scaling, p(1) * scaling};
}

Mat2 Equi4::point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;

  // Form jacobian
  const double th_r = 1.0 / (r * r + 1.0);
  double thd_th = 1.0 + 3.0 * k1 * th2;
  thd_th += 5.0 * k2 * th4;
  thd_th += 7.0 * k3 * th6;
  thd_th += 9.0 * k4 * th8;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;

  Mat2 J_point = I(2);
  J_point(0, 0) = s + x * s_r * r_x;
  J_point(0, 1) = x * s_r * r_y;
  J_point(1, 0) = y * s_r * r_x;
  J_point(1, 1) = s + y * s_r * r_y;

  return J_point;
}

MatX Equi4::params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);

  const double th3 = th * th * th;
  const double th5 = th3 * th * th;
  const double th7 = th5 * th * th;
  const double th9 = th7 * th * th;

  MatX J_dist = zeros(2, 4);
  J_dist(0, 0) = x * th3 / r;
  J_dist(0, 1) = x * th5 / r;
  J_dist(0, 2) = x * th7 / r;
  J_dist(0, 3) = x * th9 / r;

  J_dist(1, 0) = y * th3 / r;
  J_dist(1, 1) = y * th5 / r;
  J_dist(1, 2) = y * th7 / r;
  J_dist(1, 3) = y * th9 / r;

  return J_dist;
}

} // namespace xyz
