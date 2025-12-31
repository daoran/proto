#include "BrownConrady4.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// RADIAL-TANGENTIAL DISTORTION                                              //
///////////////////////////////////////////////////////////////////////////////

Vec2 radtan4_distort(const Vec4 &dist_params, const Vec2 &p) {
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

Vec2 radtan4_undistort(const Vec4 &dist_params, const Vec2 &p0) {
  int max_iter = 5;
  Vec2 p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const Vec2 p_distorted = radtan4_distort(dist_params, p);
    const Vec2 err = (p0 - p_distorted);

    // Jacobian
    const Mat2 J = radtan4_point_jacobian(dist_params, p);
    const Vec2 dp = (J.transpose() * J).inverse() * J.transpose() * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

Mat2 radtan4_point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
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

MatX radtan4_params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
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

///////////////////////////////////////////////////////////////////////////////
// BROWN-CONRADY4 CAMERA MODEL                                               //
///////////////////////////////////////////////////////////////////////////////

std::string BrownConrady4::type() const {
  std::string typestr = "BrownConrady4";
  return typestr;
}

int BrownConrady4::project(const Vec2i &res,
                           const VecX &params,
                           const Vec3 &p_C,
                           Vec2 &z_hat) const {
  // Setup
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);

  // Project, distort and then scale and center
  const double fx = proj_params(0);
  const double fy = proj_params(1);
  const double cx = proj_params(2);
  const double cy = proj_params(3);
  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  const Vec2 p_d = radtan4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

MatX BrownConrady4::project_jacobian(const VecX &params,
                                     const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};

  const Mat<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const Mat<2, 2> J_d = radtan4_point_jacobian(dist_params, p);
  const MatX J_p = pinhole_project_jacobian(p_C);
  MatX J_proj = J_k * J_d * J_p;

  return J_proj;
}

MatX BrownConrady4::params_jacobian(const VecX &params, const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);

  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  const Vec2 p_d = radtan4_distort(dist_params, p);
  const MatX J_proj_params = pinhole_params_jacobian(p_d);
  const MatX J_proj_point = pinhole_point_jacobian(proj_params);
  const MatX J_dist_params = radtan4_params_jacobian(dist_params, p);

  MatX J_params;
  J_params.resize(2, 8);
  J_params.block(0, 0, 2, 4) = J_proj_params;
  J_params.block(0, 4, 2, 4) = J_proj_point * J_dist_params;

  return J_params;
}

void BrownConrady4::back_project(const VecX &params,
                                 const Vec2 &x,
                                 Vec3 &ray) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (x.x() - cx) / fx;
  const double py = (x.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 kp = radtan4_undistort(dist_params, p);
  ray.x() = kp.x();
  ray.y() = kp.y();
  ray.z() = 1.0;
}

Vec2 BrownConrady4::undistort(const VecX &params, const Vec2 &z) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (z.x() - cx) / fx;
  const double py = (z.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p_undist = radtan4_undistort(dist_params, p);

  // Project undistorted point to image plane
  const double x = p_undist.x() * fx + cx;
  const double y = p_undist.y() * fy + cy;
  const Vec2 z_undist = {x, y};

  return z_undist;
}

} // namespace cartesian
