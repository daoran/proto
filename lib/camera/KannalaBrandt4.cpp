#include "KannalaBrandt4.hpp"

namespace xyz {

///////////////////////////////////////////////////////////////////////////////
// EQUIDISTANT DISTORTION                                                    //
///////////////////////////////////////////////////////////////////////////////

Vec2 equi4_distort(const Vec4 &dist_params, const Vec2 &p) {
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

Vec2 equi4_undistort(const Vec4 &dist_params, const Vec2 &p) {
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

Mat2 equi4_point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
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

MatX equi4_params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
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

///////////////////////////////////////////////////////////////////////////////
// KANNALA-BRANDT4 CAMERA MODEL                                              //
///////////////////////////////////////////////////////////////////////////////

std::string KannalaBrandt4::type() const {
  std::string typestr = "KannalaBrandt4";
  return typestr;
}

int KannalaBrandt4::project(const Vec2i &res,
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

  // const double k1 = dist_params(0);
  // const double k2 = dist_params(1);
  // const double k3 = dist_params(2);
  // const double k4 = dist_params(3);

  // const double px = p_C.x();
  // const double py = p_C.y();
  // const double pz = p_C.z();
  // const double r = sqrt(px * px + py * py);
  // const double th = atan2(r, pz);
  // const double th2 = th * th;
  // const double th4 = th2 * th2;
  // const double th6 = th4 * th2;
  // const double th8 = th4 * th4;
  // const double thd = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  // z_hat.x() = fx * thd * px / r + cx;
  // z_hat.y() = fy * thd * py / r + cy;

  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  const Vec2 p_d = equi4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

MatX KannalaBrandt4::project_jacobian(const VecX &params,
                                      const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};

  const Mat<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const Mat<2, 2> J_d = equi4_point_jacobian(dist_params, p);
  const MatX J_p = pinhole_project_jacobian(p_C);
  MatX J_proj = J_k * J_d * J_p;

  return J_proj;
}

MatX KannalaBrandt4::params_jacobian(const VecX &params,
                                     const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);

  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  const Vec2 p_d = equi4_distort(dist_params, p);
  const MatX J_proj_params = pinhole_params_jacobian(p_d);
  const MatX J_proj_point = pinhole_point_jacobian(proj_params);
  const MatX J_dist_params = equi4_params_jacobian(dist_params, p);

  MatX J_params;
  J_params.resize(2, 8);
  J_params.block(0, 0, 2, 4) = J_proj_params;
  J_params.block(0, 4, 2, 4) = J_proj_point * J_dist_params;

  return J_params;
}

void KannalaBrandt4::back_project(const VecX &params,
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
  const Vec2 kp = equi4_undistort(dist_params, p);
  ray.x() = kp.x();
  ray.y() = kp.y();
  ray.z() = 1.0;
}

Vec2 KannalaBrandt4::undistort(const VecX &params, const Vec2 &z) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (z.x() - cx) / fx;
  const double py = (z.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p_undist = equi4_undistort(dist_params, p);

  // Project undistorted point to image plane
  const double x = p_undist.x() * fx + cx;
  const double y = p_undist.y() * fy + cy;
  const Vec2 z_undist = {x, y};

  return z_undist;
}
} // namespace xyz
