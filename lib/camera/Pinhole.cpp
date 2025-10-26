#include "Pinhole.hpp"

namespace xyz {

double pinhole_focal(const int image_size, const double fov) {
  return ((image_size / 2.0) / tan(deg2rad(fov) / 2.0));
}

Mat3 pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy) {
  Mat3 K = zeros(3, 3);
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;
  return K;
}

int pinhole_project(const Vec2i &res,
                    const Vec4 &proj_params,
                    const Vec3 &p_C,
                    Vec2 &z_hat) {
  const double fx = proj_params(0);
  const double fy = proj_params(1);
  const double cx = proj_params(2);
  const double cy = proj_params(3);

  // Project, distort and then scale and center
  const Vec2 x = Vec2{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  z_hat(0) = fx * x(0) + cx;
  z_hat(1) = fy * x(1) + cy;

  // Check projection
  const bool x_ok = (z_hat(0) >= 0 && z_hat(0) <= res.x());
  const bool y_ok = (z_hat(1) >= 0 && z_hat(1) <= res.y());
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

Mat<2, 3> pinhole_project_jacobian(const Vec3 &p_C) {
  const double x = p_C.x();
  const double y = p_C.y();
  const double z = p_C.z();

  Mat<2, 3> J;

  J(0, 0) = 1.0 / z;
  J(0, 1) = 0.0;
  J(0, 2) = -x / (z * z);

  J(1, 0) = 0.0;
  J(1, 1) = 1.0 / z;
  J(1, 2) = -y / (z * z);

  return J;
}

Mat2 pinhole_point_jacobian(const Vec4 &proj_params) {
  Mat2 J = zeros(2, 2);
  J(0, 0) = proj_params(0); // fx
  J(1, 1) = proj_params(1); // fy
  return J;
}

Mat<2, 4> pinhole_params_jacobian(const Vec2 &x) {
  Mat<2, 4> J = zeros(2, 4);
  J(0, 0) = x(0); // x
  J(1, 1) = x(1); // y
  J(0, 2) = 1;
  J(1, 3) = 1;
  return J;
}

///////////////////////////////////////////////////////////////////////////////
// PINHOLE                                                                   //
///////////////////////////////////////////////////////////////////////////////

std::string Pinhole::type() const {
  std::string typestr = "pinhole";
  return typestr;
}

int Pinhole::project(const Vec2i &res,
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
  z_hat.x() = fx * p.x() + cx;
  z_hat.y() = fy * p.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

MatX Pinhole::project_jacobian(const VecX &params, const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};

  const Mat<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const MatX J_p = pinhole_project_jacobian(p_C);
  MatX J_proj = J_k * J_p;

  return J_proj;
}

MatX Pinhole::params_jacobian(const VecX &params, const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(4);
  const VecX dist_params = params.tail(params.size() - 4);

  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
  const MatX J_proj_params = pinhole_params_jacobian(p);
  const MatX J_proj_point = pinhole_point_jacobian(proj_params);

  MatX J_params;
  J_params.resize(2, 4);
  J_params.block(0, 0, 2, 4) = J_proj_params;

  return J_params;
}

void Pinhole::back_project(const VecX &params, const Vec2 &x, Vec3 &ray) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (x.x() - cx) / fx;
  const double py = (x.y() - cy) / fy;

  ray.x() = px;
  ray.y() = py;
  ray.z() = 1.0;
}

Vec2 Pinhole::undistort(const VecX &params, const Vec2 &z) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (z.x() - cx) / fx;
  const double py = (z.y() - cy) / fy;

  // Project point to image plane
  const double x = px * fx + cx;
  const double y = py * fy + cy;
  return Vec2{x, y};
}

} // namespace xyz
