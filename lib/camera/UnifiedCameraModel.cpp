#include "UnifiedCameraModel.hpp"
#include "BrownConrady4.hpp"

namespace xyz {

std::string UnifiedCameraModel::type() const {
  std::string typestr = "UnifiedCameraModel";
  return typestr;
}

int UnifiedCameraModel::project(const Vec2i &res,
                                const VecX &params,
                                const Vec3 &p_C,
                                Vec2 &z_hat) const {
  // Setup
  const Vec4 proj_params = params.head(5);
  const VecX dist_params = params.tail(params.size() - 4);

  // Project, distort and then scale and center
  const double xi = proj_params(0);
  const double fx = proj_params(1);
  const double fy = proj_params(2);
  const double cx = proj_params(3);
  const double cy = proj_params(4);
  const double fov = (xi <= 1.0) ? xi : 1 / xi;
  const double d = p_C.norm();

  if (p_C.z() <= -(fov * d)) {
    return false;
  }
  const double rz = 1.0 / (p_C.z() + xi * d);
  const Vec2 p{p_C.x() * rz, p_C.y() * rz};
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

MatX UnifiedCameraModel::project_jacobian(const VecX &params,
                                          const Vec3 &p_C) const {
  const Vec4 proj_params = params.head(5);
  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};

  const Mat<2, 2> J_k = pinhole_point_jacobian(proj_params);
  const Mat<2, 2> J_d = radtan4_point_jacobian(dist_params, p);
  const MatX J_p = pinhole_project_jacobian(p_C);
  MatX J_proj = J_k * J_d * J_p;

  return J_proj;
}

MatX UnifiedCameraModel::params_jacobian(const VecX &params,
                                         const Vec3 &p_C) const {
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

void UnifiedCameraModel::back_project(const VecX &params,
                                      const Vec2 &x,
                                      Vec3 &ray) const {
  // Back-project and undistort
  const double xi = params(0);
  const double fx = params(1);
  const double fy = params(2);
  const double cx = params(3);
  const double cy = params(4);
  const double px = (x.x() - cx) / fx;
  const double py = (x.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(4);
  const Vec2 kp = radtan4_undistort(dist_params, p);
  ray.x() = kp.x();
  ray.y() = kp.y();
  ray.z() = 1.0;
}

Vec2 UnifiedCameraModel::undistort(const VecX &params, const Vec2 &z) const {
  // Back-project and undistort
  const double xi = params(0);
  const double fx = params(1);
  const double fy = params(2);
  const double cx = params(3);
  const double cy = params(4);
  const double px = (z.x() - cx) / fx;
  const double py = (z.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(4);
  const Vec2 p_undist = radtan4_undistort(dist_params, p);

  // Project undistorted point to image plane
  const double x = p_undist.x() * fx + cx;
  const double y = p_undist.y() * fy + cy;
  const Vec2 z_undist = {x, y};

  return z_undist;
}

} // namespace xyz
