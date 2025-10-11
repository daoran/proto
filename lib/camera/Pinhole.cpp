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

} // namespace xyz
