#include "prototype/vision/camera/pinhole.hpp"

namespace prototype {

pinhole_t::pinhole_t() {}

pinhole_t::pinhole_t(const vec4_t &intrinsics_)
    : fx{intrinsics_(0)}, fy{intrinsics_(1)}, cx{intrinsics_(2)},
      cy{intrinsics_(3)} {
  // clang-format off
  K << fx, 0.0, cx,
       0.0, fx, cy,
       0.0, 0.0, 1.0;
  // clang-format on
}

pinhole_t::pinhole_t(const double fx_,
                     const double fy_,
                     const double cx_,
                     const double cy_)
    : fx{fx_}, fy{fy_}, cx{cx_}, cy{cy_} {
  // clang-format off
  K << fx_, 0.0, cx_,
       0.0, fx_, cy_,
       0.0, 0.0, 1.0;
  // clang-format on
}

vec2_t project(const pinhole_t &model, const vec3_t &X) {
  const vec3_t x = model.K * X;
  return vec2_t{x(0) / x(2), x(1) / x(2)};
}

mat34_t projection_matrix(const pinhole_t &model,
                          const mat3_t &R,
                          const vec3_t &t) {
  mat34_t A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;
  const mat34_t P = model.K * A;
  return P;
}

mat3_t pinhole_K(const vec4_t &intrinsics) {
  mat3_t K;

  // clang-format off
  const double fx = intrinsics(0);
  const double fy = intrinsics(1);
  const double cx = intrinsics(2);
  const double cy = intrinsics(3);
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

mat3_t
pinhole_K(const double fx, const double fy, const double cx, const double cy) {
  mat3_t K;

  // clang-format off
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

double pinhole_focal_length(const int image_width, const double fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

vec2_t pinhole_focal_length(const vec2_t &image_size,
                            const double hfov,
                            const double vfov) {
  const double fx = ((image_size(0) / 2.0) / tan(deg2rad(hfov) / 2.0));
  const double fy = ((image_size(1) / 2.0) / tan(deg2rad(vfov) / 2.0));
  return vec2_t{fx, fy};
}

mat34_t pinhole_projection_matrix(const mat3_t &K,
                                  const mat3_t &R,
                                  const vec3_t &t) {
  mat34_t A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;
  const mat34_t P = K * A;
  return P;
}

vec2_t pinhole_project(const mat3_t &K, const vec3_t &X) {
  const vec3_t x = K * X;
  return vec2_t{x(0) / x(2), x(1) / x(2)};
}

vec3_t pinhole_project(const mat3_t &K,
                       const mat3_t &R,
                       const vec3_t &t,
                       const vec4_t &X) {
  mat34_t A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;

  // Form projection matrix
  const mat34_t P = K * A;
  const vec3_t x = P * X;
  return x;
}

vec2_t pinhole_project(const mat3_t &K,
                       const mat3_t &R,
                       const vec3_t &t,
                       const vec3_t &X) {
  const vec4_t X_homo = X.homogeneous();
  const vec3_t x = pinhole_project(K, R, t, X_homo);
  return vec2_t{x(0) / x(2), x(1) / x(2)};
}

vec2_t pinhole_pixel2point(const double fx,
                           const double fy,
                           const double cx,
                           const double cy,
                           const vec2_t &pixel) {
  vec2_t pt((pixel(0) - cx) / fx, (pixel(1) - cy) / fy);
  return pt;
}

vec2_t pinhole_pixel2point(const mat3_t &K, const vec2_t &pixel) {
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  return pinhole_pixel2point(fx, fy, cx, cy, pixel);
}

} //  namespace prototype
