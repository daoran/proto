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

pinhole_t::~pinhole_t() {}

mat3_t pinhole_K(const double fx, const double fy,
                 const double cx, const double cy) {
  mat3_t K;
  // clang-format off
  K << fx, 0.0, cx,
       0.0, fx, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

mat3_t pinhole_K(const vec4_t &intrinsics) {
  const double fx = intrinsics(0);
  const double fy = intrinsics(1);
  const double cx = intrinsics(2);
  const double cy = intrinsics(3);
  return pinhole_K(fx, fy, cx, cy);
}

mat34_t pinhole_P(const mat3_t &K,
                  const mat3_t &R,
                  const vec3_t &t) {
  mat34_t A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;
  const mat34_t P = K * A;
  return P;
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

vec2_t project(const pinhole_t &model, const vec3_t &p) {
  const vec3_t x = model.K * p;
  return vec2_t{x(0) / x(2), x(1) / x(2)};
}

vec2_t project(const pinhole_t &pinhole,
               const mat3_t &R,
               const vec3_t &t,
               const vec4_t &hp) {
  const mat34_t P = pinhole_P(pinhole.K, R, t);
  const vec3_t x = P * hp;
  return vec2_t{x(0) / x(2), x(1) / x(2)};
}

vec2_t project(const pinhole_t &pinhole,
               const mat3_t &R,
               const vec3_t &t,
               const vec3_t &p) {
  const vec4_t hp = p.homogeneous();
  return project(pinhole, R, t, hp);
}

vec2_t pixel2ideal(const pinhole_t &pinhole, const vec2_t &pixel) {
  const double fx = pinhole.fx;
  const double fy = pinhole.fy;
  const double cx = pinhole.cx;
  const double cy = pinhole.cy;
  vec2_t pt((pixel(0) - cx) / fx, (pixel(1) - cy) / fy);
  return pt;
}

} //  namespace prototype
