#include "proto/vision/camera/pinhole.hpp"

namespace proto {

pinhole_t::pinhole_t() {}

pinhole_t::pinhole_t(const vec4_t &intrinsics_)
    : fx{intrinsics_(0)}, fy{intrinsics_(1)}, cx{intrinsics_(2)},
      cy{intrinsics_(3)} {}

pinhole_t::pinhole_t(const mat3_t &K_)
    : fx{K_(0, 0)}, fy{K_(1, 1)}, cx{K_(0, 2)}, cy{K_(1, 2)} {}

pinhole_t::pinhole_t(const double fx_,
                     const double fy_,
                     const double cx_,
                     const double cy_)
    : fx{fx_}, fy{fy_}, cx{cx_}, cy{cy_} {}

pinhole_t::pinhole_t(pinhole_t &pinhole)
    : fx{pinhole.fx}, fy{pinhole.fy}, cx{pinhole.cx}, cy{pinhole.cy} {}

pinhole_t::pinhole_t(const pinhole_t &pinhole)
    : fx{pinhole.fx}, fy{pinhole.fy}, cx{pinhole.cx}, cy{pinhole.cy} {}

pinhole_t::~pinhole_t() {}

void pinhole_t::operator=(const pinhole_t &src) throw() {
  fx = src.fx;
  fy = src.fy;
  cx = src.cx;
  cy = src.cy;
}

std::ostream &operator<<(std::ostream &os, const pinhole_t &pinhole) {
  os << "fx: " << pinhole.fx << std::endl;
  os << "fy: " << pinhole.fy << std::endl;
  os << "cx: " << pinhole.cx << std::endl;
  os << "cy: " << pinhole.cy << std::endl;
  return os;
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

mat3_t pinhole_K(const pinhole_t &pinhole) { return pinhole_K(*pinhole.data); }

mat3_t pinhole_K(const vec2_t &image_size,
                 const double lens_hfov,
                 const double lens_vfov) {
  const double fx = pinhole_focal_length(image_size(0), lens_hfov);
  const double fy = pinhole_focal_length(image_size(1), lens_vfov);
  const double cx = image_size(0) / 2.0;
  const double cy = image_size(1) / 2.0;
  return pinhole_K(fx, fy, cx, cy);
}

mat34_t pinhole_P(const mat3_t &K, const mat3_t &C_WC, const vec3_t &r_WC) {
  mat34_t A;
  A.block(0, 0, 3, 3) = C_WC;
  A.block(0, 3, 3, 1) = -C_WC * r_WC;
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

vec2_t project(const vec3_t &p) { return vec2_t{p(0) / p(2), p(1) / p(2)}; }

vec2_t project(const vec3_t &p, mat_t<2, 3> &J_P) {
  const double x = p(0);
  const double y = p(1);
  const double z = p(2);

  // Projection Jacobian
  J_P = zeros(2, 3);
  J_P(0, 0) = 1.0 / z;
  J_P(1, 1) = 1.0 / z;
  J_P(0, 2) = -x / (z*z);
  J_P(1, 2) = -y / (z*z);

  return vec2_t{x / z, x / z};
}

vec2_t project(const pinhole_t &model, const vec2_t &p) {
  return vec2_t{p(0) * model.fx + model.cx, p(1) * model.fy + model.cy};
}

vec2_t project(const pinhole_t &model, const vec3_t &p) {
  const double px = p(0) / p(2);
  const double py = p(1) / p(2);
  return vec2_t{px * model.fx + model.cx, py * model.fy + model.cy};
}

vec2_t project(const pinhole_t &model, const vec3_t &p, mat_t<2, 3> &J_h) {
  const double x = p(0);
  const double y = p(1);
  const double z = p(2);

  const double px = x / z;
  const double py = y / z;

  // Projection Jacobian
  mat_t<2, 3> J_P = zeros(2, 3);
  J_P(0, 0) = 1.0 / z;
  J_P(1, 1) = 1.0 / z;
  J_P(0, 2) = -x / (z*z);
  J_P(1, 3) = -y / (z*z);

  // Intrinsics Jacobian
  mat2_t J_K = zeros(2, 2);
  J_K(0, 0) = model.fx;
  J_K(1, 1) = model.fy;

  // Measurement Jacobian
  J_h = J_K * J_P;

  return vec2_t{px * model.fx + model.cx, py * model.fy + model.cy};
}


} //  namespace proto
