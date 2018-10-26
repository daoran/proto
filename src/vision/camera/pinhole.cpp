#include "prototype/vision/camera/pinhole.hpp"

namespace prototype {

mat3_t pinhole_K(const vec4_t &intrinsics) {
  mat3_t K;

  // clang-format off
  K << intrinsics(0), 0.0, intrinsics(2),
       0.0, intrinsics(1), intrinsics(3),
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

mat34_t pinhole_projection_matrix(const mat3_t &K, const mat3_t &R, const vec3_t &t) {
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

vec2_t pinhole_pixel2ideal(const double fx,
                         const double fy,
                         const double cx,
                         const double cy,
                         const vec2_t &pixel) {
  vec2_t pt((pixel(0) - cx) / fx, (pixel(1) - cy) / fy);
  return pt;
}

vec2_t pinhole_pixel2ideal(const mat3_t &K, const vec2_t &pixel) {
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  return pinhole_pixel2ideal(fx, fy, cx, cy, pixel);
}

int PinholeModel::configure(const std::string &config_file) {
  // Load config file
  config_parser_t parser;
  config_parser_add(parser, "image_width", &this->image_width);
  config_parser_add(parser, "image_height", &this->image_height);
  config_parser_add(parser, "fx", &this->fx);
  config_parser_add(parser, "fy", &this->fy);
  config_parser_add(parser, "cx", &this->cx);
  config_parser_add(parser, "cy", &this->cy);
  if (config_parser_load(parser, config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Form the intrinsics matrix
  this->K = mat3_t::Zero();
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;

  return 0;
}

mat34_t PinholeModel::P(const mat3_t &R, const vec3_t &t) {
  return pinhole_projection_matrix(this->K, R, t);
}

vec2_t PinholeModel::project(const vec3_t &X, const mat3_t &R, const vec3_t &t) {
  return pinhole_project(this->K, R, t, X);
}

vec3_t PinholeModel::project(const vec4_t &X, const mat3_t &R, const vec3_t &t) {
  return pinhole_project(this->K, R, t, X);
}

vec2_t PinholeModel::pixel2ideal(const vec2_t &pixel) {
  return pinhole_pixel2ideal(this->K, pixel);
}

vec2_t PinholeModel::pixel2ideal(const cv::Point2f &pixel) {
  return this->pixel2ideal(vec2_t{pixel.x, pixel.y});
}

vec2_t PinholeModel::pixel2ideal(const cv::KeyPoint &kp) {
  return this->pixel2ideal(vec2_t{kp.pt.x, kp.pt.y});
}

} //  namespace prototype
