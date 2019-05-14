#include "prototype/vision/camera/radtan.hpp"

namespace proto {

radtan4_t::radtan4_t() {}

radtan4_t::radtan4_t(const vec4_t &distortion_)
    : k1{distortion_(0)}, k2{distortion_(1)},
      p1{distortion_(2)}, p2{distortion_(3)} {}

radtan4_t::radtan4_t(const double k1_,
                     const double k2_,
                     const double p1_,
                     const double p2_)
    : k1{k1_}, k2{k2_}, p1{p1_}, p2{p2_} {}

radtan4_t::radtan4_t(radtan4_t &radtan4)
    : k1{radtan4.k1}, k2{radtan4.k2},
      p1{radtan4.p1}, p2{radtan4.p2} {}

radtan4_t::radtan4_t(const radtan4_t &radtan4)
    : k1{radtan4.k1}, k2{radtan4.k2},
      p1{radtan4.p1}, p2{radtan4.p2} {}

radtan4_t::~radtan4_t() {}

void radtan4_t::operator=(const radtan4_t &src) throw() {
  k1 = src.k1;
  k2 = src.k2;
  p1 = src.p1;
  p2 = src.p2;
}

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4) {
  os << "k1: " << radtan4.k1 << std::endl;
  os << "k2: " << radtan4.k2 << std::endl;
  os << "p1: " << radtan4.p1 << std::endl;
  os << "p2: " << radtan4.p2 << std::endl;
  return os;
}

vec4_t distortion_coeffs(const radtan4_t &radtan) {
  return vec4_t{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
}

vec2_t distort(const radtan4_t &radtan, const vec2_t &point) {
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const double x = point(0);
  const double y = point(1);

  // Apply radial distortion
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  // Apply tangential distortion
  const double xy = x * y;
  const double x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const double y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  return vec2_t{x_ddash, y_ddash};
}

vec2_t distort(const radtan4_t &radtan, const vec2_t &point, mat2_t &J_point) {
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const double x = point(0);
  const double y = point(1);

  // Apply radial distortion
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  // Apply tangential distortion
  const double xy = x * y;
  const double x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const double y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  // clang-format off
  J_point(0, 0) = 1 + k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(1, 0) = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1 + k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2);
  // clang-format on
  // Above is generated using sympy

  return vec2_t{x_ddash, y_ddash};
}

matx_t distort(const radtan4_t &radtan, const matx_t &points) {
  assert(points.rows() == 2);
  assert(points.cols() > 0);

  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const Eigen::ArrayXd x = points.row(0).array();
  const Eigen::ArrayXd y = points.row(1).array();

  // Apply radial distortion
  const Eigen::ArrayXd x2 = x * x;
  const Eigen::ArrayXd y2 = y * y;
  const Eigen::ArrayXd r2 = x2 + y2;
  const Eigen::ArrayXd r4 = r2 * r2;
  const Eigen::ArrayXd x_dash = x * (1 + (k1 * r2) + (k2 * r4));
  const Eigen::ArrayXd y_dash = y * (1 + (k1 * r2) + (k2 * r4));

  // Apply tangential distortion
  const Eigen::ArrayXd xy = x * y;
  const Eigen::ArrayXd x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const Eigen::ArrayXd y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  // Form results
  const int nb_points = points.cols();
  matx_t distorted_points{2, nb_points};
  distorted_points.row(0) = x_ddash;
  distorted_points.row(1) = y_ddash;

  return distorted_points;
}

vec2_t undistort(const radtan4_t &radtan,
                 const vec2_t &p0,
                 const int max_iter) {
  vec2_t p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const vec2_t p_distorted = distort(radtan, p);
    const vec2_t err = (p0 - p_distorted);

    // Jacobian
    mat2_t J;
    distort(radtan, p, J);
    const mat2_t pinv = (J.transpose() * J).inverse() * J.transpose();
    const vec2_t dp = pinv * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

} // namespace proto
