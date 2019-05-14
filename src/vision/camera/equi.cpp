#include "prototype/vision/camera/equi.hpp"

namespace proto {

equi4_t::equi4_t(const double k1_,
                 const double k2_,
                 const double k3_,
                 const double k4_)
    : k1{k1_}, k2{k2_}, k3{k3_}, k4{k4_} {}

equi4_t::~equi4_t() {}

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4) {
  os << "k1: " << equi4.k1 << std::endl;
  os << "k2: " << equi4.k2 << std::endl;
  os << "k3: " << equi4.k3 << std::endl;
  os << "k4: " << equi4.k4 << std::endl;
  return os;
}

vec2_t distort(const equi4_t &equi, const vec2_t &point) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
  const double x = point(0);
  const double y = point(1);
  const double r = sqrt(pow(x, 2) + pow(y, 2));

  if (r < 1e-8) {
    return point;
  }

  // Apply equi distortion
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double th_d = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (th_d / r) * x;
  const double y_dash = (th_d / r) * y;

  // Project equi distorted point to image plane
  return vec2_t{x_dash, y_dash};
}

vec2_t distort(const equi4_t &equi, const vec2_t &point, mat2_t &J_point) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
  const double x = point(0);
  const double y = point(1);
  const double r = sqrt(pow(x, 2) + pow(y, 2));

  if (r < 1e-8) {
    J_point = I(2);
    return point;
  }

  // Apply equi distortion
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;
  const double x_dash = s * x;
  const double y_dash = s * y;

  // Form jacobian
  // clang-format off
  const double th_r = 1.0 / (r * r + 1.0);
  const double thd_th = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;
  J_point(0,0) = s + x * s_r * r_x;
  J_point(0,1) = x * s_r * r_y;
  J_point(1,0) = y * s_r * r_x;
  J_point(1,1) = s + y * s_r * r_y;
  // clang-format on

  // Project equi distorted point to image plane
  return vec2_t{x_dash, y_dash};
}

matx_t distort(const equi4_t &equi, const matx_t &points) {
  assert(points.rows() == 2);
  assert(points.cols() > 0);

  // Setup
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
  const Eigen::ArrayXd x = points.row(0).array();
  const Eigen::ArrayXd y = points.row(1).array();
  const Eigen::ArrayXd r = (x.pow(2) + y.pow(2)).sqrt();

  // Apply equi distortion
  const auto th = r.atan();
  const auto th2 = th.pow(2);
  const auto th4 = th.pow(4);
  const auto th6 = th.pow(6);
  const auto th8 = th.pow(8);
  const auto thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const auto s = thd / r;
  const auto x_dash = s * x;
  const auto y_dash = s * y;

  // Project equi distorted points to image plane
  const int nb_points = points.cols();
  matx_t distorted_points{2, nb_points};
  distorted_points.row(0) = x_dash;
  distorted_points.row(1) = y_dash;
  return distorted_points;
}

vec2_t undistort(const equi4_t &equi, const vec2_t &p) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
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
  vec2_t p_ud{p(0) * scaling, p(1) * scaling};
  return p_ud;
}

} //  namespace proto
