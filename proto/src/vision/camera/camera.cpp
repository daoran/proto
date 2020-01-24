#include "proto/vision/camera/camera.hpp"

namespace proto {

/****************************************************************************
 *                            RADIAL-TANGENTIAL
 ***************************************************************************/

radtan4_t::radtan4_t() {}

radtan4_t::radtan4_t(const vec4_t &distortion_)
    : k1{distortion_(0)}, k2{distortion_(1)}, p1{distortion_(2)},
      p2{distortion_(3)} {}

radtan4_t::radtan4_t(const double k1_,
                     const double k2_,
                     const double p1_,
                     const double p2_)
    : k1{k1_}, k2{k2_}, p1{p1_}, p2{p2_} {}

radtan4_t::radtan4_t(radtan4_t &radtan4)
    : k1{radtan4.k1}, k2{radtan4.k2}, p1{radtan4.p1}, p2{radtan4.p2} {}

radtan4_t::radtan4_t(const radtan4_t &radtan4)
    : k1{radtan4.k1}, k2{radtan4.k2}, p1{radtan4.p1}, p2{radtan4.p2} {}

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

vec2_t distort(const radtan4_t &radtan, const vec2_t &p, mat2_t &J_point) {
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const double x = p(0);
  const double y = p(1);

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

vec2_t distort(const radtan4_t &radtan,
               const vec2_t &p,
               mat2_t &J_point,
               mat_t<2, 4> &J_params) {
  const vec2_t p_distorted = distort(radtan, p, J_point);

  const double x = p(0);
  const double y = p(1);

  const double xy = x * y;
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  J_params(0, 0) = x * r2;
  J_params(0, 1) = x * r4;
  J_params(0, 2) = 2 * xy;
  J_params(0, 3) = 3 * x2 + y2;

  J_params(1, 0) = y * r2;
  J_params(1, 1) = y * r4;
  J_params(1, 2) = x2 + 3 * y2;
  J_params(1, 3) = 2 * xy;

  return p_distorted;
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

/****************************************************************************
 *                              EQUI-DISTANCE
 ***************************************************************************/

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

/****************************************************************************
 *                                PINHOLE
 ***************************************************************************/

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
  J_P(0, 2) = -x / (z * z);
  J_P(1, 2) = -y / (z * z);

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
  J_P(0, 2) = -x / (z * z);
  J_P(1, 3) = -y / (z * z);

  // Intrinsics Jacobian
  mat2_t J_K = zeros(2, 2);
  J_K(0, 0) = model.fx;
  J_K(1, 1) = model.fy;

  // Measurement Jacobian
  J_h = J_K * J_P;

  return vec2_t{px * model.fx + model.cx, py * model.fy + model.cy};
}

} //  namespace proto
