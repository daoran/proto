#include "prototype/vision/camera/radtan.hpp"

namespace prototype {

matx_t distort(const radtan4_t &radtan, const matx_t &points) {
  assert(points.cols() == 3);

  // Setup
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const Eigen::ArrayXd z = points.col(2).array();
  const Eigen::ArrayXd x = points.col(0).array() / z.array();
  const Eigen::ArrayXd y = points.col(1).array() / z.array();

  // Apply radial distortion factor
  // clang-format off
  const int nb_points = points.rows();
  const Eigen::ArrayXd r2 = x.pow(2) + y.pow(2);
  const Eigen::ArrayXd one = ones(nb_points, 1).array();
  const Eigen::ArrayXd x_dash = x * (one + (k1 * r2) + (k2 * r2.pow(2)));
  const Eigen::ArrayXd y_dash = y * (one + (k1 * r2) + (k2 * r2.pow(2)));
  // clang-format on

  // Apply tangential distortion factor
  // clang-format off
  const Eigen::ArrayXd x_ddash = x_dash + (2 * p1 * x * y + p2 * (r2 + 2 * x.pow(2)));
  const Eigen::ArrayXd y_ddash = x_dash + (p1 * (r2 + 2 * y.pow(2)) + 2 * p2 * x * y);
  // clang-format on

  // Project rad-tan distorted points to image plane
  matx_t distorted_points{nb_points, 2};
  distorted_points.col(0) = x_ddash;
  distorted_points.col(1) = y_ddash;

  return distorted_points;
}

vec2_t distort(const radtan4_t &radtan, const vec3_t &point) {
  // Setup
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;
  const double x_dash = point(0) / point(2);
  const double y_dash = point(1) / point(2);

  // Radial distortion factor
  const double r2 = (x_dash * x_dash) + (y_dash * y_dash);
  const double r4 = r2 * r2;
  const double r6 = r2 * r4;
  const double temp = 1 + (k1 * r2) + (k2 * r4);

  // Tangential distortion factor
  // clang-format off
  const double x_ddash = x_dash * temp + (2 * p1 * x_dash * y_dash + p2 * (r2 + 2 * (x_dash * x_dash)));
  const double y_ddash = y_dash * temp + (p1 * (r2 + 2 * (y_dash * y_dash)) + 2 * p2 * x_dash * y_dash);
  // clang-format on

  // Project rad-tan distorted point to image plane
  vec2_t distorted_point{x_ddash, y_ddash};
  return distorted_point;
}

} // namespace prototype
