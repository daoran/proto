#include "prototype/vision/camera/radtan.hpp"

namespace prototype {

vec2_t distort(const radtan4_t &radtan, const vec3_t &point) {
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;

  // Project
  const double x = point(0) / point(2);
  const double y = point(1) / point(2);

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

matx_t distort(const radtan4_t &radtan, const matx_t &points) {
  assert(points.rows() == 3);
  assert(points.cols() > 0);

  const int nb_points = points.cols();
  const double k1 = radtan.k1;
  const double k2 = radtan.k2;
  const double p1 = radtan.p1;
  const double p2 = radtan.p2;

  // Project
  const Eigen::ArrayXd x = points.row(0).array() / points.row(2).array();
  const Eigen::ArrayXd y = points.row(1).array() / points.row(2).array();

  // Apply radial distortion factor
  const Eigen::ArrayXd x2 = x * x;
  const Eigen::ArrayXd y2 = y * y;
  const Eigen::ArrayXd r2 = x2 + y2;
  const Eigen::ArrayXd r4 = r2 * r2;
  const Eigen::ArrayXd x_dash = x * (1 + (k1 * r2) + (k2 * r4));
  const Eigen::ArrayXd y_dash = y * (1 + (k1 * r2) + (k2 * r4));

  // Apply tangential distortion factor
  const Eigen::ArrayXd xy = x * y;
  const Eigen::ArrayXd x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const Eigen::ArrayXd y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  // Form results
  matx_t distorted_points{2, nb_points};
  distorted_points.row(0) = x_ddash;
  distorted_points.row(1) = y_ddash;

  return distorted_points;
}

} // namespace prototype
