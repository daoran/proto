#include "prototype/vision/camera/distortion.hpp"

namespace prototype {

matx_t radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const matx_t &points) {
  // Asserts
  assert(points.cols() == 3);

  // Setup
  const Eigen::ArrayXd z = points.col(2).array();
  const Eigen::ArrayXd x = points.col(0).array() / z.array();
  const Eigen::ArrayXd y = points.col(1).array() / z.array();

  // Apply radial distortion factor
  // clang-format off
  const int nb_points = points.rows();
  const Eigen::ArrayXd r2 = x.pow(2) + y.pow(2);
  const Eigen::ArrayXd one = ones(nb_points, 1).array();
  const Eigen::ArrayXd x_dash = x * (one + (k1 * r2) + (k2 * r2.pow(2)) + (k3 * r2.pow(3)));
  const Eigen::ArrayXd y_dash = y * (one + (k1 * r2) + (k2 * r2.pow(2)) + (k3 * r2.pow(3)));
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

vec2_t radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const vec3_t &point) {
  // Setup
  const double x_dash = point(0) / point(2);
  const double y_dash = point(1) / point(2);

  // Radial distortion factor
  const double r2 = (x_dash * x_dash) + (y_dash * y_dash);
  const double r4 = r2 * r2;
  const double r6 = r2 * r4;
  const double temp = 1 + (k1 * r2) + (k2 * r4) + (k3 * r6);

  // Tangential distortion factor
  // clang-format off
  const double x_ddash = x_dash * temp + (2 * p1 * x_dash * y_dash + p2 * (r2 + 2 * (x_dash * x_dash)));
  const double y_ddash = y_dash * temp + (p1 * (r2 + 2 * (y_dash * y_dash)) + 2 * p2 * x_dash * y_dash);
  // clang-format on

  // Project rad-tan distorted point to image plane
  vec2_t distorted_point{x_ddash, y_ddash};
  return distorted_point;
}

matx_t equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const matx_t &points) {
  // Asserts
  assert(points.cols() == 3);

  // Setup
  const Eigen::ArrayXd z = points.col(2).array();
  const Eigen::ArrayXd x = points.col(0).array() / z.array();
  const Eigen::ArrayXd y = points.col(1).array() / z.array();
  const Eigen::ArrayXd r = (x.pow(2) + y.pow(2)).sqrt();

  // Apply equi distortion
  // clang-format off
  const Eigen::ArrayXd theta = r.atan();
  const Eigen::ArrayXd theta_d = theta + k1 * theta.pow(3) + k2 * theta.pow(5) + k3 * theta.pow(7) + k4 * theta.pow(9);
  const Eigen::ArrayXd x_dash = (theta_d / r) * x;
  const Eigen::ArrayXd y_dash = (theta_d / r) * y;
  // clang-format on

  // Project equi distorted points to image plane
  const int nb_points = points.rows();
  matx_t distorted_points{nb_points, 2};
  distorted_points.col(0) = x_dash;
  distorted_points.col(1) = y_dash;
  return distorted_points;
}

vec2_t equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const vec3_t &point) {
  const double z = point(2);
  const double x = point(0) / z;
  const double y = point(1) / z;
  const double r = sqrt(pow(x, 2) + pow(y, 2));

  // Apply equi distortion
  // clang-format off
  const double theta = atan(r);
  const double th2 = theta * theta;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double theta_d = theta * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (theta_d / r) * x;
  const double y_dash = (theta_d / r) * y;
  // clang-format on

  // Project equi distorted point to image plane
  return vec2_t{x_dash, y_dash};
}

void equi_undistort(const double k1,
                    const double k2,
                    const double k3,
                    const double k4,
                    vec2_t &p) {
  const double thetad = sqrt(p(0) * p(0) + p(1) * p(1));

  double theta = thetad; // Initial guess
  for (int i = 20; i > 0; i--) {
    const double th2 = theta * theta;
    const double th4 = th2 * th2;
    const double th6 = th4 * th2;
    const double th8 = th4 * th4;
    theta = thetad / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const double scaling = tan(theta) / thetad;
  p(0) *= scaling;
  p(1) *= scaling;
}

cv::Mat pinhole_equi_undistort_image(const mat3_t &K,
                                     const vecx_t &D,
                                     const cv::Mat &image,
                                     const double balance,
                                     cv::Mat &Knew) {
  // Estimate new camera matrix first
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(convert(K),
                                                          convert(D),
                                                          image.size(),
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, convert(K), convert(D), Knew);

  return image_ud;
}

cv::Mat pinhole_equi_undistort_image(const mat3_t &K,
                                     const vecx_t &D,
                                     const cv::Mat &image,
                                     cv::Mat &Knew) {
  return pinhole_equi_undistort_image(K, D, image, 0.0, Knew);
}

vec2_t project_pinhole_radtan(const mat3_t &K, const vecx_t &D, const vec3_t &X) {
  // Apply equi distortion
  const double k1 = D(0);
  const double k2 = D(1);
  const double p1 = D(2);
  const double p2 = D(3);
  const double k3 = D(4);
  const vec2_t x_distorted = radtan_distort(k1, k2, k3, p1, p2, X);

  // Project equi distorted point to image plane
  const vec2_t pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

vec2_t project_pinhole_equi(const mat3_t &K, const vec4_t &D, const vec3_t &X) {
  // Distort point
  const vec2_t x_distorted = equi_distort(D(0), D(1), D(2), D(3), X);

  // Project equi distorted point to image plane
  const vec2_t pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

} //  namespace prototype
