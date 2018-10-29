#include "prototype/vision/camera/equi.hpp"

namespace prototype {

matx_t distort(const equi_t &equi, const matx_t &points) {
  assert(points.cols() == 3);

  // Setup
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
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

vec2_t distort(const equi_t &equi, const vec3_t &point) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
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

void undistort(const equi_t &equi, vec2_t &p) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
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

cv::Mat undistort_image(const mat3_t &K,
                        const vecx_t &D,
                        const cv::Mat &image,
                        cv::Mat &Knew,
                        const double balance) {
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

} //  namespace prototype
