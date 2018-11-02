#include "prototype/vision/camera/equi.hpp"

namespace prototype {

vec2_t distort(const equi4_t &equi, const vec2_t &point) {
  const double k1 = equi.k1;
  const double k2 = equi.k2;
  const double k3 = equi.k3;
  const double k4 = equi.k4;
  const double x = point(0);
  const double y = point(1);
  const double r = sqrt(pow(x, 2) + pow(y, 2));

  // Apply equi distortion
  // clang-format off
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double th_d = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (th_d / r) * x;
  const double y_dash = (th_d / r) * y;
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
  const auto th3 = th.pow(3);
  const auto th5 = th.pow(5);
  const auto th7 = th.pow(7);
  const auto th9 = th.pow(9);
  const auto th_d = th + k1 * th3 + k2 * th5 + k3 * th7 + k4 * th9;
  const auto x_dash = (th_d / r) * x;
  const auto y_dash = (th_d / r) * y;

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
