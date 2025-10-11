#include <gtest/gtest.h>

#include "PinholeEqui4.hpp"

namespace xyz {

static void setup_pinhole_equi4_test(Vec2i &cam_res, VecX &params) {
  cam_res = Vec2i{640, 480};

  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double k3 = 0.001;
  const double k4 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, k3, k4;
}

TEST(PinholeEqui4, project) {
  // Setup
  PinholeEqui4 camera;
  Vec2i cam_res;
  VecX params;
  setup_pinhole_equi4_test(cam_res, params);

  // Test pinhole equi4 project
  const Vec3 p_C{0.1, 0.2, 1.0};
  Vec2 z_hat;
  int retval = camera.project(cam_res, params, p_C, z_hat);
  ASSERT_TRUE(retval == 0);

  // Test OpenCV's version
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double k1 = params(4);
  const double k2 = params(5);
  const double k3 = params(6);
  const double k4 = params(7);
  const cv::Point3f obj_pt(p_C.x(), p_C.y(), p_C.z());
  const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  const cv::Mat D = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  const std::vector<cv::Point3f> object_points = {obj_pt};
  std::vector<cv::Point2f> image_points;
  cv::fisheye::projectPoints(object_points, image_points, rvec, tvec, K, D);

  const auto z_hat_gnd = Vec2{image_points[0].x, image_points[0].y};
  ASSERT_TRUE(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  ASSERT_TRUE(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);
}

TEST(PinholeEqui4, project_jacobian) {
  // Setup
  PinholeEqui4 camera;
  Vec2i cam_res;
  VecX params;
  setup_pinhole_equi4_test(cam_res, params);

  // Analytical jacobian
  const Vec3 p_C{0.1, 0.2, 1.0};
  const MatX J = camera.project_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  MatX fdiff = zeros(2, 3);

  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 3; i++) {
    Vec3 p_C_diff = p_C;
    p_C_diff(i) += step;

    Vec2 z_hat_prime;
    camera.project(cam_res, params, p_C_diff, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(PinholeEqui4, params_jacobian) {
  // Setup
  PinholeEqui4 camera;
  Vec2i cam_res;
  VecX params;
  setup_pinhole_equi4_test(cam_res, params);

  // Analytical jacobian
  const Vec3 p_C{0.1, 0.2, 5.0};
  const MatX J = camera.params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  MatX fdiff = zeros(2, 8);

  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    VecX params_diff = params;
    params_diff(i) += step;

    Vec2 z_hat_prime;
    camera.project(cam_res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(PinholeEqui4, undistort) {
  // Setup
  PinholeEqui4 camera;
  Vec2i cam_res;
  VecX params;
  setup_pinhole_equi4_test(cam_res, params);

  // Project without distortion
  const Vec3 p_C{0.1, 0.2, 1.0};
  Vec2 z_hat_gnd;
  pinhole_project(cam_res, params.head(4), p_C, z_hat_gnd);

  // Project with distortion and undistort
  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);
  z_hat = camera.undistort(params, z_hat);

  ASSERT_FLOAT_EQ(z_hat_gnd.x(), z_hat.x());
  ASSERT_FLOAT_EQ(z_hat_gnd.y(), z_hat.y());
}

} // namespace xyz
