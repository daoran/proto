#include "prototype/munit.hpp"
#include "prototype/core.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/equi.hpp"

namespace prototype {

struct test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
};

pinhole_t setup_pinhole_model() {
  struct test_config config;
  return pinhole_t{config.fx, config.fy, config.cx, config.cy};
}

int test_equi_distort_point() {
  const int nb_points = 100;

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    equi4_t equi{0.1, 0.01, 0.01, 0.01};
    vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    vec2_t p{point(0) / point(2), point(1) / point(2)};
    vec2_t p_d = distort(equi, p);

    // Use opencv to use equi distortion to distort point
    const std::vector<cv::Point2f> points{cv::Point2f(p(0), p(1))};
    const cv::Mat K = convert(pinhole_K(1.0, 1.0, 0.0, 0.0));
    const cv::Vec4f D(equi.k1, equi.k2, equi.k3, equi.k4);
    std::vector<cv::Point2f> image_points;
    cv::fisheye::distortPoints(points, image_points, K, D);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p_d - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_equi_distort_points() {
  // Setup
  int nb_points = 100;
  equi4_t equi{0.1, 0.01, 0.01, 0.01};
  matx_t points;
  points.resize(2, nb_points);

  std::vector<cv::Point2f> cv_points;
  for (int i = 0; i < nb_points; i++) {
    vec3_t p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    points.block(0, i, 2, 1) = vec2_t{p(0) / p(2), p(1) / p(2)};
    cv_points.emplace_back(p(0) / p(2), p(1) / p(2));
  }

  // Use opencv to use equi distortion to distort point
  std::vector<cv::Point2f> expected_points;
  const cv::Mat K = convert(pinhole_K(1.0, 1.0, 0.0, 0.0));
  const cv::Vec4f D(equi.k1, equi.k2, equi.k3, equi.k4);
  cv::fisheye::distortPoints(cv_points, expected_points, K, D);

  // Distort points
  matx_t points_distorted = distort(equi, points);
  for (int i = 0; i < nb_points; i++) {
    const auto p_dist = points_distorted.block(0, i, 2, 1);
    const auto expected = vec2_t{expected_points[i].x, expected_points[i].y};

    // // Debug
    // std::cout << i << std::endl;
    // std::cout << p_dist.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p_dist - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_equi_undistort_point() {
  const int nb_points = 100;

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const equi4_t equi{0.1, 0.2, 0.3, 0.4};
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = distort(equi, p);
    const vec2_t p_ud = undistort(equi, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p - p_ud).norm() < 1.0e-5);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_equi_distort_point);
  MU_ADD_TEST(test_equi_distort_points);
  MU_ADD_TEST(test_equi_undistort_point);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
