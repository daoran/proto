#include "prototype/munit.hpp"
#include "prototype/vision/vision_common.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/radtan.hpp"

namespace proto {

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

int test_radtan_distort_point() {
  const int nb_points = 100;

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    radtan4_t radtan{0.1, 0.01, 0.01, 0.01};
    vec3_t p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    vec2_t pixel = distort(radtan, vec2_t{p(0) / p(2), p(1) / p(2)});

    // Use opencv to use radtan distortion to distort point
    const std::vector<cv::Point3f> points{cv::Point3f(p(0), p(1), p(2))};
    const cv::Vec3f rvec;
    const cv::Vec3f tvec;
    const cv::Mat K = convert(pinhole_K(1.0, 1.0, 0.0, 0.0));
    const cv::Vec4f D(radtan.k1, radtan.k2, radtan.p1, radtan.p2);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(points, rvec, tvec, K, D, image_points);
    const vec2_t expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((pixel - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_radtan_distort_points() {
  // Setup
  int nb_points = 100;
  radtan4_t radtan{0.1, 0.01, 0.01, 0.01};
  matx_t points;
  points.resize(2, nb_points);

  std::vector<cv::Point3f> cv_points;
  for (int i = 0; i < nb_points; i++) {
    vec3_t p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    points.block(0, i, 2, 1) = vec2_t{p(0) / p(2), p(1) / p(2)};
    cv_points.emplace_back(p(0), p(1), p(2));
  }

  // Use opencv to use radtan distortion to distort point
  const cv::Vec3f rvec;
  const cv::Vec3f tvec;
  const cv::Mat K = convert(pinhole_K(1.0, 1.0, 0.0, 0.0));
  const cv::Vec4f D(radtan.k1, radtan.k2, radtan.p1, radtan.p2);
  std::vector<cv::Point2f> expected_points;
  cv::projectPoints(cv_points, rvec, tvec, K, D, expected_points);

  // Distort points
  matx_t pixels = distort(radtan, points);
  for (int i = 0; i < nb_points; i++) {
    const auto pixel = pixels.block(0, i, 2, 1);
    const auto expected = vec2_t{expected_points[i].x, expected_points[i].y};

    // // Debug
    // std::cout << i << std::endl;
    // std::cout << cv_points[i] << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((pixel - expected).norm() < 1.0e-5);
  }

  return 0;
}

int test_radtan_undistort_point() {
  const int nb_points = 100;

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const radtan4_t radtan{0.1, 0.02, 0.03, 0.04};
    const vec3_t point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const vec2_t p{point(0) / point(2), point(1) / point(2)};
    const vec2_t p_d = distort(radtan, p);
    const vec2_t p_ud = undistort(radtan, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    MU_CHECK((p - p_ud).norm() < 1.0e-5);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_radtan_distort_point);
  MU_ADD_TEST(test_radtan_distort_points);
  MU_ADD_TEST(test_radtan_undistort_point);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
