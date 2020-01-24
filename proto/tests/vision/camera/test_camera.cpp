#include "proto/munit.hpp"
#include "proto/vision/camera/camera.hpp"
#include "proto/vision/vision_common.hpp"

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

/****************************************************************************
 *                           RADIAL-TANGENTIAL
 ***************************************************************************/

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

/****************************************************************************
 *                            EQUI-DISTANT
 ***************************************************************************/

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

    // Debug
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

/****************************************************************************
 *                                PINHOLE
 ***************************************************************************/

int test_pinhole_constructor() {
  pinhole_t pinhole;

  MU_CHECK_FLOAT(0.0, pinhole.fx);
  MU_CHECK_FLOAT(0.0, pinhole.fy);
  MU_CHECK_FLOAT(0.0, pinhole.cx);
  MU_CHECK_FLOAT(0.0, pinhole.cy);

  return 0;
}

int test_pinhole_K() {
  struct test_config config;
  pinhole_t pinhole{config.fx, config.fy, config.cx, config.cy};
  mat3_t K = pinhole_K(config.fx, config.fy, config.cx, config.cy);
  MU_CHECK((K - pinhole_K(pinhole)).norm() < 1e-4);

  return 0;
}

int test_pinhole_P() {
  struct test_config config;
  pinhole_t pinhole = setup_pinhole_model();

  mat3_t R = euler321(vec3_t{0.0, 0.0, 0.0});
  vec3_t t{1.0, 2.0, 3.0};
  mat34_t P = pinhole_P(pinhole_K(pinhole), R, t);

  mat34_t P_expected;
  // clang-format off
  P_expected << config.fx, 0.0, config.cx, -1514.26,
                0.0, config.fy, config.cy, -2068.51,
                0.0, 0.0, 1.0, -3.0;
  // clang-format on

  MU_CHECK(((P - P_expected).norm() < 0.01));

  return 0;
}

int test_pinhole_focal_length() {
  const double fov = 90.0;
  const double fx = pinhole_focal_length(600, fov);
  const double fy = pinhole_focal_length(600, fov);
  MU_CHECK_FLOAT(300.0, fy);
  MU_CHECK_FLOAT(fx, fy);

  const vec2_t image_size{600, 600};
  const vec2_t focal_length = pinhole_focal_length(image_size, fov, fov);
  MU_CHECK_FLOAT(fx, focal_length(0));
  MU_CHECK_FLOAT(fy, focal_length(1));

  return 0;
}

int test_pinhole_project() {
  pinhole_t pinhole = setup_pinhole_model();
  vec3_t p{0.0, 0.0, 10.0};

  vec2_t x = project(pinhole, p);
  MU_CHECK_FLOAT(320.0, x(0));
  MU_CHECK_FLOAT(320.0, x(1));

  return 0;
}

/****************************************************************************
 *                          CAMERA GEOMETRY
 ***************************************************************************/

int test_camera_geometry_project_pinhole_radtan() {
  const pinhole_t camera_model = setup_pinhole_model();
  const radtan4_t distortion_model{0.1, 0.01, 0.01, 0.01};
  camera_geometry_t<pinhole_t, radtan4_t> camera(camera_model,
                                                 distortion_model);

  const vec3_t point{0.1, 0.2, 10.0};
  const vec2_t pixel = camera_geometry_project(camera, point);
  std::cout << pixel.transpose() << std::endl;

  return 0;
}

int test_camera_geometry_project_pinhole_equi() {
  const pinhole_t camera_model = setup_pinhole_model();
  const equi4_t distortion_model{0.1, 0.01, 0.01, 0.01};
  camera_geometry_t<pinhole_t, equi4_t> camera(camera_model, distortion_model);

  const vec3_t point{0.1, 0.2, 10.0};
  const vec2_t pixel = camera_geometry_project(camera, point);
  std::cout << pixel.transpose() << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_radtan_distort_point);
  MU_ADD_TEST(test_radtan_distort_points);
  MU_ADD_TEST(test_radtan_undistort_point);

  MU_ADD_TEST(test_equi_distort_point);
  MU_ADD_TEST(test_equi_distort_points);
  MU_ADD_TEST(test_equi_undistort_point);

  MU_ADD_TEST(test_pinhole_constructor);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_P);
  MU_ADD_TEST(test_pinhole_focal_length);
  MU_ADD_TEST(test_pinhole_project);

  MU_ADD_TEST(test_camera_geometry_project_pinhole_radtan);
  MU_ADD_TEST(test_camera_geometry_project_pinhole_equi);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
