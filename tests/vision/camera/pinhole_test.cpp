#include "prototype/munit.hpp"
#include "prototype/vision/camera/pinhole.hpp"

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
  MU_CHECK((K - pinhole.K).norm() < 1e-4);

  return 0;
}

int test_pinhole_P() {
  struct test_config config;
  pinhole_t pinhole = setup_pinhole_model();

  mat3_t R = euler321ToRot(vec3_t{0.0, 0.0, 0.0});
  vec3_t t{1.0, 2.0, 3.0};
  mat34_t P = pinhole_P(pinhole.K, R, t);

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
  mat3_t R = euler321ToRot(vec3_t{0.0, 0.0, 0.0});
  vec3_t t{0.0, 0.0, 0.0};
  vec3_t p{0.0, 0.0, 10.0};

  vec2_t x = project(pinhole, p);
  MU_CHECK_FLOAT(320.0, x(0));
  MU_CHECK_FLOAT(320.0, x(1));

  vec2_t y = project(pinhole, R, t, p);
  MU_CHECK_FLOAT(320.0, y(0));
  MU_CHECK_FLOAT(320.0, y(1));

  return 0;
}

int test_pinhole_pixel2ideal() {
  pinhole_t pinhole = setup_pinhole_model();
  vec2_t point = pixel2ideal(pinhole, vec2_t{320, 320});

  MU_CHECK_FLOAT(0.0, point(0));
  MU_CHECK_FLOAT(0.0, point(1));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_pinhole_constructor);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_P);
  MU_ADD_TEST(test_pinhole_focal_length);
  MU_ADD_TEST(test_pinhole_project);
  MU_ADD_TEST(test_pinhole_pixel2ideal);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
