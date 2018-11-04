#include "prototype/munit.hpp"
#include "prototype/core.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/equi.hpp"
#include "prototype/vision/camera/radtan.hpp"
#include "prototype/vision/camera/camera_geometry.hpp"

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

int test_camera_geometry_project_pinhole_radtan() {
  const pinhole_t camera_model = setup_pinhole_model();
  const radtan4_t distortion_model{0.1, 0.01, 0.01, 0.01};
  camera_geometry_t<pinhole_t, radtan4_t> camera(camera_model, distortion_model);

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
  MU_ADD_TEST(test_camera_geometry_project_pinhole_radtan);
  MU_ADD_TEST(test_camera_geometry_project_pinhole_equi);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
