#include "proto/munit.hpp"
#include "proto/viz/camera.hpp"

using namespace proto;

int test_glcamera() {
  glcamera_t camera(0, 0, glm::vec3{0.0, 0.0, 0.0});

  MU_CHECK_FLOAT(camera.world_up.x, 0.0);
  MU_CHECK_FLOAT(camera.world_up.y, 1.0);
  MU_CHECK_FLOAT(camera.world_up.z, 0.0);

  MU_CHECK_FLOAT(camera.position.x, 0.0);
  MU_CHECK_FLOAT(camera.position.y, 0.0);
  MU_CHECK_FLOAT(camera.position.z, 0.0);

  MU_CHECK_FLOAT(camera.front.x, 0.0);
  MU_CHECK_FLOAT(camera.front.y, 0.0);
  MU_CHECK_FLOAT(camera.front.z, -1.0);

  MU_CHECK_FLOAT(camera.up.x, 0.0);
  MU_CHECK_FLOAT(camera.up.y, 1.0);
  MU_CHECK_FLOAT(camera.up.z, 0.0);

  MU_CHECK_FLOAT(camera.right.x, -1.0);
  MU_CHECK_FLOAT(camera.right.y, 0.0);
  MU_CHECK_FLOAT(camera.right.z, 0.0);

  MU_CHECK_FLOAT(camera.yaw, -90.0);
  MU_CHECK_FLOAT(camera.pitch, 0.0);

  return 0;
}

int test_glcamera_projection() {
  glm::vec3 position{0.0, 0.0, 0.0};
  glcamera_t camera(640, 480, position);

  glm::mat4 P = glcamera_projection(camera);
  // std::cout << glm::to_string(P) << std::endl;

  return 0;
}

int test_glcamera_view_matrix() { return 0; }

int test_glcamera_update() { return 0; }

int test_glcamera_keyboard_handler() { return 0; }

int test_glcamera_mouse_handler() { return 0; }

int test_glcamera_scroll_handler() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_glcamera);
  MU_ADD_TEST(test_glcamera_projection);
  MU_ADD_TEST(test_glcamera_view_matrix);
  MU_ADD_TEST(test_glcamera_update);
  MU_ADD_TEST(test_glcamera_keyboard_handler);
  MU_ADD_TEST(test_glcamera_mouse_handler);
  MU_ADD_TEST(test_glcamera_scroll_handler);
}

MU_RUN_TESTS(test_suite);
