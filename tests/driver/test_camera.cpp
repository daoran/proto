#include "proto/munit.hpp"
#include "proto/driver/camera.hpp"

#define TEST_CONFIG "test_configs/camera/webcam/config.yaml"

namespace proto {

int test_camera_config() {
  camera_config_t config{TEST_CONFIG};

  MU_FALSE(config.ok);
  MU_CHECK(0 == config.index);
  MU_CHECK(0 == config.image_width);
  MU_CHECK(0 == config.image_height);

  camera_config_load(config);
  MU_CHECK(config.ok);
  MU_CHECK(0 == config.index);
  MU_CHECK(640 == config.image_width);
  MU_CHECK(480 == config.image_height);

  return 0;
}

int test_camera() {
  camera_t cam;

  camera_connect(cam);
  camera_get_frame(cam);
  camera_disconnect(cam);

  // cv::imshow("Windows", cam.image);
  // cv::waitKey(1);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_camera_config);
  MU_ADD_TEST(test_camera);
}

} // namespace proto
MU_RUN_TESTS(proto::test_suite);
