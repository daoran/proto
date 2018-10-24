#include "prototype/munit.hpp"
#include "dataset/triclops/camera_data.hpp"

namespace prototype {
namespace triclops {

#define TEST_DATA "test_data/triclops/cam0"

int test_CameraData_constructor() {
  CameraData camera_data;

  MU_CHECK_EQ(0, camera_data.timestamps.size());
  MU_CHECK_EQ(0, camera_data.time.size());
  MU_CHECK_EQ(0, camera_data.image_paths.size());

  return 0;
}

int test_CameraData_load() {
  CameraData camera_data;

  int retval = camera_data.load(TEST_DATA);
  MU_CHECK_EQ(0.0, retval);
  MU_CHECK_EQ(10, camera_data.timestamps.size());
  MU_CHECK_EQ(10, camera_data.time.size());
  MU_CHECK_EQ(10, camera_data.image_paths.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraData_constructor);
  MU_ADD_TEST(test_CameraData_load);
}

}  // namespace triclops
}  // namespace prototype

MU_RUN_TESTS(prototype::triclops::test_suite);
