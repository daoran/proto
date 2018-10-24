#include "prototype/munit.hpp"
#include "dataset/triclops/gimbal_data.hpp"

namespace prototype {
namespace triclops {

#define TEST_DATA "test_data/triclops/gimbal"

int test_GimbalData_constructor() {
  GimbalData gimbal_data;

  // Data
  MU_CHECK_EQ(0, gimbal_data.timestamps.size());
  MU_CHECK_EQ(0, gimbal_data.time.size());
  MU_CHECK_EQ(0, gimbal_data.joint_angles.size());

  return 0;
}

int test_GimbalData_load() {
  GimbalData gimbal_data;

  int retval = gimbal_data.load(TEST_DATA);

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(91, gimbal_data.timestamps.size());
  MU_CHECK_EQ(91, gimbal_data.time.size());
  MU_CHECK_EQ(91, gimbal_data.joint_angles.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalData_constructor);
  MU_ADD_TEST(test_GimbalData_load);
}

} // namespace triclops
} // namespace prototype

MU_RUN_TESTS(prototype::triclops::test_suite);
