#include "prototype/munit.hpp"
#include "dataset/triclops/imu_data.hpp"

namespace prototype {
namespace triclops {

#define TEST_DATA "test_data/triclops/imu0"

int test_IMUData_constructor() {
  IMUData imu_data;

  // Data
  MU_CHECK_EQ(0, imu_data.timestamps.size());
  MU_CHECK_EQ(0, imu_data.time.size());
  MU_CHECK_EQ(0, imu_data.w_B.size());
  MU_CHECK_EQ(0, imu_data.a_B.size());

  return 0;
}

int test_IMUData_load() {
  IMUData imu_data;

  int retval = imu_data.load(TEST_DATA);

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(91, imu_data.timestamps.size());
  MU_CHECK_EQ(91, imu_data.time.size());
  MU_CHECK_EQ(91, imu_data.w_B.size());
  MU_CHECK_EQ(91, imu_data.a_B.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_IMUData_constructor);
  MU_ADD_TEST(test_IMUData_load);
}

} // namespace triclops
} // namespace prototype

MU_RUN_TESTS(prototype::triclops::test_suite);
