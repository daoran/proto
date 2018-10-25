#include "prototype/munit.hpp"
#include "prototype/dataset/kitti/raw/raw.hpp"

namespace prototype {

#define TEST_DATA_PATH "test_data/kitti/raw/"

int test_kitti_raw_load() {
  kitti_raw_dataset_t raw_dataset(TEST_DATA_PATH, "2011_09_26", "0001");
  MU_CHECK(kitti_raw_dataset_load(raw_dataset) == 0);
  return 0;
}

void test_suite() { MU_ADD_TEST(test_kitti_raw_load); }

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
