#include "prototype/munit.hpp"
#include "dataset/kitti/raw/raw.hpp"

namespace prototype {

#define TEST_DATA_PATH "test_data/kitti/raw/"

int test_RAW_load() {
  RawDataset raw_dataset(TEST_DATA_PATH, "2011_09_26", "0001");
  MU_CHECK(raw_dataset.load() == 0);
  return 0;
}

void test_suite() { MU_ADD_TEST(test_RAW_load); }

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
