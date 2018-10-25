#include "prototype/munit.hpp"
#include "prototype/dataset/kitti/raw/oxts.hpp"

#ifndef TEST_KITTI_DATA
#define TEST_KITTI_DATA                                                        \
  "test_data/kitti/raw/2011_09_26/2011_09_26_drive_0001_sync"
#endif

#include <limits>

namespace prototype {

int test_oxts_load() {
  oxts_t oxts;
  oxts_load(oxts, TEST_KITTI_DATA "/oxts");

  for (size_t i = 0; i < oxts.timestamps.size(); i++) {
    MU_CHECK(oxts.time[i] >= 0.0);
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_oxts_load); }

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
