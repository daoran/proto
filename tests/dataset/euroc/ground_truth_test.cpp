#include "prototype/dataset/euroc/ground_truth.hpp"
#include "prototype/munit.hpp"

namespace prototype {

#define TEST_DATA "test_data/euroc/state_groundtruth_estimate0"

int test_ground_truth_constructor() {
  ground_truth_t ground_truth;

  MU_CHECK_EQ(0, ground_truth.timestamps.size());
  MU_CHECK_EQ(0, ground_truth.time.size());
  MU_CHECK_EQ(0, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.q_RS.size());
  MU_CHECK_EQ(0, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(0, ground_truth.b_a_RS_S.size());

  return 0;
}

int test_ground_truth_load() {
  ground_truth_t ground_truth{TEST_DATA};

  int retval = ground_truth_load(ground_truth);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(999, ground_truth.timestamps.size());
  MU_CHECK_EQ(999, ground_truth.time.size());
  MU_CHECK_EQ(999, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.q_RS.size());
  MU_CHECK_EQ(999, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(999, ground_truth.b_a_RS_S.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_ground_truth_constructor);
  MU_ADD_TEST(test_ground_truth_load);
}
} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
