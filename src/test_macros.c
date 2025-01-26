#include "munit.h"
#include "macros.h"

int test_median_value(void) {
  real_t median = 0.0f;
  real_t buf[5] = {4.0, 1.0, 0.0, 3.0, 2.0};
  MEDIAN_VALUE(real_t, fltcmp2, buf, 5, median);
  MU_ASSERT(fltcmp(median, 2.0) == 0);

  return 0;
}

int test_mean_value(void) {
  real_t mean = 0.0f;
  real_t buf[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
  MEAN_VALUE(real_t, buf, 5, mean);
  MU_ASSERT(fltcmp(mean, 2.0) == 0);

  return 0;
}

void test_suite(void) {
  // MACROS
  MU_ADD_TEST(test_median_value);
  MU_ADD_TEST(test_mean_value);
}
MU_RUN_TESTS(test_suite)
