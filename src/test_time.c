#include "munit.h"
#include "time.h"


int test_tic_toc(void) {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(toc(&t_start) - 1e-3) < 1e-2);
  return 0;
}

int test_mtoc(void) {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(mtoc(&t_start) - 1e-3) < 1);
  return 0;
}

int test_time_now(void) {
  timestamp_t t_now = time_now();
  // printf("t_now: %ld\n", t_now);
  MU_ASSERT(t_now > 0);
  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_tic_toc);
  MU_ADD_TEST(test_mtoc);
  MU_ADD_TEST(test_time_now);
}
MU_RUN_TESTS(test_suite)
