#include <unistd.h>

#include "proto/core/time.hpp"
#include "proto/munit.hpp"

namespace proto {

int test_ticAndToc() {
  struct timespec start = tic();
  usleep(10 * 1000);
  MU_CHECK(toc(&start) < 0.011);
  MU_CHECK(toc(&start) > 0.009);
  MU_CHECK(mtoc(&start) < 11.0);
  MU_CHECK(mtoc(&start) > 9.0);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_ticAndToc); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
