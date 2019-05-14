#include "prototype/munit.hpp"

namespace proto {

int test_constructor() { return 0; }
int test_load() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_constructor);
  MU_ADD_TEST(test_load);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
