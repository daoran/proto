#include "prototype/munit.hpp"

namespace prototype {

int test_constructor() { return 0; }
int test_load() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_constructor);
  MU_ADD_TEST(test_load);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
