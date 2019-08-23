#include "proto/control/pid.hpp"
#include "proto/munit.hpp"

namespace proto {

int test_pid_construct() {
  pid_t p;

  MU_CHECK_FLOAT(0.0, p.error_prev);
  MU_CHECK_FLOAT(0.0, p.error_sum);

  MU_CHECK_FLOAT(0.0, p.error_p);
  MU_CHECK_FLOAT(0.0, p.error_i);
  MU_CHECK_FLOAT(0.0, p.error_d);

  MU_CHECK_FLOAT(0.0, p.k_p);
  MU_CHECK_FLOAT(0.0, p.k_i);
  MU_CHECK_FLOAT(0.0, p.k_d);

  return 0;
}

int test_pid_setup() {
  pid_t p(1.0, 2.0, 3.0);

  MU_CHECK_FLOAT(0.0, p.error_prev);
  MU_CHECK_FLOAT(0.0, p.error_sum);

  MU_CHECK_FLOAT(0.0, p.error_p);
  MU_CHECK_FLOAT(0.0, p.error_i);
  MU_CHECK_FLOAT(0.0, p.error_d);

  MU_CHECK_FLOAT(1.0, p.k_p);
  MU_CHECK_FLOAT(2.0, p.k_i);
  MU_CHECK_FLOAT(3.0, p.k_d);

  return 0;
}

int test_pid_update() {
  pid_t p(1.0, 2.0, 3.0);

  // test and assert
  double output = pid_update(p, 10.0, 0.0, 0.1);
  std::cout << p << std::endl;

  // MU_CHECK_FLOAT(1.0, p.error_sum);
  // MU_CHECK_FLOAT(10.0, p.error_p);
  // MU_CHECK_FLOAT(2.0, p.error_i);
  // MU_CHECK_FLOAT(300.0, p.error_d);
  // MU_CHECK_FLOAT(10.0, p.error_prev);
  // MU_CHECK_FLOAT(111.0, output);

  return 0;
}

int test_pid_reset() {
  pid_t p;

  p.error_prev = 0.1;
  p.error_sum = 0.2;

  p.error_p = 0.3;
  p.error_i = 0.4;
  p.error_d = 0.5;

  pid_reset(p);

  MU_CHECK_FLOAT(0.0, p.error_prev);
  MU_CHECK_FLOAT(0.0, p.error_sum);

  MU_CHECK_FLOAT(0.0, p.error_p);
  MU_CHECK_FLOAT(0.0, p.error_i);
  MU_CHECK_FLOAT(0.0, p.error_d);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_pid_construct);
  MU_ADD_TEST(test_pid_setup);
  MU_ADD_TEST(test_pid_update);
  MU_ADD_TEST(test_pid_reset);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
