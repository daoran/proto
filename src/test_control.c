#include "munit.h"
#include "xyz.h"
#include "xyz_control.h"

int test_pid_ctrl(void) {
  const real_t kp = 0.1;
  const real_t ki = 0.2;
  const real_t kd = 0.3;
  pid_ctrl_t pid;
  pid_ctrl_setup(&pid, kp, ki, kd);

  MU_ASSERT(fltcmp(pid.error_prev, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_sum, 0.0) == 0);

  MU_ASSERT(fltcmp(pid.error_p, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_i, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_d, 0.0) == 0);

  MU_ASSERT(fltcmp(pid.k_p, kp) == 0);
  MU_ASSERT(fltcmp(pid.k_i, ki) == 0);
  MU_ASSERT(fltcmp(pid.k_d, kd) == 0);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_pid_ctrl);
}
MU_RUN_TESTS(test_suite)
