#include "prototype/munit.hpp"
#include "prototype/mav/att_ctrl.hpp"

namespace proto {

#define TEST_CONFIG "test_data/mav/att_ctrl.yaml"

int test_att_ctrl_constructor() {
  att_ctrl_t att_ctrl;

  MU_CHECK_FLOAT(0.0, att_ctrl.dt);
  MU_CHECK((zeros(4, 1) - att_ctrl.outputs).norm() < 1e-5);

  return 0;
}

int test_att_ctrl_load() {
  att_ctrl_t att_ctrl;
  if (att_ctrl_configure(att_ctrl, TEST_CONFIG) != 0) {
    LOG_ERROR("Failed to configure attitude controller!");
    return -1;
  }

  MU_CHECK_FLOAT(0.0, att_ctrl.dt);
  MU_CHECK((zeros(4, 1) - att_ctrl.outputs).norm() < 1e-5);

  MU_CHECK_FLOAT(-35.0, rad2deg(att_ctrl.roll_limit[0]));
  MU_CHECK_FLOAT(35.0, rad2deg(att_ctrl.roll_limit[1]));
  MU_CHECK_FLOAT(-35.0, rad2deg(att_ctrl.pitch_limit[0]));
  MU_CHECK_FLOAT(35.0, rad2deg(att_ctrl.pitch_limit[1]));

  MU_CHECK_FLOAT(200.0, att_ctrl.roll_ctrl.k_p);
  MU_CHECK_FLOAT(0.001, att_ctrl.roll_ctrl.k_i);
  MU_CHECK_FLOAT(1.0, att_ctrl.roll_ctrl.k_d);

  MU_CHECK_FLOAT(200.0, att_ctrl.pitch_ctrl.k_p);
  MU_CHECK_FLOAT(0.001, att_ctrl.pitch_ctrl.k_i);
  MU_CHECK_FLOAT(1.0, att_ctrl.pitch_ctrl.k_d);

  MU_CHECK_FLOAT(10.0, att_ctrl.yaw_ctrl.k_p);
  MU_CHECK_FLOAT(0.001, att_ctrl.yaw_ctrl.k_i);
  MU_CHECK_FLOAT(1.0, att_ctrl.yaw_ctrl.k_d);

  return 0;
}

int test_att_ctrl_update() {
  att_ctrl_t att_ctrl;
  if (att_ctrl_configure(att_ctrl, TEST_CONFIG) != 0) {
    LOG_ERROR("Failed to configure attitude controller!");
    return -1;
  }

  const vec4_t setpoints;
  const mat4_t T_WB;
  const double dt = 0.002;
  att_ctrl_update(att_ctrl, setpoints, T_WB, dt);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_att_ctrl_constructor);
  MU_ADD_TEST(test_att_ctrl_load);
  MU_ADD_TEST(test_att_ctrl_update);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
