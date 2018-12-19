#include "prototype/munit.hpp"
#include "prototype/mav/pos_ctrl.hpp"

namespace proto {

#define TEST_CONFIG "test_data/mav/pos_ctrl.yaml"

int test_pos_ctrl_constructor() {
  pos_ctrl_t p_ctrl;

  MU_CHECK_FLOAT(0.0, p_ctrl.dt);

  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(0));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(1));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(2));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(3));

  MU_CHECK_FLOAT(-30.0, p_ctrl.roll_limit[0]);
  MU_CHECK_FLOAT(30.0, p_ctrl.roll_limit[1]);
  MU_CHECK_FLOAT(-30.0, p_ctrl.pitch_limit[0]);
  MU_CHECK_FLOAT(30.0, p_ctrl.pitch_limit[1]);
  MU_CHECK_FLOAT(0.5, p_ctrl.hover_throttle);

  return 0;
}

int test_pos_ctrl_configure() {
  pos_ctrl_t p_ctrl;
  if (pos_ctrl_configure(p_ctrl, TEST_CONFIG) != 0) {
    LOG_ERROR("Failed to configure position controller!");
    return -1;
  }

  MU_CHECK_FLOAT(0.0, p_ctrl.dt);

  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(0));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(1));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(2));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(3));

  MU_CHECK_FLOAT(-50.0, rad2deg(p_ctrl.roll_limit[0]));
  MU_CHECK_FLOAT(50.0, rad2deg(p_ctrl.roll_limit[1]));
  MU_CHECK_FLOAT(-50.0, rad2deg(p_ctrl.pitch_limit[0]));
  MU_CHECK_FLOAT(50.0, rad2deg(p_ctrl.pitch_limit[1]));
  MU_CHECK_FLOAT(0.6, p_ctrl.hover_throttle);

  MU_CHECK_FLOAT(0.1, p_ctrl.x_ctrl.k_p);
  MU_CHECK_FLOAT(0.2, p_ctrl.x_ctrl.k_i);
  MU_CHECK_FLOAT(0.3, p_ctrl.x_ctrl.k_d);

  MU_CHECK_FLOAT(0.1, p_ctrl.y_ctrl.k_p);
  MU_CHECK_FLOAT(0.2, p_ctrl.y_ctrl.k_i);
  MU_CHECK_FLOAT(0.3, p_ctrl.y_ctrl.k_d);

  MU_CHECK_FLOAT(0.1, p_ctrl.z_ctrl.k_p);
  MU_CHECK_FLOAT(0.2, p_ctrl.z_ctrl.k_i);
  MU_CHECK_FLOAT(0.3, p_ctrl.z_ctrl.k_d);

  return 0;
}

// int test_pos_ctrl_update() {
//   vec3_t setpoint_nwu;
//   Pose actual;
//   float yaw_setpoint, dt;
//   pos_ctrl p_ctrl;
//
//   // setup
//   pos_ctrl_configure(p_ctrl, TEST_CONFIG);
//
//   // CHECK HOVERING PID OUTPUT
//   setpoint_nwu << 0, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   std::cout << outputs.transpose() << std::endl;
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//   MU_CHECK_FLOAT(controller.hover_throttle, controller.outputs(3));
//
//   // CHECK MOVING TOWARDS THE Y LOCATION
//   setpoint_nwu << 0, 1, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   pos_ctrl_reset(p_ctrl);
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   std::cout << outputs.transpose() << std::endl;
//
//   MU_CHECK(controller.outputs(0) < 0.0);
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//
//   // CHECK MOVING TOWARDS THE X LOCATION
//   setpoint_nwu << 1, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   pos_ctrl_reset(p_ctrl);
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   std::cout << outputs.transpose() << std::endl;
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK(controller.outputs(1) > 0.0);
//
//   // CHECK MOVING TOWARDS THE X AND Y LOCATION
//   setpoint_nwu << 1, 1, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   pos_ctrl_reset(p_ctrl);
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   std::cout << outputs.transpose() << std::endl;
//
//   MU_CHECK(controller.outputs(0) < 0.0);
//   MU_CHECK(controller.outputs(1) > 0.0);
//
//   // CHECK MOVING YAW
//   setpoint_nwu << 0, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = deg2rad(90.0);
//   dt = 0.1;
//
//   pos_ctrl_reset(p_ctrl);
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   std::cout << outputs.transpose() << std::endl;
//
//   MU_CHECK_FLOAT(yaw_setpoint, controller.outputs(2));
// }

// int test_pos_ctrl_update2() {
//   vec3_t setpoint_nwu, euler;
//   Pose actual;
//   float yaw_setpoint, dt;
//   pos_ctrl controller;
//
//   // setup
//   controller.configure(TEST_CONFIG);
//
//   // CHECK HEADING AT 90 DEGREE
//   setpoint_nwu << 1, 0, 0; // setpoint infront of quad
//   actual.position << 0, 0, 0;
//   euler << 0.0, 0.0, deg2rad(90.0);
//   actual.orientation = euler321ToQuat(euler);
//
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   pos_ctrl_update(p_ctrl, setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK(controller.outputs(0) > 0);
//   MU_CHECK_NEAR(0.0, controller.outputs(1), 0.1);
//   MU_CHECK_NEAR(controller.hover_throttle, controller.outputs(3), 0.01);
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_pos_ctrl_constructor);
  MU_ADD_TEST(test_pos_ctrl_configure);
  // MU_ADD_TEST(test_pos_ctrl_update);
  // MU_ADD_TEST(test_pos_ctrl_update2);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
