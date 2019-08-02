#include "proto/munit.hpp"
#include "proto/mav/wp_ctrl.hpp"

#define TEST_CONFIG "tests/configs/control/waypoint_controller.yaml"

namespace proto {

int test_wp_ctrl_constructor() {
  wp_ctrl_t controller;

  MU_CHECK(controller.configured == false);

  MU_CHECK_FLOAT(0.0, controller.dt);

  MU_CHECK_FLOAT(0.0, controller.at_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.at_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.at_controller.k_d);
  MU_CHECK_FLOAT(0.0, controller.ct_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.ct_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.ct_controller.k_d);
  MU_CHECK_FLOAT(0.0, controller.z_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.z_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.z_controller.k_d);
  MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_d);

  MU_CHECK_FLOAT(0.0, controller.roll_limit[0]);
  MU_CHECK_FLOAT(0.0, controller.roll_limit[1]);
  MU_CHECK_FLOAT(0.0, controller.pitch_limit[0]);
  MU_CHECK_FLOAT(0.0, controller.pitch_limit[1]);
  MU_CHECK_FLOAT(0.0, controller.hover_throttle);

  MU_CHECK_FLOAT(0.0, controller.outputs(0));
  MU_CHECK_FLOAT(0.0, controller.outputs(1));
  MU_CHECK_FLOAT(0.0, controller.outputs(2));
  MU_CHECK_FLOAT(0.0, controller.outputs(3));

  return 0;
}

// int test_wp_ctrl_configure() {
//   wp_ctrl_t controller;
//
//   wp_ctrl_configure(controller, TEST_CONFIG);
//
//   MU_CHECK_FLOAT(deg2rad(-20.0), controller.roll_limit[0]);
//   MU_CHECK_FLOAT(deg2rad(20.0), controller.roll_limit[1]);
//
//   MU_CHECK_FLOAT(deg2rad(-20.0), controller.pitch_limit[0]);
//   MU_CHECK_FLOAT(deg2rad(20.0), controller.pitch_limit[1]);
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//   MU_CHECK_FLOAT(0.0, controller.outputs(2));
//   MU_CHECK_FLOAT(0.0, controller.outputs(3));
//
//   return 0;
// }

// int test_wp_ctrl_update() {
//   Mission mission;
//   wp_ctrl_t controller;
//
//   // setup
//   controller.configure(TEST_CONFIG);
//
//   // push waypoints in ENU coorindates
//   // note that the controller works in NWU coordinates
//   vec3_t wp;
//   wp << 0.0, 0.0, 10.0;
//   mission.wp_start = wp;
//   mission.local_waypoints.push_back(wp);
//
//   wp << 10.0, 6.0, 10.0;
//   mission.wp_end = wp;
//   mission.local_waypoints.push_back(wp);
//
//   wp << 17.0, -7.0, 10.0;
//   mission.local_waypoints.push_back(wp);
//
//   wp << 9.0, -10.0, 10.0;
//   mission.local_waypoints.push_back(wp);
//   mission.configured = true;
//
//   // update controller
//   Pose pose{WORLD_FRAME, 0.0, 0.0, deg2rad(-90.0), 0.0, 0.0, 10.0};
//   vec3_t vel{1.0, 0.0, 0.0};
//   double dt = 0.011;
//   controller.update(mission, pose, vel, dt);
//
//   // Pose pose2{0.0, 0.0, deg2rad(0.0), 5.0, 6.0, 10.0};
//   // controller.update(mission, pose2, vel, dt);
//   //
//   // Pose pose3{0.0, 0.0, deg2rad(0.0), 5.0, 6.0, 10.0};
//   // controller.update(mission, pose3, vel, dt);
// }

void test_suite() {
  MU_ADD_TEST(test_wp_ctrl_constructor);
  // MU_ADD_TEST(test_wp_ctrl_configure);
  // MU_ADD_TEST(test_pos_ctrl_update2);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
