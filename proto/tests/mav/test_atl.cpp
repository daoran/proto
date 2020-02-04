#include <iostream>
#include <fstream>

#include "proto/munit.hpp"
#include "proto/mav/atl.hpp"
#include "proto/model/model.hpp"

namespace proto {

/*****************************************************************************
 *                       ATTITUDE CONTROLLER
 ****************************************************************************/

#define TEST_ATT_CTRL_CONFIG "test_data/mav/att_ctrl.yaml"

int test_att_ctrl_constructor() {
  att_ctrl_t att_ctrl;

  MU_CHECK_FLOAT(0.0, att_ctrl.dt);
  MU_CHECK((zeros(4, 1) - att_ctrl.outputs).norm() < 1e-5);

  return 0;
}

int test_att_ctrl_load() {
  att_ctrl_t att_ctrl;
  if (att_ctrl_configure(att_ctrl, TEST_ATT_CTRL_CONFIG) != 0) {
    LOG_ERROR("Failed to configure attitude controller!");
    return -1;
  }

  MU_CHECK_FLOAT(0.0, att_ctrl.dt);
  MU_CHECK((zeros(4, 1) - att_ctrl.outputs).norm() < 1e-5);

  MU_CHECK_FLOAT(-35.0, rad2deg(att_ctrl.roll_limits[0]));
  MU_CHECK_FLOAT(35.0, rad2deg(att_ctrl.roll_limits[1]));
  MU_CHECK_FLOAT(-35.0, rad2deg(att_ctrl.pitch_limits[0]));
  MU_CHECK_FLOAT(35.0, rad2deg(att_ctrl.pitch_limits[1]));

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
  if (att_ctrl_configure(att_ctrl, TEST_ATT_CTRL_CONFIG) != 0) {
    LOG_ERROR("Failed to configure attitude controller!");
    return -1;
  }

  const vec4_t setpoints;
  const mat4_t T_WB;
  const double dt = 0.002;
  att_ctrl_update(att_ctrl, setpoints, T_WB, dt);

  return 0;
}

/*****************************************************************************
 *                          POSITION CONTROLLER
 ****************************************************************************/

#define TEST_POS_CTRL_CONFIG "test_data/mav/pos_ctrl.yaml"

int test_pos_ctrl_constructor() {
  pos_ctrl_t p_ctrl;

  MU_CHECK_FLOAT(0.0, p_ctrl.dt);

  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(0));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(1));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(2));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(3));

  MU_CHECK_FLOAT(-30.0, p_ctrl.roll_limits[0]);
  MU_CHECK_FLOAT(30.0, p_ctrl.roll_limits[1]);
  MU_CHECK_FLOAT(-30.0, p_ctrl.pitch_limits[0]);
  MU_CHECK_FLOAT(30.0, p_ctrl.pitch_limits[1]);
  MU_CHECK_FLOAT(0.5, p_ctrl.hover_throttle);

  return 0;
}

int test_pos_ctrl_configure() {
  pos_ctrl_t p_ctrl;
  if (pos_ctrl_configure(p_ctrl, TEST_POS_CTRL_CONFIG) != 0) {
    LOG_ERROR("Failed to configure position controller!");
    return -1;
  }

  MU_CHECK_FLOAT(0.0, p_ctrl.dt);

  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(0));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(1));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(2));
  MU_CHECK_FLOAT(0.0, p_ctrl.outputs(3));

  MU_CHECK_FLOAT(-50.0, rad2deg(p_ctrl.roll_limits[0]));
  MU_CHECK_FLOAT(50.0, rad2deg(p_ctrl.roll_limits[1]));
  MU_CHECK_FLOAT(-50.0, rad2deg(p_ctrl.pitch_limits[0]));
  MU_CHECK_FLOAT(50.0, rad2deg(p_ctrl.pitch_limits[1]));
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
//   pos_ctrl_configure(p_ctrl, TEST_POS_CTRL_CONFIG);
//
//   // CHECK HOVERING PID OUTPUT
//   setpoint_nwu << 0, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual,
//   yaw_setpoint, dt); std::cout << outputs.transpose() << std::endl;
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
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual,
//   yaw_setpoint, dt); std::cout << outputs.transpose() << std::endl;
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
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual,
//   yaw_setpoint, dt); std::cout << outputs.transpose() << std::endl;
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
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual,
//   yaw_setpoint, dt); std::cout << outputs.transpose() << std::endl;
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
//   const vec4_t outputs = pos_ctrl_update(p_ctrl, setpoint_nwu, actual,
//   yaw_setpoint, dt); std::cout << outputs.transpose() << std::endl;
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
//   controller.configure(TEST_POS_CTRL_CONFIG);
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

/*****************************************************************************
 *                          WAYPOINT CONTROLLER
 ****************************************************************************/

#define TEST_WP_CTRL_CONFIG "tests/configs/control/waypoint_controller.yaml"

// int test_wp_ctrl_constructor() {
//   wp_ctrl_t controller;
//
//   MU_CHECK(controller.configured == false);
//
//   MU_CHECK_FLOAT(0.0, controller.dt);
//
//   MU_CHECK_FLOAT(0.0, controller.at_controller.k_p);
//   MU_CHECK_FLOAT(0.0, controller.at_controller.k_i);
//   MU_CHECK_FLOAT(0.0, controller.at_controller.k_d);
//   MU_CHECK_FLOAT(0.0, controller.ct_controller.k_p);
//   MU_CHECK_FLOAT(0.0, controller.ct_controller.k_i);
//   MU_CHECK_FLOAT(0.0, controller.ct_controller.k_d);
//   MU_CHECK_FLOAT(0.0, controller.z_controller.k_p);
//   MU_CHECK_FLOAT(0.0, controller.z_controller.k_i);
//   MU_CHECK_FLOAT(0.0, controller.z_controller.k_d);
//   MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_p);
//   MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_i);
//   MU_CHECK_FLOAT(0.0, controller.yaw_controller.k_d);
//
//   MU_CHECK_FLOAT(0.0, controller.roll_limit[0]);
//   MU_CHECK_FLOAT(0.0, controller.roll_limit[1]);
//   MU_CHECK_FLOAT(0.0, controller.pitch_limit[0]);
//   MU_CHECK_FLOAT(0.0, controller.pitch_limit[1]);
//   MU_CHECK_FLOAT(0.0, controller.hover_throttle);
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//   MU_CHECK_FLOAT(0.0, controller.outputs(2));
//   MU_CHECK_FLOAT(0.0, controller.outputs(3));
//
//   return 0;
// }

// int test_wp_ctrl_configure() {
//   wp_ctrl_t controller;
//
//   wp_ctrl_configure(controller, TEST_WP_CTRL_CONFIG);
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
//   controller.configure(TEST_WP_CTRL_CONFIG);
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

/*****************************************************************************
 *                              LANDING ZONE
 ****************************************************************************/

#define TEST_LZ_CONFIG "test_data/mav/lz.yaml"

int test_lz_detector_detector_contrustor() {
  {
    const std::vector<int> tag_ids{1, 2};
    const std::vector<double> tag_sizes{0.1, 0.2};
    lz_detector_t lz_detector{tag_ids, tag_sizes};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }
  {
    lz_detector_t lz_detector{TEST_LZ_CONFIG};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }

  return 0;
}

int test_lz_detector_detector_configure() {
  {
    const std::vector<int> tag_ids{1, 2};
    const std::vector<double> tag_sizes{0.1, 0.2};
    lz_detector_t lz_detector{tag_ids, tag_sizes};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }
  {
    lz_detector_t lz_detector{TEST_LZ_CONFIG};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }

  return 0;
}

int test_lz_detector_detect() {
  const std::vector<int> tag_ids{1, 2};
  const std::vector<double> tag_sizes{0.1, 0.2};
  const lz_detector_t lz_detector{tag_ids, tag_sizes};

  cv::Mat image;
  const double fx = pinhole_focal_length(image.cols, 90.0);
  const double fy = pinhole_focal_length(image.rows, 90.0);
  const double cx = image.cols / 2.0;
  const double cy = image.rows / 2.0;
  const pinhole_t pinhole{fx, fy, cx, cy};
  proto::mat4_t T_CZ;
  // lz_detector_detect(lz, image, pinhole, T_CZ);

  return 0;
}

/*****************************************************************************
 *                               MISSION
 ****************************************************************************/

#define TEST_GPS_CONFIG "test_configs/missions/mission_gps.yaml"
#define TEST_LOCAL_CONFIG "test_configs/missions/mission_local.yaml"
#define WAYPOINTS_FILE "/tmp/waypoints.dat"
#define STATE_FILE "/tmp/state.dat"

int test_wp_mission_constructor() {
  wp_mission_t mission;

  MU_CHECK(mission.configured == false);
  MU_CHECK(mission.check_waypoints);
  MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);
  MU_CHECK_FLOAT(0.5, mission.desired_velocity);

  return 0;
}

int test_wp_mission_configure_GPS() {
  wp_mission_t mission;

  int retval = wp_mission_configure(mission, TEST_GPS_CONFIG);
  wp_mission_set_gps_homepoint(mission, 43.474024, -80.540287);

  MU_CHECK(retval == 0);

  MU_CHECK(mission.check_waypoints);
  MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);

  MU_CHECK_FLOAT(0.5, mission.desired_velocity);
  MU_CHECK(4 == (size_t) mission.local_waypoints.size());

  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](0), 0.001);
  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[0](2), 0.001);

  MU_CHECK_NEAR(6.01125, mission.local_waypoints[1](0), 0.001);
  MU_CHECK_NEAR(-10.1787, mission.local_waypoints[1](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[1](2), 0.001);

  MU_CHECK_NEAR(-7.45841, mission.local_waypoints[2](0), 0.001);
  MU_CHECK_NEAR(-17.2876, mission.local_waypoints[2](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[2](2), 0.001);

  MU_CHECK_NEAR(-10.5754, mission.local_waypoints[3](0), 0.001);
  MU_CHECK_NEAR(-9.20928, mission.local_waypoints[3](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[3](2), 0.001);

  return 0;
}

// int test_wp_mission_configure_LOCAL() {
//   Mission mission;
//
//   int retval = mission.configure(TEST_LOCAL_CONFIG);
//   MU_CHECK_EQ(0, retval);
//
//   MU_CHECK(mission.check_waypoints);
//   MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);
//
//   MU_CHECK_FLOAT(0.5, mission.desired_velocity);
//   MU_CHECK_EQ(4, (size_t) mission.local_waypoints.size());
//
//   MU_CHECK_NEAR(0.0, mission.local_waypoints[0](0), 0.001);
//   MU_CHECK_NEAR(0.0, mission.local_waypoints[0](1), 0.001);
//   MU_CHECK_NEAR(10.0, mission.local_waypoints[0](2), 0.001);
//
//   MU_CHECK_NEAR(5.0, mission.local_waypoints[1](0), 0.001);
//   MU_CHECK_NEAR(0.0, mission.local_waypoints[1](1), 0.001);
//   MU_CHECK_NEAR(10.0, mission.local_waypoints[1](2), 0.001);
//
//   MU_CHECK_NEAR(5.0, mission.local_waypoints[2](0), 0.001);
//   MU_CHECK_NEAR(5.0, mission.local_waypoints[2](1), 0.001);
//   MU_CHECK_NEAR(10.0, mission.local_waypoints[2](2), 0.001);
//
//   MU_CHECK_NEAR(0.0, mission.local_waypoints[3](0), 0.001);
//   MU_CHECK_NEAR(5.0, mission.local_waypoints[3](1), 0.001);
//   MU_CHECK_NEAR(10.0, mission.local_waypoints[3](2), 0.001);
//
//   return 0;
// }
//
// int test_wp_mission_closestPoint() {
//   Mission mission;
//
//   // setup
//   vec3_t wp_start{1.0, 1.0, 1.0};
//   vec3_t wp_end{10.0, 10.0, 10.0};
//   vec3_t position{5.0, 5.0, 5.0};
//   vec3_t expected{5.0, 5.0, 5.0};
//
//   // test and assert
//   mission.wp_start = wp_start;
//   mission.wp_end = wp_end;
//   vec3_t point = mission.closestPoint(position);
//   MU_CHECK(point.isApprox(expected));
//
//   return 0;
// }
//
// int test_wp_mission_pointLineSide() {
//   Mission mission;
//
//   vec3_t wp_start{0.0, 0.0, 0.0};
//   vec3_t wp_end{5.0, 0.0, 0.0};
//
//   // test colinear
//   vec3_t position{0.0, 0.0, 0.0};
//   mission.wp_start = wp_start;
//   mission.wp_end = wp_end;
//   double s = mission.pointLineSide(position);
//   MU_CHECK_EQ(0, s);
//
//   // test left side
//   position << 0.0, 3.0, 0.0;
//   s = mission.pointLineSide(position);
//   MU_CHECK_EQ(1, s);
//
//   // test right side
//   position << 0.0, -3.0, 0.0;
//   s = mission.pointLineSide(position);
//   MU_CHECK_EQ(-1, s);
//
//   return 0;
// }
//
// int test_wp_mission_crossTrackError() {
//   Mission mission;
//
//   // setup
//   mission.wp_start = vec3_t{0.0, 0.0, 10.0};
//   mission.wp_end = vec3_t{10.0, 0.0, 10.0};
//   vec3_t position = vec3_t::Zero();
//   double e = 0.0;
//
//   // test position to the right of waypoint track
//   position << 2.0, -3.0, 1.0;
//   e = mission.crossTrackError(position);
//   MU_CHECK_FLOAT(-3.0, e);
//
//   // test position to the left of waypoint track
//   position << 2.0, 3.0, 1.0;
//   e = mission.crossTrackError(position);
//   MU_CHECK_FLOAT(3.0, e);
//
//   return 0;
// }
//
// int test_wp_mission_waypointInterpolate() {
//   Mission mission;
//
//   // setup
//   double r = 0.0;
//   mission.wp_start = vec3_t{0.0, 0.0, 10.0};
//   mission.wp_end = vec3_t{10.0, 10.0, 10.0};
//   vec3_t pos{5.0, 8.0, 0.0};
//   vec3_t exp{6.5, 6.5, 10.0};
//
//   // test and assert
//   vec3_t point = mission.waypointInterpolate(pos, r);
//   MU_CHECK(point.isApprox(exp));
//
//   return 0;
// }
//
// int test_wp_mission_waypointHeading() {
//   Mission mission;
//
//   // setup
//   mission.wp_start = vec3_t{0.0, 0.0, 0.0};
//   double heading = 0.0;
//
//   // test 45 degree
//   mission.wp_end = vec3_t{1.0, 1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(45.0), heading);
//
//   // test 90 degree
//   mission.wp_end = vec3_t{0.0, 1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(90.0), heading);
//
//   // test 135 degree
//   mission.wp_end = vec3_t{-1.0, 1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(135.0), heading);
//
//   // test 180 degree
//   mission.wp_end = vec3_t{-1.0, 0.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(180.0), heading);
//
//   // test -45 degree
//   mission.wp_end = vec3_t{1.0, -1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(-45.0), heading);
//
//   // test -90 degree
//   mission.wp_end = vec3_t{0.0, -1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(-90.0), heading);
//
//   // test -135 degree
//   mission.wp_end = vec3_t{-1.0, -1.0, 0.0};
//   heading = mission.waypointHeading();
//   MU_CHECK_FLOAT(deg2rad(-135.0), heading);
//
//   return 0;
// }
//
// int test_wp_mission_waypointReached() {
//   Mission mission;
//
//   // setup
//   mission.configured = true;
//   mission.wp_end = vec3_t{10.0, 10.0, 0.0};
//   mission.threshold_waypoint_reached = 2.0;
//
//   // test waypoint not reached
//   vec3_t pos{0.0, 0.0, 0.0};
//   int retval = mission.waypointReached(pos);
//   MU_CHECK_EQ(0, retval);
//
//   // test waypoint reached
//   pos << 9.0, 10.0, 0.0;
//   retval = mission.waypointReached(pos);
//   MU_CHECK_EQ(1, retval);
//
//   return 0;
// }
//
// int test_wp_mission_update() {
//   Mission mission;
//
//   // setup
//   vec3_t position{2.0, 3.0, 0.0};
//   mission.look_ahead_dist = 1;
//   mission.threshold_waypoint_reached = 1.0;
//   mission.configured = true;
//
//   // push waypoints
//   vec3_t wp;
//   wp << 0.0, 0.0, 0.0;
//   mission.wp_start = wp;
//   mission.local_waypoints.push_back(wp);
//
//   wp << 10.0, 10.0, 0.0;
//   mission.wp_end = wp;
//   mission.local_waypoints.push_back(wp);
//
//   wp << 15.0, 15.0, 0.0;
//   mission.local_waypoints.push_back(wp);
//
//   // record waypoints
//   std::ofstream waypoints_file(WAYPOINTS_FILE);
//   for (auto wp : mission.local_waypoints) {
//     waypoints_file << wp(0) << ", ";
//     waypoints_file << wp(1) << ", ";
//     waypoints_file << wp(2) << std::endl;
//   }
//   waypoints_file.close();
//
//   // record robot state
//   std::ofstream state_file(STATE_FILE);
//   for (int i = 0; i < 40; i++) {
//     // waypoint update
//     vec3_t waypoint;
//     int retval = mission.update(position, waypoint);
//     if (retval != 0) {
//       MU_CHECK_EQ(-2, retval);
//       break;
//     }
//
//     // record position
//     state_file << position(0) << ", ";
//     state_file << position(1) << ", ";
//     state_file << position(2) << ", ";
//
//     // record interpolated waypoint
//     state_file << waypoint(0) << ", ";
//     state_file << waypoint(1) << ", ";
//     state_file << waypoint(2) << std::endl;
//
//     // update position
//     position(0) = position(0) + 0.5;
//     position(1) = position(1) + 0.5;
//   }
//   state_file.close();
//
//   // assert
//   MU_CHECK_EQ(3, mission.local_waypoints.size());
//
//   return 0;
// }

/*****************************************************************************
 *                                 ATL
 ****************************************************************************/

#define TEST_CONFIG "test_data/mav/atl.yaml"
#define OUTPUT_FILE "/tmp/output.csv"

int tune_att_ctrl() {
  atl_t atl;
  MU_CHECK(atl_configure(atl, TEST_CONFIG) == 0);

  std::ofstream output_file{OUTPUT_FILE};
  if (output_file.good() == false) {
    LOG_ERROR("Cannot open file for writing [%s]!", OUTPUT_FILE);
    return -1;
  }

  mav_model_t model;
  output_file << "t,r,p,y" << std::endl;
  const double r = model.attitude(0);
  const double p = model.attitude(1);
  const double y = model.attitude(2);
  output_file << 0.0 << "," << r << "," << p << "," << y << std::endl;

  double t = 0.0;
  const double dt = 0.001;
  while (t <= 1.0) {
    const vec3_t r_WB = model.position;
    const mat3_t C_WB = euler321(model.attitude);
    const mat4_t T_WB = tf(C_WB, r_WB);
    t += dt;

    // Attitude control
    vec4_t att_setpoint{0.1, 0.2, 0.3, 0.0};
    const vec4_t u = att_ctrl_update(atl.att_ctrl, att_setpoint, T_WB, dt);
    mav_model_update(model, u, dt);

    // Output to file
    const double r = model.attitude(0);
    const double p = model.attitude(1);
    const double y = model.attitude(2);
    output_file << t << "," << r << "," << p << "," << y << std::endl;
  }
  OCTAVE_SCRIPT("scripts/mav/plot_att_ctrl.m " OUTPUT_FILE);

  return 0;
}

int tune_pos_ctrl() {
  atl_t atl;
  MU_CHECK(atl_configure(atl, TEST_CONFIG) == 0);

  std::ofstream output_file{OUTPUT_FILE};
  if (output_file.good() == false) {
    LOG_ERROR("Cannot open file for writing [%s]!", OUTPUT_FILE);
    return -1;
  }

  mav_model_t model;
  output_file << "t,px,py,pz,r,p,y" << std::endl;
  const double px = model.position(0);
  const double py = model.position(1);
  const double pz = model.position(2);
  const double r = model.attitude(0);
  const double p = model.attitude(1);
  const double y = model.attitude(2);
  output_file << 0.0 << "," << px << "," << py << "," << pz << ",";
  output_file << r << "," << p << "," << y << std::endl;

  double t = 0.0;
  const double dt = 0.001;
  const vec3_t pos_setpoint{10.0, 0.0, 5.0};
  atl.yaw_setpoint = 0.0;
  while (t <= 10.0) {
    const mat3_t C_WB = euler321(model.attitude);
    const vec3_t r_WB = model.position;
    const mat4_t T_WB = tf(C_WB, r_WB);
    t += dt;

    // Position control
    const vec4_t att_setpoint =
        pos_ctrl_update(atl.pos_ctrl, pos_setpoint, T_WB, atl.yaw_setpoint, dt);

    // Attitude control
    const vec4_t u = att_ctrl_update(atl.att_ctrl, att_setpoint, T_WB, dt);
    mav_model_update(model, u, dt);

    // Output to file
    const double px = model.position(0);
    const double py = model.position(1);
    const double pz = model.position(2);
    const double r = model.attitude(0);
    const double p = model.attitude(1);
    const double y = model.attitude(2);
    output_file << t << "," << px << "," << py << "," << pz << ",";
    output_file << r << "," << p << "," << y << std::endl;
  }
  OCTAVE_SCRIPT("scripts/mav/plot_pos_ctrl.m " OUTPUT_FILE);

  return 0;
}

int test_atl_constructor() {
  atl_t atl;
  MU_CHECK(atl_configure(atl, TEST_CONFIG) == 0);

  return 0;
}

int test_atl_step_hover_mode() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_att_ctrl_constructor);
  MU_ADD_TEST(test_att_ctrl_load);
  MU_ADD_TEST(test_att_ctrl_update);

  MU_ADD_TEST(test_pos_ctrl_constructor);
  MU_ADD_TEST(test_pos_ctrl_configure);
  // MU_ADD_TEST(test_pos_ctrl_update);
  // MU_ADD_TEST(test_pos_ctrl_update2);

  // MU_ADD_TEST(test_wp_ctrl_constructor);
  // MU_ADD_TEST(test_wp_ctrl_configure);
  // MU_ADD_TEST(test_pos_ctrl_update2);

  MU_ADD_TEST(test_lz_detector_detector_contrustor);
  MU_ADD_TEST(test_lz_detector_detector_configure);
  MU_ADD_TEST(test_lz_detector_detect);

  MU_ADD_TEST(test_wp_mission_constructor);
  // MU_ADD_TEST(test_wp_mission_configure_GPS);
  // MU_ADD_TEST(test_wp_mission_configure_LOCAL);
  // MU_ADD_TEST(test_wp_mission_closestPoint);
  // MU_ADD_TEST(test_wp_mission_pointLineSide);
  // MU_ADD_TEST(test_wp_mission_crossTrackError);
  // MU_ADD_TEST(test_wp_mission_waypointInterpolate);
  // MU_ADD_TEST(test_wp_mission_waypointHeading);
  // MU_ADD_TEST(test_wp_mission_waypointReached);
  // MU_ADD_TEST(test_wp_mission_update);

  MU_ADD_TEST(tune_att_ctrl);
  MU_ADD_TEST(tune_pos_ctrl);
  MU_ADD_TEST(test_atl_constructor);
  MU_ADD_TEST(test_atl_step_hover_mode);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
