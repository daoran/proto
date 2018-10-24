#include "prototype/munit.hpp"
#include "prototype/control/carrot_ctrl.hpp"

namespace prototype {

int test_carrot_ctrl_constructor() {
  struct carrot_ctrl cc;

  MU_CHECK(cc.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(cc.wp_end.isApprox(Vec3::Zero()));
  MU_CHECK_EQ(0, cc.wp_index);
  MU_CHECK_FLOAT(0.0, cc.look_ahead_dist);

  return 0;
}

int test_carrot_ctrl_configure() {
  carrot_ctrl cc;

  std::vector<Vec3> waypoints;
  waypoints.emplace_back(0.0, 0.0, 0.0);
  waypoints.emplace_back(1.0, 1.0, 0.0);
  waypoints.emplace_back(2.0, 2.0, 0.0);
  waypoints.emplace_back(3.0, 3.0, 0.0);
  carrot_ctrl_configure(cc, waypoints, 0.1);

  MU_CHECK(cc.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(cc.wp_end.isApprox(Vec3{1.0, 1.0, 0.0}));
  MU_CHECK_EQ(1, cc.wp_index);
  MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);

  return 0;
}

int test_carrot_ctrl_closest_point() {
  struct carrot_ctrl cc;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 1.0, 0.0);
  wps.emplace_back(2.0, 2.0, 0.0);
  wps.emplace_back(3.0, 3.0, 0.0);
  carrot_ctrl_configure(cc, wps, 0.1);

  MU_CHECK(cc.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(cc.wp_end.isApprox(Vec3{1.0, 1.0, 0.0}));
  MU_CHECK_EQ(1, cc.wp_index);
  MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, -1.0, 0.0};
  Vec3 res0;
  int s0 = carrot_ctrl_closest_point(cc, pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{-1.0, -1.0, 0.0}));
  MU_CHECK_EQ(-1, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.5, 0.0};
  Vec3 res1;
  int s1 = carrot_ctrl_closest_point(cc, pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.5, 0.5, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 1.5, 0.0};
  Vec3 res2;
  int s2 = carrot_ctrl_closest_point(cc, pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.5, 1.5, 0.0}));
  MU_CHECK_EQ(1, s2);

  return 0;
}

int test_carrot_ctrl_carrot_point() {
  carrot_ctrl cc;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 0.0, 0.0);
  wps.emplace_back(2.0, 0.0, 0.0);
  wps.emplace_back(3.0, 0.0, 0.0);
  carrot_ctrl_configure(cc, wps, 0.1);

  MU_CHECK(cc.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(cc.wp_end.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, cc.wp_index);
  MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, 0.0, 0.0};
  Vec3 res0;
  int s0 = carrot_ctrl_carrot_point(cc, pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{0.0, 0.0, 0.0}));
  MU_CHECK_EQ(-1, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.0, 0.0};
  Vec3 res1;
  int s1 = carrot_ctrl_carrot_point(cc, pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.6, 0.0, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 0.0, 0.0};
  Vec3 res2;
  int s2 = carrot_ctrl_carrot_point(cc, pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, s2);

  return 0;
}

int test_carrot_ctrl_update() {
  struct carrot_ctrl cc;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 0.0, 0.0);
  wps.emplace_back(2.0, 0.0, 0.0);
  wps.emplace_back(3.0, 0.0, 0.0);
  carrot_ctrl_configure(cc, wps, 0.1);

  MU_CHECK(cc.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(cc.wp_end.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, cc.wp_index);
  MU_CHECK_FLOAT(0.1, cc.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, 0.0, 0.0};
  Vec3 res0;
  int s0 = carrot_ctrl_update(cc, pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{0.0, 0.0, 0.0}));
  MU_CHECK_EQ(0, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.0, 0.0};
  Vec3 res1;
  int s1 = carrot_ctrl_update(cc, pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.6, 0.0, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 0.0, 0.0};
  Vec3 res2;
  int s2 = carrot_ctrl_update(cc, pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(0, s2);
  MU_CHECK_EQ(2, cc.wp_index);
  MU_CHECK(cc.wp_start.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK(cc.wp_end.isApprox(Vec3{2.0, 0.0, 0.0}));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_carrot_ctrl_constructor);
  MU_ADD_TEST(test_carrot_ctrl_configure);
  MU_ADD_TEST(test_carrot_ctrl_closest_point);
  MU_ADD_TEST(test_carrot_ctrl_carrot_point);
  MU_ADD_TEST(test_carrot_ctrl_update);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
