#include "prototype/munit.hpp"
#include "sim/camera_motion.hpp"

namespace prototype {

int test_CameraMotion_constructor() {
  const std::vector<vec3_t> pos_points;
  const std::vector<vec3_t> att_points;
  const double time_dt = 0.01;
  const double time_end = 100.0;
  CameraMotion camera_motion(pos_points, att_points, time_dt, time_end);

  MU_CHECK_EQ(0, camera_motion.pos_points.size());
  MU_CHECK_EQ(0, camera_motion.att_points.size());
  MU_CHECK_EQ(100.0, camera_motion.time_end);
  MU_CHECK_EQ(100.0, camera_motion.time_end);

  return 0;
}

int test_CameraMotion_update() {
  const std::vector<vec3_t> pos_points{vec3_t{0.0, 0.0, 0.0},
                                     vec3_t{0.0, 3.0, 0.0},
                                     vec3_t{10.0, -10.0, 0.0},
                                     vec3_t{5.0, 0.0, 0.0}};
  const std::vector<vec3_t> att_points{vec3_t{0.0, 0.0, 0.0},
                                     vec3_t{0.0, 0.0, 0.25},
                                     vec3_t{0.0, 0.0, 0.75},
                                     vec3_t{0.0, 0.0, 1.0}};

  // Setup camera motion
  const double time_dt = 0.01;
  const double time_end = 5.0;
  CameraMotion camera_motion(pos_points, att_points, time_dt, time_end);

  // Simulate camera motion
  vec3_t pos = camera_motion.p_G;
  vec3_t vel = camera_motion.v_G;
  vec3_t acc = camera_motion.a_G;
  vec3_t rpy = camera_motion.rpy_G;
  vec3_t w_m = camera_motion.w_G;

  // std::cout << "pos: " << pos.transpose() << std::endl;
  // std::cout << "vel: " << vel.transpose() << std::endl;
  // std::cout << "acc: " << acc.transpose() << std::endl;
  // std::cout << "rpy: " << rpy.transpose() << std::endl;

  std::vector<vec3_t> position = {pos_points[0]};
  std::vector<vec3_t> attitude = {att_points[0]};

  while (camera_motion.update() == 0) {
    acc = camera_motion.a_G;
    vel = vel + (acc * camera_motion.time_dt);
    pos = pos + (vel * camera_motion.time_dt);

    w_m = camera_motion.w_G;
    rpy = rpy + (w_m * camera_motion.time_dt);

    position.push_back(pos);
    attitude.push_back(rpy);
  }

  std::cout << "gnd pos: " << camera_motion.p_G.transpose() << std::endl;
  std::cout << "est pos: " << position.back().transpose() << std::endl;

  std::cout << "gnd rpy: " << camera_motion.rpy_G.transpose() << std::endl;
  std::cout << "est rpy: " << attitude.back().transpose() << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraMotion_constructor);
  MU_ADD_TEST(test_CameraMotion_update);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
