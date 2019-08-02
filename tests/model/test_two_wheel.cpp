#include "proto/munit.hpp"
#include "proto/model/two_wheel.hpp"

namespace proto {

int test_two_wheel_constructor() {
  two_wheel_t model;

  MU_CHECK(model.p_G.isApprox(vec3_t::Zero()));
  MU_CHECK(model.rpy_G.isApprox(vec3_t::Zero()));
  MU_CHECK(model.w_B.isApprox(vec3_t::Zero()));
  MU_CHECK(model.v_B.isApprox(vec3_t::Zero()));
  MU_CHECK(model.a_B.isApprox(vec3_t::Zero()));

  return 0;
}

int test_two_wheel_update() {
  // Setup output file
  std::ofstream output_file("/tmp/twowheel.dat");
  if (output_file.good() == false) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // Write output file header
  const std::string header = "t,x,y,z,vx,vy,vz,ax,ay,az,roll,pitch,yaw";
  output_file << header << std::endl;

  // Setup model
  double t_end = 10.0;
  const double dt = 0.1;

  double wz_B = 0.0;
  const double circle_radius = 10.0;
  const double circle_velocity = 1.0;
  circle_trajectory(circle_radius, circle_velocity, &wz_B, &t_end);
  two_wheel_t model;
  model.v_B(0) = circle_velocity;
  model.w_B(2) = wz_B;

  // Record initial model state
  output_file << 0.0 << ",";
  output_file << model.p_G(0) << ",";
  output_file << model.p_G(1) << ",";
  output_file << model.p_G(2) << ",";
  output_file << model.v_G(0) << ",";
  output_file << model.v_G(1) << ",";
  output_file << model.v_G(2) << ",";
  output_file << model.a_G(0) << ",";
  output_file << model.a_G(1) << ",";
  output_file << model.a_G(2) << ",";
  output_file << model.rpy_G(0) << ",";
  output_file << model.rpy_G(1) << ",";
  output_file << model.rpy_G(2) << std::endl;

  // Simulate model motion
  for (double t = 0.0; t < t_end; t += dt) {
    // Update
    two_wheel_update(model, dt);

    // Record model state
    output_file << t << ",";
    output_file << model.p_G(0) << ",";
    output_file << model.p_G(1) << ",";
    output_file << model.p_G(2) << ",";
    output_file << model.v_G(0) << ",";
    output_file << model.v_G(1) << ",";
    output_file << model.v_G(2) << ",";
    output_file << model.a_G(0) << ",";
    output_file << model.a_G(1) << ",";
    output_file << model.a_G(2) << ",";
    output_file << model.rpy_G(0) << ",";
    output_file << model.rpy_G(1) << ",";
    output_file << model.rpy_G(2) << std::endl;
  }
  // PYTHON_SCRIPT("scripts/plot_twowheel.py /tmp/twowheel.dat")

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_two_wheel_constructor);
  MU_ADD_TEST(test_two_wheel_update);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
