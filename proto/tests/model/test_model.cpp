#include "proto/core/core.hpp"
#include "proto/model/model.hpp"
#include "proto/munit.hpp"

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

int setup_output_files(std::ofstream &gnd_file,
                       std::ofstream &mea_file,
                       std::ofstream &est_file) {
  // Ground truth file
  const std::string gnd_file_path = "/tmp/mav_gnd.dat";
  gnd_file.open(gnd_file_path);
  if (gnd_file.good() == false) {
    LOG_ERROR("Failed to open ground truth file for recording [%s]",
              gnd_file_path.c_str());
    return -1;
  }

  // Measurement file
  const std::string mea_file_path = "/tmp/mav_mea.dat";
  mea_file.open(mea_file_path);
  if (mea_file.good() == false) {
    LOG_ERROR("Failed to open measurement file for recording [%s]",
              mea_file_path.c_str());
    return -1;
  }

  // Estimate file
  const std::string est_file_path = "/tmp/mav_est.dat";
  est_file.open(est_file_path);
  if (est_file.good() == false) {
    LOG_ERROR("Failed to open estimate file for recording [%s]",
              est_file_path.c_str());
    return -1;
  }

  // Write header
  const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  gnd_file << gnd_header << std::endl;
  const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
  mea_file << mea_header << std::endl;
  const std::string est_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  est_file << est_header << std::endl;

  return 0;
}

void record_timestep(const double t,
                     const mav_model_t &mav,
                     std::ofstream &gnd_file,
                     std::ofstream &mea_file,
                     std::ofstream &est_file) {
  // Record mav ground truth
  // -- Time
  gnd_file << t << ",";
  // -- Position
  gnd_file << mav.position(0) << ",";
  gnd_file << mav.position(1) << ",";
  gnd_file << mav.position(2) << ",";
  // -- Velocity
  gnd_file << mav.linear_velocity(0) << ",";
  gnd_file << mav.linear_velocity(1) << ",";
  gnd_file << mav.linear_velocity(2) << ",";
  // -- Attitude
  gnd_file << mav.attitude(0) << ",";
  gnd_file << mav.attitude(1) << ",";
  gnd_file << mav.attitude(2) << std::endl;

  // // Record mav body acceleration and angular velocity
  // // -- Time
  // mea_file << t << ",";
  // // -- Body acceleration
  // mea_file << mav.a_B(0) << ",";
  // mea_file << mav.a_B(1) << ",";
  // mea_file << mav.a_B(2) << ",";
  // // -- Body angular velocity
  // mea_file << mav.w_B(0) << ",";
  // mea_file << mav.w_B(1) << ",";
  // mea_file << mav.w_B(2) << std::endl;
}

int test_mav_model_constructor() {
  mav_model_t mav;
  return 0;
}

int test_mav_model_update() {
  // Setup output files
  std::ofstream gnd_file;
  std::ofstream mea_file;
  std::ofstream est_file;
  int retval = setup_output_files(gnd_file, mea_file, est_file);
  if (retval != 0) {
    LOG_ERROR("Failed to setup output files!");
    return -1;
  }

  // Setup mav model
  mav_model_t mav;

  // Record initial mav state
  record_timestep(0.0, mav, gnd_file, mea_file, est_file);

  // Set mav desired position
  const vec3_t pos_desired{1.0, 0.0, 5.0};
  mav.position = pos_desired;

  // Simulate
  const double dt = 0.01;
  // for (double t = 0.0; t <= 30.0; t += dt) {
  for (double t = 0.0; t <= 30.0; t += dt) {
    // Update mav model
    const vec4_t motor_inputs = zeros(4, 1);
    mav_model_update(mav, motor_inputs, dt);

    // Record
    record_timestep(t, mav, gnd_file, mea_file, est_file);
  }
  gnd_file.close();
  mea_file.close();
  est_file.close();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_two_wheel_constructor);
  MU_ADD_TEST(test_two_wheel_update);

  MU_ADD_TEST(test_mav_model_constructor);
  MU_ADD_TEST(test_mav_model_update);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
