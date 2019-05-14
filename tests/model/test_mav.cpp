#include "prototype/control/carrot_ctrl.hpp"
#include "prototype/model/mav.hpp"
#include "prototype/munit.hpp"

namespace proto {

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
  gnd_file << mav.p_G(0) << ",";
  gnd_file << mav.p_G(1) << ",";
  gnd_file << mav.p_G(2) << ",";
  // -- Velocity
  gnd_file << mav.v_G(0) << ",";
  gnd_file << mav.v_G(1) << ",";
  gnd_file << mav.v_G(2) << ",";
  // -- Attitude
  gnd_file << mav.rpy_G(0) << ",";
  gnd_file << mav.rpy_G(1) << ",";
  gnd_file << mav.rpy_G(2) << std::endl;

  // Record mav body acceleration and angular velocity
  // -- Time
  mea_file << t << ",";
  // -- Body acceleration
  mea_file << mav.a_B(0) << ",";
  mea_file << mav.a_B(1) << ",";
  mea_file << mav.a_B(2) << ",";
  // -- Body angular velocity
  mea_file << mav.w_B(0) << ",";
  mea_file << mav.w_B(1) << ",";
  mea_file << mav.w_B(2) << std::endl;
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
  mav_model_set_position(mav, pos_desired);

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
  MU_ADD_TEST(test_mav_model_constructor);
  MU_ADD_TEST(test_mav_model_update);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
