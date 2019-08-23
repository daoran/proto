#include <iostream>
#include <fstream>

#include "proto/munit.hpp"
#include "proto/mav/atl.hpp"
#include "proto/model/mav.hpp"

namespace proto {

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
  MU_ADD_TEST(tune_att_ctrl);
  MU_ADD_TEST(tune_pos_ctrl);
  MU_ADD_TEST(test_atl_constructor);
  MU_ADD_TEST(test_atl_step_hover_mode);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
