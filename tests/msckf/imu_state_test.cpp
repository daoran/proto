#include "prototype/munit.hpp"
#include "msckf/imu_state.hpp"

namespace prototype {

int test_IMUState_constructor() {
  IMUState imu_state;

  MU_CHECK_EQ(15, imu_state.size);

  MU_CHECK(vec4_t(0.0, 0.0, 0.0, 1.0).isApprox(imu_state.q_IG));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.b_g));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.v_G));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.b_a));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.p_G));

  MU_CHECK(vec3_t(0.0, 0.0, -9.81).isApprox(imu_state.g_G));

  // MU_CHECK(zeros(1, 1).isApprox(imu_state.P));
  // MU_CHECK(zeros(1, 1).isApprox(imu_state.Q));
  // MU_CHECK(zeros(1, 1).isApprox(imu_state.Phi));

  return 0;
}

int test_IMUState_F() {
  IMUState imu_state;

  const vec3_t w_hat = vec3_t{1.0, 2.0, 3.0};
  const vec4_t q_hat = vec4_t{0.0, 0.0, 0.0, 1.0};
  const vec3_t a_hat = vec3_t{1.0, 2.0, 3.0};

  const matx_t F = imu_state.F(w_hat, q_hat, a_hat);
  std::cout << F << std::endl;

  return 0;
}

int test_IMUState_G() {
  IMUState imu_state;

  const vec4_t q_hat = vec4_t{0.0, 0.0, 0.0, 1.0};
  const matx_t G = imu_state.G(q_hat);
  std::cout << G << std::endl;

  return 0;
}

int test_IMUState_update() {
  IMUState imu_state;

  const vec3_t a_m{0.0, 0.0, 0.0};
  const vec3_t w_m{0.0, 0.0, 0.0};
  const double dt = 0.1;

  imu_state.update(a_m, w_m, dt);
  MU_CHECK(imu_state.P.maxCoeff() < 1.0);

  return 0;
}

int test_IMUState_correct() {
  IMUState imu_state;

  const vec3_t dtheta_IG{0.0, 0.0, 0.0};
  const vec3_t db_g{0.0, 0.0, 0.0};
  const vec3_t dv_G{0.0, 0.0, 0.0};
  const vec3_t db_a{0.0, 0.0, 0.0};
  const vec3_t dp_G{0.0, 0.0, 0.0};

  vecx_t dx;
  dx.resize(15, 1);
  dx.block(0, 0, 3, 1) = dtheta_IG;
  dx.block(3, 0, 3, 1) = db_g;
  dx.block(6, 0, 3, 1) = dv_G;
  dx.block(9, 0, 3, 1) = db_a;
  dx.block(12, 0, 3, 1) = dp_G;

  imu_state.correct(dx);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_IMUState_constructor);
  MU_ADD_TEST(test_IMUState_F);
  MU_ADD_TEST(test_IMUState_G);
  MU_ADD_TEST(test_IMUState_update);
  MU_ADD_TEST(test_IMUState_correct);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
