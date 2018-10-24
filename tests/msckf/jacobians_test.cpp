#include "prototype/munit.hpp"
#include "msckf/jacobians.hpp"
#include "gimbal/gimbal_model.hpp"

namespace prototype {

int test_p_c2_f_jacobian_wrt_theta1() {
  for (int i = 0; i < 1000; i++) {
    // Feature point observed from camera1 (i.e. static camera)
    const Vec3 p_c1_f{randf(-10.0, 10.0),
                      randf(-10.0, 10.0),
                      randf(1.0, 10.0)};

    // Static to dynamic camera transform parameters
    // -- Static camera to base frame transform params
    Vec6 tau_s;  // (x, y, z, roll, pitch, yaw)
    tau_s << -0.0419, -0.0908, 0.0843, 0.0189, -0.0317, -0.0368;
    // -- End-effector to dynamic camera frame transform params
    Vec6 tau_d;  // (x, y, z, roll, pitch, yaw)
    tau_d << 0.0032, 0.0007, -0.0302, 1.5800, 0.0123, -1.5695;
    // -- Gimbal joint 1
    const double theta1_offset = -M_PI / 2.0;
    const double Lambda1 = randf(-0.5, 0.5);
    const Vec3 w1{1.5731, -0.0012, 0.0404};
    // -- Gimbal joint 2
    const Vec3 w2{0.0, 0.0, 0.0};
    const double Lambda2 = randf(-0.5, 0.5);

    // Setup gimbal model
    GimbalModel gimbal_model{tau_s, tau_d, Lambda1, w1, Lambda2, w2, theta1_offset};

    // Transform feature point from static camera to dynamic camera
    const Vec3 p_c2_f_1 = (gimbal_model.T_ds() * p_c1_f.homogeneous()).head(3);
    // -- Perturb first gimbal joint angle and tranform again
    gimbal_model.Lambda1 += 1e-4;
    const Vec3 p_c2_f_2 = (gimbal_model.T_ds() * p_c1_f.homogeneous()).head(3);
    // -- Calculate forward finite difference
    const Vec3 finite_diff = (p_c2_f_2 - p_c2_f_1) / 1e-4;

    // Calculate jacobian using function generated using Matlab
    const Vec3 p_c2_f_jacobian = p_c2_f_jacobian_wrt_theta1(
      p_c1_f,
      tau_s,
      tau_d,
      Lambda1,
      w1,
      Lambda2,
      w2,
      theta1_offset
    );
    const Vec3 diff = p_c2_f_jacobian - finite_diff;

    // Assert jacobian is same as performing forward difference
    MU_CHECK(diff(0) < 1e-3);
    MU_CHECK(diff(1) < 1e-3);
    MU_CHECK(diff(2) < 1e-3);
    MU_CHECK(diff(3) < 1e-3);
    // std::cout << Lambda1 << std::endl;
    // std::cout << Lambda2 << std::endl;
    // std::cout << diff.transpose() << std::endl;
    // std::cout << std::endl;
  }

  return 0;
}

int test_p_c2_f_jacobian_wrt_theta2() {
  for (int i = 0; i < 1000; i++) {
    // Feature point observed from camera1 (i.e. static camera)
    const Vec3 p_c1_f{randf(-10.0, 10.0),
                      randf(-10.0, 10.0),
                      randf(1.0, 10.0)};

    // Static to dynamic camera transform parameters
    // -- Static camera to base frame transform params
    Vec6 tau_s;  // (x, y, z, roll, pitch, yaw)
    tau_s << -0.0419, -0.0908, 0.0843, 0.0189, -0.0317, -0.0368;
    // -- End-effector to dynamic camera frame transform params
    Vec6 tau_d;  // (x, y, z, roll, pitch, yaw)
    tau_d << 0.0032, 0.0007, -0.0302, 1.5800, 0.0123, -1.5695;
    // -- Gimbal joint 1
    const double theta1_offset = -M_PI / 2.0;
    const double Lambda1 = randf(-0.5, 0.5);
    const Vec3 w1{1.5731, -0.0012, 0.0404};
    // -- Gimbal joint 2
    const double Lambda2 = randf(-0.5, 0.5);
    const Vec3 w2{0.0, 0.0, 0.0};

    // Setup gimbal model
    GimbalModel gimbal_model{tau_s, tau_d, Lambda1, w1, Lambda2, w2, theta1_offset};

    // Transform feature point from static camera to dynamic camera
    const Vec3 p_c2_f_1 = (gimbal_model.T_ds() * p_c1_f.homogeneous()).head(3);
    // -- Perturb second gimbal joint angle and tranform again
    gimbal_model.Lambda2 += 1e-4;
    const Vec3 p_c2_f_2 = (gimbal_model.T_ds() * p_c1_f.homogeneous()).head(3);
    // -- Calculate forward finite difference
    const Vec3 finite_diff = (p_c2_f_2 - p_c2_f_1) / 1e-4;

    // Calculate jacobian using function generated using Matlab
    const Vec3 p_c2_f_jacobian = p_c2_f_jacobian_wrt_theta2(
      p_c1_f,
      tau_s,
      tau_d,
      Lambda1,
      w1,
      Lambda2,
      w2,
      theta1_offset
    );

    // Assert jacobian is same as performing forward difference
    const Vec3 diff = p_c2_f_jacobian - finite_diff;
    MU_CHECK(diff(0) < 1e-3);
    MU_CHECK(diff(1) < 1e-3);
    MU_CHECK(diff(2) < 1e-3);
    MU_CHECK(diff(3) < 1e-3);
    // std::cout << Lambda1 << std::endl;
    // std::cout << Lambda2 << std::endl;
    // std::cout << diff.transpose() << std::endl;
    // std::cout << std::endl;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_p_c2_f_jacobian_wrt_theta1);
  MU_ADD_TEST(test_p_c2_f_jacobian_wrt_theta2);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
