#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

static int check_jacobian(const std::string &jac_name,
                          const matx_t &fdiff,
                          const matx_t &jac,
                          const double threshold,
                          const bool print = false) {
  int retval = 0;
  const matx_t delta = (fdiff - jac);
  bool failed = false;

  // Check if any of the values are beyond the threshold
  for (long i = 0; i < delta.rows(); i++) {
    for (long j = 0; j < delta.cols(); j++) {
      if (fabs(delta(i, j)) >= threshold) {
        failed = true;
      }
    }
  }

  // Print result
  if (failed) {
    retval = -1;
    if (print) {
      printf("Check [%s] failed!\n", jac_name.c_str());
    }
    const matx_t fdiff_minus_jac = fdiff - jac;
    const matx_t num_diff = fdiff;

    if (print) {
      printf("----------------------------------------\n");
    }

  } else {
    if (print) {
      printf("Check [%s] passed!\n", jac_name.c_str());
    }
    retval = 0;
  }

  return retval;
}

int check_J_point(const mat4_t &T_WC,
                  const vec3_t &p_W,
                  const mat_t<2, 3> &J_point,
                  const double step_size = 1e-5,
                  const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const vec3_t p_C = (T_WC * p_W.homogeneous()).head(3);
  const vec2_t z_hat{p_C(0) / p_C(2), p_C(1) / p_C(2)};
  const vec2_t e = z - z_hat;

  matx_t fdiff = zeros(2, 3);
  mat3_t dr = I(3) * step_size;

  // Perturb landmark
  for (int i = 0; i < 3; i++) {
    const vec3_t p_W_diff = p_W + dr.col(i);
    const vec3_t p_C_diff = (T_WC * p_W_diff.homogeneous()).head(3);
    const vec2_t z_hat{p_C_diff(0) / p_C_diff(2), p_C_diff(1) / p_C_diff(2)};
    const vec2_t e_prime = z - z_hat;
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_point", fdiff, J_point, threshold, true);
}

int check_J_cam_pose(const mat4_t &T_WC,
                     const vec3_t &p_W,
                     const mat_t<2, 6> &J_cam_pose,
                     const double step_size = 1e-5,
                     const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const vec3_t p_C = (T_WC * p_W.homogeneous()).head(3);
  const vec2_t z_hat{p_C(0) / p_C(2), p_C(1) / p_C(2)};
  const vec2_t e = z - z_hat;

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    const mat4_t T_WC_diff = tf_perturb_rot(T_WC, step_size, i);
    const vec3_t p_C_diff = (T_WC_diff * p_W.homogeneous()).head(3);
    const vec2_t z_hat{p_C_diff(0) / p_C_diff(2), p_C_diff(1) / p_C_diff(2)};

    const vec2_t e_prime = z - z_hat;
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    const mat4_t T_WC_diff = tf_perturb_trans(T_WC, step_size, i);
    const vec3_t p_C_diff = (T_WC_diff * p_W.homogeneous()).head(3);
    const vec2_t z_hat{p_C_diff(0) / p_C_diff(2), p_C_diff(1) / p_C_diff(2)};

    const vec2_t e_prime = z - z_hat;
    fdiff.block(0, i + 3, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_cam_pose", fdiff, J_cam_pose, threshold, true);
}

int test_pose() {
  // pose_t pose{0, 0, vec3_t{1.0, 2.0, 3.0}};
  // MU_CHECK(pose.id == 0);
  return 0;
}

int test_landmark() {
  landmark_t landmark{0, vec3_t{1.0, 2.0, 3.0}};
  MU_CHECK(landmark.id == 0);
  return 0;
}

int test_ba_factor() {
  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const vec2_t measurement{0.0, 0.0};
  const vec3_t landmark{1.0, 2.0, 3.0};

  const mat3_t C_WC = I(3);
  const quat_t q_WC{C_WC};
  const vec3_t r_WC = zeros(3, 1);
  ba_factor_t factor(ts, id, measurement);

  // Evaluate factor
  vec2_t residuals;
  mat_t<2, 3, row_major_t> J_point;
  mat_t<2, 6, row_major_t> J_cam_pose;
  double cam_pose[7] =
      {q_WC.w(), q_WC.x(), q_WC.y(), q_WC.z(), r_WC(0), r_WC(1), r_WC(2)};
  const double *parameters[2] = {landmark.data(), cam_pose};
  double *jacobians[2] = {J_point.data(), J_cam_pose.data()};
  factor.Evaluate(parameters, residuals.data(), jacobians);

  // Check factor jacobians
  check_J_cam_pose(tf(C_WC, r_WC), landmark, J_cam_pose);
  check_J_point(tf(C_WC, r_WC), landmark, J_point);

  return 0;
}

int test_graph() {
  graph_t graph;

  MU_CHECK(graph.variables.size() == 0);
  MU_CHECK(graph.factors.size() == 0);

  return 0;
}

int test_graph_free() {
  graph_t graph;

  timestamp_t ts = 0;
  mat4_t T_WS = I(4);
  graph_add_pose(graph, ts, T_WS);
  graph_add_pose(graph, ts, T_WS);
  graph_add_pose(graph, ts, T_WS);

  MU_CHECK(graph.variables.size() == 3);
  MU_CHECK(graph.factors.size() == 0);

  graph_free(graph);
  MU_CHECK(graph.variables.size() == 0);
  MU_CHECK(graph.factors.size() == 0);

  return 0;
}

int test_graph_add_pose() {
  graph_t graph;

  timestamp_t ts = 0;
  mat4_t T_WS = I(4);
  graph_add_pose(graph, ts, T_WS);

  MU_CHECK(graph.variables.size() == 1);
  MU_CHECK(graph.variables[0] != nullptr);
  graph_free(graph);

  return 0;
}

int test_graph_add_landmark() {
  graph_t graph;

  vec3_t landmark = zeros(3, 1);
  graph_add_landmark(graph, landmark);

  MU_CHECK(graph.variables.size() == 1);
  MU_CHECK(graph.variables[0] != nullptr);
  graph_free(graph);

  return 0;
}

int test_graph_add_ba_factor() {
  graph_t graph;

  // Setup variables
  const timestamp_t ts = 0;
  const vec2_t z{0.0, 0.0};
  const vec3_t p_W{0.0, 0.0, 0.0};
  const mat4_t T_WC = I(4);

  // Add factor
  const size_t p_id = graph_add_landmark(graph, p_W);
  const size_t pose_id = graph_add_pose(graph, ts, T_WC);
  size_t id = graph_add_ba_factor(graph, ts, z, p_id, pose_id);
  MU_CHECK(id == 0);
  MU_CHECK(graph.factors.size() == 1);
  MU_CHECK(graph.variables.size() == 2);

  // Cleanup
  graph_free(graph);

  return 0;
}

int test_graph_solve() {
  graph_t graph;

  // Add landmarks
  const vec3_t p0_W{1.0, 2.0, 3.0};
  const vec3_t p1_W{4.0, 5.0, 6.0};
  const vec3_t p2_W{7.0, 8.0, 9.0};
  const size_t p0_id = graph_add_landmark(graph, p0_W);
  const size_t p1_id = graph_add_landmark(graph, p1_W);
  const size_t p2_id = graph_add_landmark(graph, p2_W);

  // Add pose
  const timestamp_t ts = 0;
  const vec2_t z{0.0, 0.0};
  const vec3_t rpy_WC{-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const mat3_t C_WC = euler321(rpy_WC);
  const vec3_t r_WC = zeros(3, 1);
  const mat4_t T_WC = tf(C_WC, r_WC);
  const size_t pose_id = graph_add_pose(graph, ts, T_WC);

  //   pose_t *T_WS_0 = nullptr;
  //   pose_t *T_WS_1 = nullptr;
  // Add Factors
  graph_add_ba_factor(graph, ts, z, p0_id, pose_id);
  graph_add_ba_factor(graph, ts, z, p1_id, pose_id);
  graph_add_ba_factor(graph, ts, z, p2_id, pose_id);

  graph_solve(graph);
  graph_free(graph);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_pose);
  MU_ADD_TEST(test_landmark);
  MU_ADD_TEST(test_ba_factor);
  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_free);
  MU_ADD_TEST(test_graph_add_pose);
  MU_ADD_TEST(test_graph_add_landmark);
  MU_ADD_TEST(test_graph_add_ba_factor);
  MU_ADD_TEST(test_graph_solve);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
