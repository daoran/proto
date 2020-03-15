#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

template <typename CM, typename DM>
static vec2_t camera_project(const int img_w,
                             const int img_h,
                             const mat4_t &T_WS,
                             const mat4_t &T_SC,
                             const vec3_t &p_W,
                             const CM &cm,
                             const DM &dm) {
  const mat4_t T_CW = T_SC.inverse() * T_WS.inverse();
  const vec3_t p_C = tf_point(T_CW, p_W);
  vec2_t z_hat;
  if (project(img_w, img_h, cm, dm, p_C, z_hat) != 0) {
    FATAL("Failed to project point!");
  }
  return z_hat;
}

template <typename CM, typename DM>
static int check_J_h(const int img_w,
                     const int img_h,
                     const double *camera_params,
                     const double *dist_params,
                     const double step_size = 1e-8,
                     const double threshold = 1e-3) {
  // Calculate baseline
  // clang-format off
  const vec2_t z{0.0, 0.0};
  const CM cm{camera_params};
  const DM dm{dist_params};
  const vec3_t p_C{1.0, 2.0, 10.0};
  vec2_t z_hat;
  mat_t<2, 3> J_h;
  project(img_w, img_h, cm, dm, p_C, z_hat, J_h);
  const vec2_t e = z - z_hat;
  // clang-format on

  // Perturb camera parameters
  mat_t<2, 3> fdiff = zeros(2, 3);
  for (int i = 0; i < 3; i++) {
    vec3_t p_C_diff = p_C;
    p_C_diff(i) += step_size;
    project(img_w, img_h, cm, dm, p_C_diff, z_hat);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_h", fdiff, -1 * J_h, threshold, true);
}

template <typename CM, typename DM>
static int check_J_sensor_pose(const int img_w,
                               const int img_h,
                               const mat4_t &T_WS,
                               const mat4_t &T_SC,
                               const vec3_t &p_W,
                               const double *camera_params,
                               const double *dist_params,
                               const matx_t &J,
                               const double step_size = 1e-8,
                               const double threshold = 1e-3) {
  // Calculate baseline
  const vec2_t z{0.0, 0.0};
  const CM cm{camera_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    auto T_WS_diff = tf_perturb_rot(T_WS, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WS_diff, T_SC, p_W, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    auto T_WS_diff = tf_perturb_trans(T_WS, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WS_diff, T_SC, p_W, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i + 3, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_sensor_pose", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
static int check_J_sensor_camera_pose(const int img_w,
                                      const int img_h,
                                      const mat4_t &T_WS,
                                      const mat4_t &T_SC,
                                      const vec3_t &p_W,
                                      const double *camera_params,
                                      const double *dist_params,
                                      const matx_t &J,
                                      const double step_size = 1e-8,
                                      const double threshold = 1e-5) {
  // Calculate baseline
  const vec2_t z{0.0, 0.0};
  const CM cm{camera_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    auto T_SC_diff = tf_perturb_rot(T_SC, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WS, T_SC_diff, p_W, cm, dm);
    auto e_prime = z - z_hat;
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    auto T_SC_diff = tf_perturb_trans(T_SC, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WS, T_SC_diff, p_W, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i + 3, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_sensor_camera_pose", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
int check_J_landmark(const int img_w,
                     const int img_h,
                     const mat4_t &T_WS,
                     const mat4_t &T_SC,
                     const vec3_t &p_W,
                     const double *camera_params,
                     const double *dist_params,
                     const matx_t &J,
                     const double step_size = 1e-5,
                     const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{camera_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb landmark
  matx_t fdiff = zeros(2, 3);
  mat3_t dr = I(3) * step_size;
  for (int i = 0; i < 3; i++) {
    auto p_W_diff = p_W + dr.col(i);
    auto z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W_diff, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_landmark", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
int check_J_cam_params(const int img_w,
                       const int img_h,
                       const mat4_t &T_WS,
                       const mat4_t &T_SC,
                       const vec3_t &p_W,
                       const double *cam_params,
                       const double *dist_params,
                       const matx_t &J,
                       const double step_size = 1e-8,
                       const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{cam_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb camera parameters
  matx_t fdiff = zeros(2, 4);
  for (int i = 0; i < 4; i++) {
    double params_fd[4] = {
      cam_params[0],
      cam_params[1],
      cam_params[2],
      cam_params[3]
    };
    params_fd[i] += step_size;

    const CM cm_fd{params_fd};
    auto z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm_fd, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_cam_params", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
int check_J_dist_params(const int img_w,
                       const int img_h,
                       const mat4_t &T_WS,
                       const mat4_t &T_SC,
                       const vec3_t &p_W,
                       const double *cam_params,
                       const double *dist_params,
                       const matx_t &J,
                       const double step_size = 1e-8,
                       const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{cam_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb camera parameters
  matx_t fdiff = zeros(2, 4);
  for (int i = 0; i < 4; i++) {
    double params_fd[4] = {
      dist_params[0],
      dist_params[1],
      dist_params[2],
      dist_params[3]
    };
    params_fd[i] += step_size;

    const DM dm_fd{params_fd};
    auto z_hat = camera_project(img_w, img_h, T_WS, T_SC, p_W, cm, dm_fd);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_dist_params", fdiff, J, threshold, true);
}

int test_cam_factor_jacobians() {
  // Setup parameters
  // clang-format off
  const int img_w = 640;
  const int img_h = 480;

  mat4_t T_WS;
  T_WS << 0.0, 0.0, 1.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

  mat4_t T_SC;
  T_SC << 0.0, -1.0, 0.0, -0.02,
          1.0, 0.0,  0.0, -0.06,
          0.0, 0.0, 1.0, 0.00,
          0.0, 0.0, 0.0, 1.0;

  vec3_t p_W{10.0, 0.0, 0.0};
  double cam_params[4] = {600.0, 600.0, 325.0, 240.0};
  double dist_params[4] = {0.15, -0.3, 0.0001, 0.001};
  // clang-format on

  // Create camera factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const int cam_index = 0;
  const vec2_t z{0.0, 0.0};
  cam_factor_t<pinhole_t, radtan4_t> factor{ts, id, cam_index, img_w, img_h, z};

  // Evaluate camera factor
  double *params[5] {
    pose_t{T_WS}.data(),
    pose_t{T_SC}.data(),
    landmark_t{p_W}.data(),
    cam_params,
    dist_params
  };
  factor.eval(params);

  // Check camera factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  printf("Check J_h\n");
  printf("--------------------------------------------------\n");
  check_J_h<pinhole_t, radtan4_t>(img_w, img_h, cam_params, dist_params);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check sensor pose jacobian
  printf("\nCheck J_sensor_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 6> J0 = factor.jacobians[0];
  retval = check_J_sensor_pose<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J0);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check sensor camera pose jacobian
  printf("\nCheck J_sensor_camera_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 6> J1 = factor.jacobians[1];
  retval = check_J_sensor_camera_pose<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J1);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check landmark jacobian
  printf("\nCheck J_landmark\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 3> J2 = factor.jacobians[2];
  retval = check_J_landmark<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J2);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check cam params jacobian
  printf("\nCheck J_cam_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J3 = factor.jacobians[3];
  retval = check_J_cam_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J3);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check dist params jacobian
  printf("\nCheck J_dist_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J4 = factor.jacobians[4];
  retval = check_J_dist_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J4);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");

  return 0;
}

int test_graph() {
  graph_t graph;

  MU_CHECK(graph.params.size() == 0);
  MU_CHECK(graph.params.size() == 0);
  MU_CHECK(graph.factors.size() == 0);

  return 0;
}

int test_graph_add_pose() {
  graph_t graph;

  timestamp_t ts = 0;
  mat4_t T_WS = I(4);
  graph_add_pose(graph, ts, T_WS);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_landmark() {
  graph_t graph;

  vec3_t landmark = zeros(3, 1);
  graph_add_landmark(graph, landmark);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

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

  // Add Factors
  // graph_add_factor(graph, ts, z, p0_id, pose_id);
  // graph_add_factor(graph, ts, z, p1_id, pose_id);
  // graph_add_factor(graph, ts, z, p2_id, pose_id);

  // graph_solve(graph);
  // graph_free(graph);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_cam_factor_jacobians);

  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_add_pose);
  MU_ADD_TEST(test_graph_add_landmark);
  MU_ADD_TEST(test_graph_solve);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
