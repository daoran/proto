#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

template <typename CM, typename DM>
static vec2_t camera_project(const int img_w,
                             const int img_h,
                             const mat4_t &T_WC,
                             const vec3_t &p_W,
                             const CM &cm,
                             const DM &dm) {
  const mat4_t T_CW = T_WC.inverse();
  const vec3_t p_C = tf_point(T_CW, p_W);
  vec2_t z_hat;
  if (project(img_w, img_h, cm, dm, p_C, z_hat) != 0) {
    FATAL("Failed to project point!");
  }
  return z_hat;
}

template <typename CM, typename DM>
static vec2_t camera_project(const int img_w,
                             const int img_h,
                             const mat4_t &T_WS,
                             const mat4_t &T_SC,
                             const vec3_t &p_W,
                             const CM &cm,
                             const DM &dm) {
  const mat4_t T_WC = T_WS * T_SC;
  return camera_project(img_w, img_h, T_WC, p_W, cm, dm);
}


void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const vec3s_t &y) {
  std::ofstream file{save_path};
  if (file.good() != true) {
    printf("Failed to open file for output!");
    exit(-1);
  }

  for (size_t i = 0; i < ts.size(); i++) {
    file << ts[i] << ",";
    file << y[i](0) << ",";
    file << y[i](1) << ",";
    file << y[i](2) << std::endl;
  }

  file.close();
}

void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const quats_t &y) {
  std::ofstream file{save_path};
  if (file.good() != true) {
    printf("Failed to open file for output!");
    exit(-1);
  }

  for (size_t i = 0; i < ts.size(); i++) {
    file << ts[i] << ",";
    file << y[i].w() << ",";
    file << y[i].x() << ",";
    file << y[i].y() << ",";
    file << y[i].z() << std::endl;
  }

  file.close();
}

template <typename CM, typename DM>
static int check_J_h(
    const int img_w,
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
static int check_J_cam_params(
    const int img_w,
    const int img_h,
    const mat4_t &T_WC,
    const vec3_t &p_W,
    const double *cam_params,
    const double *dist_params,
    const matx_t &J,
    const double step_size = 1e-8,
    const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{cam_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WC, p_W, cm, dm);
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
    auto z_hat = camera_project(img_w, img_h, T_WC, p_W, cm_fd, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_cam_params", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
static int check_J_dist_params(
    const int img_w,
    const int img_h,
    const mat4_t &T_WC,
    const vec3_t &p_W,
    const double *cam_params,
    const double *dist_params,
    const matx_t &J,
    const double step_size = 1e-8,
    const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{cam_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WC, p_W, cm, dm);
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
    auto z_hat = camera_project(img_w, img_h, T_WC, p_W, cm, dm_fd);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_dist_params", fdiff, J, threshold, true);
}


template <typename CM, typename DM>
static int check_ba_factor_J_cam_pose(
    const int img_w,
    const int img_h,
    const mat4_t &T_WC,
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
  const vec2_t z_hat = camera_project(img_w, img_h, T_WC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    auto T_WC_diff = tf_perturb_rot(T_WC, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WC_diff, p_W, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    auto T_WC_diff = tf_perturb_trans(T_WC, step_size, i);
    auto z_hat = camera_project(img_w, img_h, T_WC_diff, p_W, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i + 3, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_sensor_pose", fdiff, J, threshold, true);
}

template <typename CM, typename DM>
static int check_ba_factor_J_landmark(
    const int img_w,
    const int img_h,
    const mat4_t &T_WC,
    const vec3_t &p_W,
    const double *camera_params,
    const double *dist_params,
    const matx_t &J,
    const double step_size = 1e-5,
    const double threshold = 1e-5) {
  const vec2_t z{0.0, 0.0};
  const CM cm{camera_params};
  const DM dm{dist_params};
  const vec2_t z_hat = camera_project(img_w, img_h, T_WC, p_W, cm, dm);
  const vec2_t e = z - z_hat;

  // Perturb landmark
  matx_t fdiff = zeros(2, 3);
  mat3_t dr = I(3) * step_size;
  for (int i = 0; i < 3; i++) {
    auto p_W_diff = p_W + dr.col(i);
    auto z_hat = camera_project(img_w, img_h, T_WC, p_W_diff, cm, dm);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_landmark", fdiff, J, threshold, true);
}

int test_ba_factor_jacobians() {
  // Setup parameters
  // -- Camera
  const int img_w = 640;
  const int img_h = 480;
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(euler));
  const vec3_t r_WC{0.0, 0.0, 0.0};
  const mat4_t T_WC = tf(C_WC, r_WC);
  pose_t cam_pose{T_WC};
  // -- Landmark
  const vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{p_W};
  // -- Camera intrinsics
  double cam_params[4] = {600.0, 600.0, 325.0, 240.0};
  double dist_params[4] = {0.15, -0.3, 0.0001, 0.001};

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const int cam_index = 0;
  const vec2_t z{0.0, 0.0};
  ba_factor_t<pinhole_t, radtan4_t> factor{id, ts, cam_index, img_w, img_h, z};

  // Evaluate factor
  double *params[4] {
    cam_pose.data(),
    landmark.data(),
    cam_params,
    dist_params
  };
  factor.eval(params);

  // Check ba factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  printf("Check J_h\n");
  printf("--------------------------------------------------\n");
  retval = check_J_h<pinhole_t, radtan4_t>(
    img_w, img_h, cam_params, dist_params);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check camera pose jacobian
  printf("\nCheck J_sensor_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 6> J0 = factor.jacobians[0];
  retval = check_ba_factor_J_cam_pose<pinhole_t, radtan4_t>(
    img_w, img_h, T_WC, p_W, cam_params, dist_params, J0);
  MU_CHECK(retval == 0);
  // -- Check landmark jacobian
  printf("\nCheck J_sensor_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 3> J1 = factor.jacobians[1];
  retval = check_ba_factor_J_landmark<pinhole_t, radtan4_t>(
    img_w, img_h, T_WC, p_W, cam_params, dist_params, J1);
  MU_CHECK(retval == 0);
  // -- Check cam params jacobian
  printf("\nCheck J_cam_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J2 = factor.jacobians[2];
  retval = check_J_cam_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WC, p_W, cam_params, dist_params, J2);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check dist params jacobian
  printf("\nCheck J_dist_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J3 = factor.jacobians[3];
  retval = check_J_dist_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WC, p_W, cam_params, dist_params, J3);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");

  return 0;
}

template <typename CM, typename DM>
static int check_cam_factor_J_sensor_pose(
    const int img_w,
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
static int check_cam_factor_J_sensor_camera_pose(
    const int img_w,
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
static int check_cam_factor_J_landmark(
    const int img_w,
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
  pose_t sensor_pose{T_WS};

  mat4_t T_SC;
  T_SC << 0.0, -1.0, 0.0, -0.02,
          1.0, 0.0,  0.0, -0.06,
          0.0, 0.0, 1.0, 0.00,
          0.0, 0.0, 0.0, 1.0;
  pose_t imu_cam_extrinsics{T_SC};

  vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{p_W};

  double cam_params[4] = {600.0, 600.0, 325.0, 240.0};
  double dist_params[4] = {0.15, -0.3, 0.0001, 0.001};
  // clang-format on

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const int cam_index = 0;
  const vec2_t z{0.0, 0.0};
  cam_factor_t<pinhole_t, radtan4_t> factor{id, ts, cam_index, img_w, img_h, z};

  // Evaluate factor
  double *params[5] {
    sensor_pose.data(),
    imu_cam_extrinsics.data(),
    landmark.data(),
    cam_params,
    dist_params
  };
  factor.eval(params);

  // Check camera factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  printf("Check J_h\n");
  printf("--------------------------------------------------\n");
  retval = check_J_h<pinhole_t, radtan4_t>(
    img_w, img_h, cam_params, dist_params);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check sensor pose jacobian
  printf("\nCheck J_sensor_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 6> J0 = factor.jacobians[0];
  retval = check_cam_factor_J_sensor_pose<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J0);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check sensor camera pose jacobian
  printf("\nCheck J_sensor_camera_pose\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 6> J1 = factor.jacobians[1];
  retval = check_cam_factor_J_sensor_camera_pose<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J1);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check landmark jacobian
  printf("\nCheck J_landmark\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 3> J2 = factor.jacobians[2];
  retval = check_cam_factor_J_landmark<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS, T_SC, p_W, cam_params, dist_params, J2);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check cam params jacobian
  printf("\nCheck J_cam_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J3 = factor.jacobians[3];
  retval = check_J_cam_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS * T_SC, p_W, cam_params, dist_params, J3);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");
  // -- Check dist params jacobian
  printf("\nCheck J_dist_params\n");
  printf("--------------------------------------------------\n");
  const mat_t<2, 4> J4 = factor.jacobians[4];
  retval = check_J_dist_params<pinhole_t, radtan4_t>(
    img_w, img_h, T_WS * T_SC, p_W, cam_params, dist_params, J4);
  MU_CHECK(retval == 0);
  printf("--------------------------------------------------\n");

  return 0;
}

static int check_imu_factor_J_sensor_pose_i(
    imu_factor_t &factor,
    mat4_t &T_WS_i, vec_t<9> &sb_i,
    mat4_t &T_WS_j, vec_t<9> &sb_j,
    matx_t &J,
    const real_t step_size=1e-8,
    const real_t threshold=1e-4) {
  imu_factor_t imu_factor = factor;
  pose_t sensor_pose_i{T_WS_i};
  pose_t sensor_pose_j{T_WS_j};
  real_t *params[4] = {
    sensor_pose_i.data(),
    sb_i.data(),
    sensor_pose_j.data(),
    sb_j.data()
  };
  factor.eval(params);
  const auto e = factor.residuals;

  // Perturb rotation
  matx_t fdiff = zeros(15, 6);
  for (int i = 0; i < 3; i++) {
    auto T_WS_i_diff = tf_perturb_rot(T_WS_i, step_size, i);
    pose_t sensor_pose_i{T_WS_i_diff};
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i.data(),
      sensor_pose_j.data(),
      sb_j.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i, 15, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    auto T_WS_i_diff = tf_perturb_trans(T_WS_i, step_size, i);
    pose_t sensor_pose_i{T_WS_i_diff};
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i.data(),
      sensor_pose_j.data(),
      sb_j.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i + 3, 15, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_sensor_pose_i", fdiff, J, threshold, true);
}

static int check_imu_factor_J_speed_bias_i(
    imu_factor_t &factor,
    mat4_t &T_WS_i, vec_t<9> &sb_i,
    mat4_t &T_WS_j, vec_t<9> &sb_j,
    matx_t &J,
    const real_t step_size=1e-8,
    const real_t threshold=1e-4) {
  imu_factor_t imu_factor = factor;
  pose_t sensor_pose_i{T_WS_i};
  pose_t sensor_pose_j{T_WS_j};
  real_t *params[4] = {
    sensor_pose_i.data(),
    sb_i.data(),
    sensor_pose_j.data(),
    sb_j.data()
  };
  factor.eval(params);
  const auto e = factor.residuals;

  // Perturb
  matx_t fdiff = zeros(15, 9);
  for (int i = 0; i < 9; i++) {
    auto sb_i_diff = sb_i;
    sb_i_diff(i) += step_size;
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i_diff.data(),
      sensor_pose_j.data(),
      sb_j.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i, 15, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_speed_bias_i", fdiff, J, threshold, true);
}

static int check_imu_factor_J_sensor_pose_j(
    imu_factor_t &factor,
    mat4_t &T_WS_i, vec_t<9> &sb_i,
    mat4_t &T_WS_j, vec_t<9> &sb_j,
    matx_t &J,
    const real_t step_size=1e-8,
    const real_t threshold=1e-4) {
  imu_factor_t imu_factor = factor;
  pose_t sensor_pose_i{T_WS_i};
  pose_t sensor_pose_j{T_WS_j};
  real_t *params[4] = {
    sensor_pose_i.data(),
    sb_i.data(),
    sensor_pose_j.data(),
    sb_j.data()
  };
  factor.eval(params);
  const auto e = factor.residuals;

  // Perturb rotation
  matx_t fdiff = zeros(15, 6);
  for (int i = 0; i < 3; i++) {
    auto T_WS_j_diff = tf_perturb_rot(T_WS_j, step_size, i);
    pose_t sensor_pose_j{T_WS_j_diff};
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i.data(),
      sensor_pose_j.data(),
      sb_j.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i, 15, 1) = (e_prime - e) / step_size;
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    auto T_WS_j_diff = tf_perturb_trans(T_WS_j, step_size, i);
    pose_t sensor_pose_j{T_WS_j_diff};
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i.data(),
      sensor_pose_j.data(),
      sb_j.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i + 3, 15, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_sensor_pose_j", fdiff, J, threshold, true);
}

static int check_imu_factor_J_speed_bias_j(
    imu_factor_t &factor,
    mat4_t &T_WS_i, vec_t<9> &sb_i,
    mat4_t &T_WS_j, vec_t<9> &sb_j,
    matx_t &J,
    const real_t step_size=1e-8,
    const real_t threshold=1e-4) {
  imu_factor_t imu_factor = factor;
  pose_t sensor_pose_i{T_WS_i};
  pose_t sensor_pose_j{T_WS_j};
  real_t *params[4] = {
    sensor_pose_i.data(),
    sb_i.data(),
    sensor_pose_j.data(),
    sb_j.data()
  };
  factor.eval(params);
  const auto e = factor.residuals;

  // Perturb
  matx_t fdiff = zeros(15, 9);
  for (int i = 0; i < 9; i++) {
    auto sb_j_diff = sb_j;
    sb_j_diff(i) += step_size;
    real_t *params[4] = {
      sensor_pose_i.data(),
      sb_i.data(),
      sensor_pose_j.data(),
      sb_j_diff.data()
    };
    factor.eval(params);
    const auto e_prime = factor.residuals;

    fdiff.block(0, i, 15, 1) = (e_prime - e) / step_size;
  }

  return check_jacobian("J_speed_bias_i", fdiff, J, threshold, true);
}

int test_imu_factor_jacobians() {
  // Generate trajectory
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;
  for (int i = 0; i <= 10; i++) {
    timestamps.push_back(i * 1e9);
    positions.emplace_back(i, i, 0.0);
    orientations.emplace_back(1.0, 0.0, 0.0, 0.0);
  }
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  // Setup imu sim
  sim_imu_t imu;
  imu.rate = 400;
  imu.tau_a = 3600;
  imu.tau_g = 3600;
  imu.sigma_g_c = 0.00275;
  imu.sigma_a_c = 0.0250;
  imu.sigma_gw_c = 1.65e-05;
  imu.sigma_aw_c = 0.000441;
  imu.g = 9.81007;

  // Simulate IMU measurements
  std::default_random_engine rndeng;
  timestamps_t imu_ts;
  vec3s_t imu_accel;
  vec3s_t imu_gyro;
  vec3s_t pos_prop;
  vec3s_t vel_prop;
  quats_t att_prop;

  timestamp_t ts_k = 0;
  const timestamp_t ts_end = timestamps.back();
  const timestamp_t dt = (1 / imu.rate) * 1e9;

  // -- Initialize position, velocity and attidue
  auto T_WS = ctraj_get_pose(ctraj, 0.0);
  vec3_t r_WS = tf_trans(T_WS);
  mat3_t C_WS = tf_rot(T_WS);
  vec3_t v_WS = ctraj_get_velocity(ctraj, 0.0);

  // -- Simulate imu measurements
  while (ts_k <= ts_end) {
    const auto T_WS_W = ctraj_get_pose(ctraj, ts_k);
    const auto w_WS_W = ctraj_get_angular_velocity(ctraj, ts_k);
    const auto a_WS_W = ctraj_get_acceleration(ctraj, ts_k);
    vec3_t a_WS_S;
    vec3_t w_WS_S;
    sim_imu_measurement(imu,
                        rndeng,
                        ts_k,
                        T_WS_W,
                        w_WS_W,
                        a_WS_W,
                        a_WS_S,
                        w_WS_S);

    // Propagate simulated IMU measurements
    const double dt_k = ts2sec(dt);
    const double dt_k_sq = dt_k * dt_k;
    const vec3_t g{0.0, 0.0, -imu.g};
    // -- Position at time k
    const vec3_t b_a = ones(3, 1) * imu.b_a;
    const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
    r_WS += v_WS * dt_k;
    r_WS += 0.5 * g * dt_k_sq;
    r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_k_sq;
    // -- velocity at time k
    v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_k + g * dt_k;
    // -- Attitude at time k
    const vec3_t b_g = ones(3, 1) * imu.b_g;
    const vec3_t n_g = ones(3, 1) * imu.sigma_g_c;
    C_WS = C_WS * so3_exp((w_WS_S - b_g - n_g) * ts2sec(dt));

    // Reocord IMU measurments
    pos_prop.push_back(r_WS);
    vel_prop.push_back(v_WS);
    att_prop.emplace_back(quat_t{C_WS});
    imu_ts.push_back(ts_k);
    imu_accel.push_back(a_WS_S);
    imu_gyro.push_back(w_WS_S);

    ts_k += dt;
  }
  save_data("/tmp/att_prop.csv", imu_ts, att_prop);
  save_data("/tmp/pos_prop.csv", imu_ts, pos_prop);
  save_data("/tmp/imu_accel.csv", imu_ts, imu_accel);
  save_data("/tmp/imu_gyro.csv", imu_ts, imu_gyro);

  // Create imu factor
  imu_factor_t factor(0, imu_ts, imu_gyro, imu_accel);
  factor.propagate(imu_ts, imu_gyro, imu_accel);
  std::cout << "dp: " << factor.dp.transpose() << std::endl;
  std::cout << "dv: " << factor.dv.transpose() << std::endl;
  std::cout << "dq: " << factor.dq.w() << " " << factor.dq.vec().transpose() << std::endl;

  mat4_t T_WS_i = tf(orientations.front(), positions.front());
  pose_t pose_i{T_WS_i};

  const vec3_t v_WS_i = ctraj_get_velocity(ctraj, 0.0);
  vec_t<9> sb_i;
  sb_i << v_WS_i, zeros(3, 1), zeros(3, 1);

  mat4_t T_WS_j = tf(orientations.back(), positions.back());
  pose_t pose_j{T_WS_j};

  const vec3_t v_WS_j = ctraj_get_velocity(ctraj, 10.0);
  vec_t<9> sb_j;
  sb_j << v_WS_j, zeros(3, 1), zeros(3, 1);

  print_matrix("T_WS_i", T_WS_i);
  print_matrix("T_WS_j", T_WS_j);
  print_vector("sb_i", sb_i);
  print_vector("sb_j", sb_j);

  double *params[4] = {
    pose_i.data(),
    sb_i.data(),
    pose_j.data(),
    sb_j.data(),
  };
  factor.eval(params);

  check_imu_factor_J_sensor_pose_i(factor,
                                   T_WS_i, sb_i,
                                   T_WS_j, sb_j,
                                   factor.jacobians[0]);

  check_imu_factor_J_speed_bias_i(factor,
                                  T_WS_i, sb_i,
                                  T_WS_j, sb_j,
                                  factor.jacobians[1]);

  check_imu_factor_J_sensor_pose_j(factor,
                                   T_WS_i, sb_i,
                                   T_WS_j, sb_j,
                                   factor.jacobians[2]);

  check_imu_factor_J_speed_bias_j(factor,
                                  T_WS_i, sb_i,
                                  T_WS_j, sb_j,
                                  factor.jacobians[3]);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_imu_measurements.m "
                  "/tmp/pos_data.csv "
                  "/tmp/pos_prop.csv "
                  "/tmp/att_data.csv "
                  "/tmp/att_prop.csv "
                  "/tmp/imu_accel.csv "
                  "/tmp/imu_gyro.csv ");
  }

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
  MU_ADD_TEST(test_ba_factor_jacobians);
  MU_ADD_TEST(test_cam_factor_jacobians);
  MU_ADD_TEST(test_imu_factor_jacobians);

  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_add_pose);
  MU_ADD_TEST(test_graph_add_landmark);
  MU_ADD_TEST(test_graph_solve);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
