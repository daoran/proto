#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

#if PRECISION == SINGLE
  real_t step = 1e-3;
  real_t threshold = 0.5;
#else
  real_t step = 1e-8;
  real_t threshold = 1e-4;
#endif

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

int test_pose() {
  pose_t pose;

  MU_CHECK(pose.ts == 0);
  MU_CHECK(fltcmp(pose.param[0], 1.0) == 0);
  MU_CHECK(fltcmp(pose.param[1], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[2], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[3], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[4], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[5], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[6], 0.0) == 0);

  pose.param[4] = 0.1;
  pose.param[5] = 0.2;
  pose.param[6] = 0.3;
  MU_CHECK(fltcmp(pose.param[4], 0.1) == 0);
  MU_CHECK(fltcmp(pose.param[5], 0.2) == 0);
  MU_CHECK(fltcmp(pose.param[6], 0.3) == 0);

  return 0;
}

int test_landmark() {
  landmark_t landmark{0, vec3_t{1.0, 2.0, 3.0}};
  MU_CHECK(landmark.id == 0);
  return 0;
}

int test_pose_factor_jacobians() {
  // Form measurement pose
  const vec3_t euler{randf(-0.5, 0.5), randf(-0.5, 0.5), randf(-0.5, 0.5)};
  const mat3_t C_WS = euler321(euler);
  const vec3_t r_WS{randf(-0.5, 0.5), randf(-0.5, 0.5), randf(-0.5, 0.5)};
  const mat4_t T_WS = tf(C_WS, r_WS);

  // Form estimation pose
  pose_t pose{T_WS};

  // Pose factor
  std::vector<param_t *> params{&pose};
  pose_factor_t factor(0, T_WS, I(6), params);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_pose", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

template <typename CAMERA>
static int check_J_h(
    const int img_w,
    const int img_h,
    const real_t *proj_params,
    const real_t *dist_params) {
  // Calculate baseline
  const vec2_t z{0.0, 0.0};
  const CAMERA cam{img_w, img_h, proj_params, dist_params};
  const vec3_t p_C{1.0, 2.0, 10.0};
  vec2_t z_hat;
  mat_t<2, 3> J_h;
  cam.project(p_C, z_hat, J_h);
  const vec2_t e = z - z_hat;

  // Perturb camera parameters
  mat_t<2, 3> fdiff = zeros(2, 3);
  for (int i = 0; i < 3; i++) {
    vec3_t p_C_diff = p_C;
    p_C_diff(i) += step;
    cam.project(p_C_diff, z_hat, J_h);
    auto e_prime = z - z_hat;

    // Forward finite difference
    fdiff.block(0, i, 2, 1) = (e_prime - e) / step;
  }

  return check_jacobian("J_h", fdiff, -1 * J_h, threshold, true);
}

int test_ba_factor_jacobians() {
  // Setup parameters
  // -- Camera
  const int cam_id = 0;
  const int img_w = 640;
  const int img_h = 480;
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(euler));
  const vec3_t r_WC{0.0, 0.0, 0.0};
  const mat4_t T_WC = tf(C_WC, r_WC);
  pose_t cam_pose{T_WC};
  // -- Camera intrinsics
  proj_param_t proj_params{0, cam_id, vec4_t{600.0, 600.0, 325.0, 240.0}};
  dist_param_t dist_params{0, cam_id, vec4_t{0.15, -0.3, 0.0001, 0.001}};
  // -- Landmark
  const vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{p_W};

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const int cam_index = 0;
  const vec2_t z{0.0, 0.0};
  std::vector<param_t *> params{
    &cam_pose,
    &landmark,
    &proj_params,
    &dist_params
  };
  ba_factor_t<pinhole_radtan4_t> factor{id, ts,
                                        cam_index,
                                        img_w, img_h,
                                        z, I(2), params};

  // Check ba factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  retval = check_J_h<pinhole_radtan4_t>(
    img_w, img_h, proj_params.data(), dist_params.data());
  MU_CHECK(retval == 0);
  // -- Check camera pose jacobian
  retval = check_jacobians(&factor, 0, "J_cam_pose", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check proj params jacobian
  retval = check_jacobians(&factor, 1, "J_proj_params", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check dist params jacobian
  retval = check_jacobians(&factor, 2, "J_dist_params", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check landmark jacobian
  retval = check_jacobians(&factor, 3, "J_landmark", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_cam_factor_jacobians() {
  // Setup parameters
  // clang-format off
  const int img_w = 754;
  const int img_h = 480;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_h, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;

  mat4_t T_WS;
  T_WS << 0.0, 0.0, 1.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  pose_t sensor_pose{T_WS};

  mat4_t T_SC;
  T_SC << 0.0, -1.0, 0.0, 0.0,
          1.0, 0.0,  0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  pose_t imucam_pose{T_SC};

  vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{p_W};

  proj_param_t proj_params{0, 0, vec4_t{fx, fy, cx, cy}};
  dist_param_t dist_params{1, 0, vec4_t{0.0, 0.0, 0.0, 0.0}};
  // clang-format on

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const int cam_index = 0;
  const vec2_t z{0.0, 0.0};
  std::vector<param_t *> params{
    &sensor_pose,
    &imucam_pose,
    &landmark,
    &proj_params,
    &dist_params
  };
  cam_factor_t<pinhole_radtan4_t> factor{id, ts, cam_index, img_w, img_h, z, I(2), params};

  // Check camera factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  retval = check_J_h<pinhole_radtan4_t>(
    img_w, img_h, proj_params.data(), dist_params.data());
  MU_CHECK(retval == 0);
  // -- Check sensor pose jacobian
  retval = check_jacobians(&factor, 0, "J_sensor_pose", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check sensor camera pose jacobian
  retval = check_jacobians(&factor, 1, "J_imucam_pose", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check landmark jacobian
  retval = check_jacobians(&factor, 2, "J_landmark", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check cam params jacobian
  retval = check_jacobians(&factor, 3, "J_proj_param", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check dist params jacobian
  retval = check_jacobians(&factor, 4, "J_proj_param", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_imu_factor_jacobians() {
  // Generate trajectory
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;
  for (int i = 0; i <= 5; i++) {
    timestamps.push_back(i * 1e8);
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
    const real_t dt_k = ts2sec(dt);
    const real_t dt_k_sq = dt_k * dt_k;
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
    C_WS = C_WS * lie::Exp((w_WS_S - b_g - n_g) * ts2sec(dt));

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

  // Create factor
  // -- Sensor pose at i
  mat4_t T_WS_i = tf(orientations.front(), positions.front());
  pose_t pose_i{0, 0, T_WS_i};
  // -- Speed and bias at i
  const vec3_t v_WS_i = ctraj_get_velocity(ctraj, 0.0);
  sb_param_t sb_i{1, 0, v_WS_i, zeros(3, 1), zeros(3, 1)};
  // -- Sensor pose at j
  mat4_t T_WS_j = tf(orientations.back(), positions.back());
  pose_t pose_j{2, timestamps.back(), T_WS_j};
  // -- Speed and bias at j
  const vec3_t v_WS_j = ctraj_get_velocity(ctraj, 10.0);
  sb_param_t sb_j{1, timestamps.back(), v_WS_j, zeros(3, 1), zeros(3, 1)};
  // -- IMU factor
  std::vector<param_t *> params{&pose_i, &sb_i, &pose_j, &sb_j};
  imu_factor_t factor(0, imu_ts, imu_accel, imu_gyro, I(15), params);
  factor.propagate(imu_ts, imu_accel, imu_gyro);

  // Check jacobians
  int retval = 0;
  // -- Check jacobian of sensor pose at i
  retval = check_jacobians(&factor, 0, "J_sensor_pose_i", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check jacobian of speed and bias at i
  retval = check_jacobians(&factor, 0, "J_sb_i", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check jacobian of sensor pose at j
  retval = check_jacobians(&factor, 0, "J_sensor_pose_j", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check jacobian of speed and bias at j
  retval = check_jacobians(&factor, 0, "J_sb_j", step, threshold);
  MU_CHECK(retval == 0);

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

int test_graph_add_proj_params() {
  graph_t graph;

  int cam_index = 0;
  vec4_t params{1.0, 2.0, 3.0, 4.0};
  graph_add_proj_params(graph, cam_index, params);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_dist_params() {
  graph_t graph;

  int cam_index = 0;
  vec4_t params{1.0, 2.0, 3.0, 4.0};
  graph_add_dist_params(graph, cam_index, params);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_pose_factor() {
  graph_t graph;

  timestamp_t ts = 0;
  mat4_t T_WS = I(4);
  const size_t pose_id = graph_add_pose(graph, ts, T_WS);
  graph_add_pose_factor(graph, pose_id, T_WS);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  MU_CHECK(graph.factors.size() == 1);
  MU_CHECK(graph.factors[0] != nullptr);

  return 0;
}

int test_graph_add_sb_params() {
  graph_t graph;

  int imu_index = 0;
  vec3_t v{1.0, 2.0, 3.0};
  vec3_t ba{4.0, 5.0, 6.0};
  vec3_t bg{7.0, 8.0, 9.0};
  graph_add_sb_params(graph, imu_index, v, ba, bg);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_ba_factor() {
  graph_t graph;

  // Camera pose
  const timestamp_t ts = 0;
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(euler));
  const vec3_t r_WC = zeros(3, 1);
  const mat4_t T_WC = tf(C_WC, r_WC);
  const auto cam_pose_id = graph_add_pose(graph, ts, T_WC);

  // Landmark
  const vec3_t p_W{10.0, 0.0, 0.0};
  const auto landmark_id = graph_add_landmark(graph, p_W);

  // Camera and distortion parameters
  const int cam_index = 0;
  const vec4_t proj_params{640, 480, 320, 240};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  const auto cam_param_id = graph_add_proj_params(graph, cam_index, proj_params);
  const auto dist_param_id = graph_add_dist_params(graph, cam_index, dist_params);

  // BA factor
  const int img_w = 640;
  const int img_h = 480;
  const pinhole_radtan4_t cm{img_w, img_h, proj_params, dist_params};

  vec2_t z;
  cm.project(tf_point(T_WC.inverse(), p_W), z);
  graph_add_ba_factor<pinhole_radtan4_t>(
    graph,
    ts,
    cam_index,
    img_w,
    img_h,
    cam_pose_id,
    landmark_id,
    cam_param_id,
    dist_param_id,
    z
  );

  MU_CHECK(graph.params.size() == 4);
  MU_CHECK(graph.params[0] != nullptr);
  MU_CHECK(graph.params[1] != nullptr);
  MU_CHECK(graph.params[2] != nullptr);
  MU_CHECK(graph.params[3] != nullptr);

  MU_CHECK(graph.factors.size() == 1);

  return 0;
}

int test_graph_add_cam_factor() {
  graph_t graph;

  // Sensor pose
  const timestamp_t ts = 0;
  const mat3_t C_WS = I(3);
  const vec3_t r_WS = zeros(3, 1);
  const mat4_t T_WS = tf(C_WS, r_WS);
  const auto sensor_pose_id = graph_add_pose(graph, ts, T_WS);

  // IMU-Camera pose
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_SC = euler321(deg2rad(euler));
  const vec3_t r_SC = zeros(3, 1);
  const mat4_t T_SC = tf(C_SC, r_SC);
  const auto imucam_pose_id = graph_add_pose(graph, ts, T_SC);

  // Landmark
  const vec3_t p_W{10.0, 0.0, 0.0};
  const auto landmark_id = graph_add_landmark(graph, p_W);

  // Camera and distortion parameters
  const int cam_index = 0;
  const vec4_t proj_params{640, 480, 320, 240};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  const auto cam_param_id = graph_add_proj_params(graph, cam_index, proj_params);
  const auto dist_param_id = graph_add_dist_params(graph, cam_index, dist_params);

  // BA factor
  const int img_w = 640;
  const int img_h = 480;
  const pinhole_radtan4_t cm{img_w, img_h, proj_params, dist_params};
  vec2_t z;
  cm.project(tf_point((T_WS * T_SC).inverse(), p_W), z);
  graph_add_cam_factor<pinhole_radtan4_t>(
    graph,
    ts,
    cam_index,
    img_w,
    img_h,
    sensor_pose_id,
    imucam_pose_id,
    landmark_id,
    cam_param_id,
    dist_param_id,
    z
  );

  MU_CHECK(graph.params.size() == 5);
  MU_CHECK(graph.params[0] != nullptr);
  MU_CHECK(graph.params[1] != nullptr);
  MU_CHECK(graph.params[2] != nullptr);
  MU_CHECK(graph.params[3] != nullptr);
  MU_CHECK(graph.params[4] != nullptr);

  MU_CHECK(graph.factors.size() == 1);

  return 0;
}

// int test_graph_add_imu_factor() {
//   vio_sim_data_t sim_data;
//   sim_circle_trajectory(4.0, sim_data);
//
//   // Create graph
//   graph_t graph;
//   size_t nb_imu_meas = 10;
//   timestamp_t t0 = sim_data.imu_ts[0];
//   timestamp_t t1 = sim_data.imu_ts[nb_imu_meas];
//
//   // -- Add sensor pose at i
//   const mat4_t T_WS_i = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
//   auto pose0_id = graph_add_pose(graph, t0, T_WS_i);
//   // -- Add speed and bias at i
//   const vec3_t v_WS_i = sim_data.imu_vel[0];
//   const vec3_t ba_i{0.0, 0.0, 0.0};
//   const vec3_t bg_i{0.0, 0.0, 0.0};
//   auto sb0_id = graph_add_sb_params(graph, t0, v_WS_i, ba_i, bg_i);
//   // -- Add sensor pose at j
//   const mat4_t T_WS_j = tf(sim_data.imu_rot[nb_imu_meas], sim_data.imu_pos[nb_imu_meas]);
//   auto pose1_id = graph_add_pose(graph, t1, T_WS_j);
//   // -- Add speed and bias at j
//   const vec3_t v_WS_j = sim_data.imu_vel[nb_imu_meas];
//   const vec3_t ba_j{0.0, 0.0, 0.0};
//   const vec3_t bg_j{0.0, 0.0, 0.0};
//   auto sb1_id = graph_add_sb_params(graph, t1, v_WS_j, ba_j, bg_j);
//   // -- Add imu factor
//   const int imu_index = 0;
//   graph_add_imu_factor(graph,
//                        imu_index,
//                        slice(sim_data.imu_ts, 0, nb_imu_meas),
//                        slice(sim_data.imu_acc, 0, nb_imu_meas),
//                        slice(sim_data.imu_gyr, 0, nb_imu_meas),
//                        pose0_id,
//                        sb0_id,
//                        pose1_id,
//                        sb1_id);
//
//   real_t *params[4] = {
//     graph.params[0]->data(),
//     graph.params[1]->data(),
//     graph.params[2]->data(),
//     graph.params[3]->data()
//   };
//   graph.factors[0]->eval(params);
//   std::cout << graph.factors[0]->residuals << std::endl;
//
//   // Asserts
//   MU_CHECK(graph.params.size() == 4);
//   MU_CHECK(graph.factors.size() == 1);
//
//   return 0;
// }

void save_features(const std::string &path, const vec3s_t &features) {
  FILE *csv = fopen(path.c_str(), "w");
  for (const auto &f : features) {
    fprintf(csv, "%f,%f,%f\n", f(0), f(1), f(2));
  }
  fflush(csv);
  fclose(csv);
}

void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const vec3s_t &positions,
                const quats_t &orientations) {
  FILE *csv = fopen(path.c_str(), "w");
  for (size_t i = 0; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const vec3_t pos = positions[i];
    const quat_t rot = orientations[i];
    fprintf(csv, "%ld,", ts);
    fprintf(csv, "%f,%f,%f,", pos(0), pos(1), pos(2));
    fprintf(csv, "%f,%f,%f,%f\n", rot.w(), rot.x(), rot.y(), rot.z());
  }
  fflush(csv);
  fclose(csv);
}

void save_imu_data(const std::string &imu_data_path,
                   const std::string &imu_poses_path,
                   const timestamps_t &imu_ts,
                   const vec3s_t &imu_accel,
                   const vec3s_t &imu_gyro,
                   const vec3s_t &imu_pos,
                   const quats_t &imu_rot) {
  {
    FILE *csv = fopen(imu_data_path.c_str(), "w");
    for (size_t i = 0; i < imu_ts.size(); i++) {
      const timestamp_t ts = imu_ts[i];
      const vec3_t acc = imu_accel[i];
      const vec3_t gyr = imu_gyro[i];
      fprintf(csv, "%ld,", ts);
      fprintf(csv, "%f,%f,%f,", acc(0), acc(1), acc(2));
      fprintf(csv, "%f,%f,%f\n", gyr(0), gyr(1), gyr(2));
    }
    fflush(csv);
    fclose(csv);
  }

  {
    FILE *csv = fopen(imu_poses_path.c_str(), "w");
    for (size_t i = 0; i < imu_ts.size(); i++) {
      const timestamp_t ts = imu_ts[i];
      const vec3_t pos = imu_pos[i];
      const quat_t rot = imu_rot[i];
      fprintf(csv, "%ld,", ts);
      fprintf(csv, "%f,%f,%f,", pos(0), pos(1), pos(2));
      fprintf(csv, "%f,%f,%f,%f\n", rot.w(), rot.x(), rot.y(), rot.z());
    }
    fflush(csv);
    fclose(csv);
  }
}

int test_graph_eval() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Create camera
  const int img_w = 640;
  const int img_h = 480;
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(img_w, lens_hfov);
  const real_t fy = pinhole_focal(img_h, lens_vfov);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const pinhole_radtan4_t cam0{img_w, img_h, proj_params, dist_params};

  // Create graph
  graph_t graph;
	bool prior_set = false;

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    graph_add_landmark(graph, feature);
  }
  // -- Add cam0 parameters
  int cam_id = 0;
  size_t proj_param_id = graph_add_proj_params(graph, cam_id, cam0.proj_params(), true);
  size_t dist_param_id = graph_add_dist_params(graph, cam_id, cam0.dist_params(), true);
  // -- Add cam0 poses and ba factors
  size_t pose_idx = 0;
	int cam_pose = 0;
  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      // Add cam0 pose
      const quat_t q_WC0 = sim_data.cam_rot[pose_idx];
      const vec3_t r_WC0 = sim_data.cam_pos[pose_idx];
      const mat4_t T_WC0 = tf(q_WC0, r_WC0);
      const size_t cam0_pose_id = graph_add_pose(graph, ts, T_WC0);
      pose_idx++;

      if (prior_set == false) {
        graph_add_pose_factor(graph, cam0_pose_id, T_WC0);
        prior_set = true;
      }

      // Add cam0 observations at ts
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_id = event.frame.feature_ids[i];
        const auto z = event.frame.keypoints[i];
        graph_add_ba_factor<pinhole_radtan4_t>(graph, ts, event.sensor_id,
                                               cam0.img_w, cam0.img_h,
                                               cam0_pose_id, feature_id,
                                               proj_param_id, dist_param_id, z);
      }

			cam_pose++;
			if (cam_pose == 2) {
				break;
			}
    }
  }

  // Evaluate graph
	vecx_t r;
	matx_t J;
  graph_eval(graph, r, J);
	mat2csv("/tmp/J.csv", J);
	mat2csv("/tmp/r.csv", r);

	// Solve Gauss-Newton system [H dx = g]: Solve for dx
	real_t lambda = 1e-3;
	matx_t H = J.transpose() * J; // Hessian approx: H = J^t J
	matx_t H_diag = (H.diagonal().asDiagonal());
	H = H + lambda * H_diag;			// R. Fletcher trust region mod
	const vecx_t g = -J.transpose() * r;
	const vecx_t dx = H.ldlt().solve(g);   // Cholesky decomp
	mat2csv("/tmp/dx.csv", dx);

	OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/J.csv");
	// OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/r.csv");

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_sim.m");
  }

  return 0;
}

int test_graph_solve() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Create camera
  const int img_w = 640;
  const int img_h = 480;
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(img_w, lens_hfov);
  const real_t fy = pinhole_focal(img_h, lens_vfov);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const pinhole_radtan4_t cam0{img_w, img_h, proj_params, dist_params};

  // Create graph
  bool prior_set = false;
  graph_t graph;

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    // graph_add_landmark(graph, feature + vec3_t{randf(-0.01, 0.01), randf(-0.01, 0.01), randf(-0.01, 0.01)});
    graph_add_landmark(graph, feature);
  }
  // -- Add cam0 parameters
  int cam_id = 0;
  size_t proj_param_id = graph_add_proj_params(graph, cam_id, cam0.proj_params());
  size_t dist_param_id = graph_add_dist_params(graph, cam_id, cam0.dist_params());
  // -- Add cam0 poses and ba factors
  size_t pose_idx = 0;
	int cam_pose = 0;
  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      // Add cam0 pose
      const quat_t q_WC0 = sim_data.cam_rot[pose_idx];
      const vec3_t r_WC0_diff{randf(-0.05, 0.05), randf(-0.05, 0.05), randf(-0.05, 0.05)};
      const vec3_t r_WC0 = sim_data.cam_pos[pose_idx] + r_WC0_diff;
      // const vec3_t r_WC0 = sim_data.cam_pos[pose_idx];
      const mat4_t T_WC0 = tf(q_WC0, r_WC0);
      const size_t cam0_pose_id = graph_add_pose(graph, ts, T_WC0);
      pose_idx++;

      if (prior_set == false) {
        graph_add_pose_factor(graph, cam0_pose_id, T_WC0);
        prior_set = true;
      }

      // Add cam0 observations at ts
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_id = event.frame.feature_ids[i];
        const auto z = event.frame.keypoints[i];
        graph_add_ba_factor<pinhole_radtan4_t>(graph, ts, event.sensor_id,
                                               cam0.img_w, cam0.img_h,
                                               cam0_pose_id, feature_id,
                                               proj_param_id, dist_param_id, z);
      }

			cam_pose++;
			if (cam_pose == 20) {
				break;
			}
    }
  }

	tiny_solver_t solver;
	solver.max_iter = 30;
	solver.lambda = 1e6;
	solver.solve(graph);
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/E.csv");

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_sim.m");
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_pose);
  MU_ADD_TEST(test_landmark);

  MU_ADD_TEST(test_pose_factor_jacobians);
  MU_ADD_TEST(test_ba_factor_jacobians);
  MU_ADD_TEST(test_cam_factor_jacobians);
  MU_ADD_TEST(test_imu_factor_jacobians);

  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_add_pose);
  MU_ADD_TEST(test_graph_add_landmark);
  MU_ADD_TEST(test_graph_add_proj_params);
  MU_ADD_TEST(test_graph_add_dist_params);
  MU_ADD_TEST(test_graph_add_sb_params);
  MU_ADD_TEST(test_graph_add_pose_factor);
  MU_ADD_TEST(test_graph_add_ba_factor);
  MU_ADD_TEST(test_graph_add_cam_factor);
  // MU_ADD_TEST(test_graph_add_imu_factor);
  // MU_ADD_TEST(test_graph_eval);
  // MU_ADD_TEST(test_graph_solve);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
