#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

#define TEST_BA_DATA "./test_data/estimation/ba_data"

#if PRECISION == 1
  real_t step = 1e-5;
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
  mat4_t T = I(4);
  pose_t pose{0, 1, T};

  MU_CHECK(pose.type == "pose_t");
  MU_CHECK(pose.id == 0);
  MU_CHECK(pose.ts == 1);
  MU_CHECK(pose.local_size == 6);
  MU_CHECK(pose.global_size == 7);

  MU_CHECK(fltcmp(pose.param[0], 1.0) == 0);
  MU_CHECK(fltcmp(pose.param[1], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[2], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[3], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[4], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[5], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[6], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[4], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[5], 0.0) == 0);
  MU_CHECK(fltcmp(pose.param[6], 0.0) == 0);

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
  pose_t pose{0, 0, T_WS};

  // Pose factor
  std::vector<param_t *> params{&pose};
  pose_factor_t factor(0, T_WS, I(6), params);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_pose", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

template <typename T>
static int check_J_h(const int resolution[2], const vecx_t &cam_params) {
  // Calculate baseline
  const vec2_t z{0.0, 0.0};
  const T cam{resolution, cam_params};
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
  const int cam_index = 0;
  const int resolution[2] = {640, 480};
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(euler));
  const vec3_t r_WC{0.0, 0.0, 0.0};
  const mat4_t T_WC = tf(C_WC, r_WC);
  pose_t cam_pose{0, 0, T_WC};
  // -- Camera intrinsics
  vec4_t proj_params{600.0, 600.0, 325.0, 240.0};
  vec4_t dist_params{0.15, -0.3, 0.0001, 0.001};
  camera_params_t cam_params{1, cam_index, resolution, proj_params, dist_params};
  // -- Landmark
  const vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{2, p_W};

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const vec2_t z{0.0, 0.0};
  std::vector<param_t *> params{
    &cam_pose,
    &landmark,
    &cam_params,
  };
  ba_factor_t<pinhole_radtan4_t> factor{id, ts, z, I(2), params};

  // Check ba factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  retval = check_J_h<pinhole_radtan4_t>(resolution, cam_params.param);
  MU_CHECK(retval == 0);
  // -- Check camera pose jacobian
  retval = check_jacobians(&factor, 0, "J_cam_pose", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check landmark jacobian
  retval = check_jacobians(&factor, 1, "J_landmark", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check camera params jacobian
  retval = check_jacobians(&factor, 2, "J_cam_params", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_cam_factor_jacobians() {
  // Setup parameters
  // clang-format off
  int cam_index = 0;
  const int resolution[2] = {754, 480};
  const real_t fx = pinhole_focal(resolution[0], 90.0);
  const real_t fy = pinhole_focal(resolution[1], 90.0);
  const real_t cx = resolution[0] / 2.0;
  const real_t cy = resolution[1] / 2.0;

  mat4_t T_WS;
  T_WS << 0.0, 0.0, 1.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  pose_t sensor_pose{0, 0, T_WS};

  mat4_t T_SC;
  T_SC << 0.0, -1.0, 0.0, 0.0,
          1.0, 0.0,  0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  pose_t imucam_pose{0, 0, T_SC};

  vec3_t p_W{10.0, 0.0, 0.0};
  landmark_t landmark{0, p_W};

  vec4_t proj_params{vec4_t{fx, fy, cx, cy}};
  vec4_t dist_params{vec4_t{0.0, 0.0, 0.0, 0.0}};
  camera_params_t cam_params{1, cam_index, resolution, proj_params, dist_params};
  // clang-format on

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const vec2_t z{0.0, 0.0};
  std::vector<param_t *> params{
    &sensor_pose,
    &imucam_pose,
    &landmark,
    &cam_params,
  };
  cam_factor_t<pinhole_radtan4_t> factor{id, ts, z, I(2), params};

  // Check camera factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  retval = check_J_h<pinhole_radtan4_t>(resolution, cam_params.param);
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
  retval = check_jacobians(&factor, 3, "J_cam_params", step, threshold);
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
  sb_params_t sb_i{1, 0, v_WS_i, zeros(3, 1), zeros(3, 1)};
  // -- Sensor pose at j
  mat4_t T_WS_j = tf(orientations.back(), positions.back());
  pose_t pose_j{2, timestamps.back(), T_WS_j};
  // -- Speed and bias at j
  const vec3_t v_WS_j = ctraj_get_velocity(ctraj, 10.0);
  sb_params_t sb_j{1, timestamps.back(), v_WS_j, zeros(3, 1), zeros(3, 1)};
  // -- IMU factor
  std::vector<param_t *> params{&pose_i, &sb_i, &pose_j, &sb_j};
  imu_factor_t factor(0, imu_ts, imu_accel, imu_gyro, I(15), params);

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

int test_imu_propagate() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
	sim_data.save("/tmp/sim_data");

  vec_t<7> pose_i;
  vec_t<9> sb_i;
  auto q_WS = sim_data.imu_rot[0];
  auto r_WS = sim_data.imu_pos[0];
  pose_i << q_WS.w(), q_WS.x(), q_WS.y(), q_WS.z(), r_WS;
  sb_i << sim_data.imu_vel[0], zeros(6, 1);

  imu_data_t imu_data;
  const vec3_t g{0.0, 0.0, -9.81};

  for (size_t k = 0; k < sim_data.imu_ts.size(); k++) {
    vec_t<7> pose_j = pose_i;
    vec_t<9> sb_j = sb_i;
    imu_data.add(sim_data.imu_ts[k], sim_data.imu_acc[k], sim_data.imu_gyr[k]);

    if (imu_data.size() > 10) {
      imu_propagate(imu_data, g, pose_i, sb_i, pose_j, sb_j);
      imu_data.clear();
    }

    pose_i = pose_j;
    sb_i = sb_j;
  }

  print_vector("pose_j", pose_i);
  print_matrix("T_WS", tf(pose_i));

  // const auto pose_diff = T_WS - T_WS_j;
  // const auto pos_diff = tf_trans(pose_diff);
  // const auto rpy_diff = quat2euler(tf_quat(pose_diff));
	// print_vector("pos diff", pos_diff);
	// print_vector("rpy diff", rpy_diff);
	// MU_CHECK((pos_diff).norm() < 1e-2);
	// MU_CHECK((rpy_diff).norm() < 1e-2);
	// MU_CHECK((sb - sb_j).norm() < 1e-2);

  return 0;
}

int test_marg_factor() {
  // Setup parameters
  size_t next_param_id = 0;
	size_t nb_poses = 5;
	size_t nb_landmarks = 5;
  // -- Camera poses
  const vec3_t euler{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(euler));
  vec3_t r_WC{0.0, 0.0, 0.0};
  poses_t cam_poses;
  for (size_t i = 0; i < nb_poses; i++) {
    r_WC(1) += 0.05;
    const mat4_t T_WC = tf(C_WC, r_WC);
    cam_poses.emplace_back(next_param_id, 0, T_WC);
    next_param_id++;
  }
  // -- Camera geometry
  const int cam_index = 0;
  const int resolution[2] = {640, 480};
  const double fx = pinhole_focal(resolution[0], 90.0);
  const double fy = pinhole_focal(resolution[1], 90.0);
  const double cx = resolution[0] / 2.0;
  const double cy = resolution[1] / 2.0;
  vec4_t proj_params{fx, fy, cx, cy};
  vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  camera_params_t cam_params{next_param_id, cam_index, resolution,
														 proj_params, dist_params};
  next_param_id++;
  pinhole_radtan4_t camera{resolution, proj_params, dist_params};
  // -- Landmarks
	landmarks_t landmarks;
	for (size_t i = 0; i < nb_landmarks; i++) {
		const vec3_t p_W{1.0, 0.01 * i, 0.0};
		landmarks.emplace_back(next_param_id, p_W);
		next_param_id++;
	}

  // Create ba factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  std::vector<factor_t *> factors;
  for (size_t i = 0; i < cam_poses.size(); i++) {
		for (size_t j = 0; j < landmarks.size(); j++) {
			mat4_t T_WC = tf(cam_poses[i].param);
			vec3_t p_C = tf_point(T_WC.inverse(), landmarks[j].param);

			vec2_t z;
			camera.project(p_C, z);
			// z(0) += 5.0;
			// z(1) += 5.0;

			std::vector<param_t *> params{&cam_poses[i], &landmarks[j], &cam_params};
			factor_t *factor = new ba_factor_t<pinhole_radtan4_t>{id, ts, z, I(2), params};
			factor->eval();
			factors.push_back(factor);
		}
  }

  // Create marginalization factor
  marg_factor_t marg;
  for (size_t i = 0; i < factors.size(); i++) {
    if (i < landmarks.size()) {
      marg.add(factors[i], {0});
    } else {
      marg.add(factors[i], {});
    }
  }
  // MU_CHECK(marg.marg_params.size() == 1);
  // MU_CHECK(marg.remain_params.size() == 2);

  marg.eval();
  OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/H.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/b.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hmm.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hmr.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hrm.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hrr.csv");
  OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/H_marg.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/b_marg.csv");

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

int test_graph_add_camera() {
  graph_t graph;

  int cam_index = 0;
  int resolution[2] = {0, 0};
  vec4_t proj_params{0.0, 0.0, 0.0, 0.0};
  vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  graph_add_camera(graph, cam_index, resolution, proj_params, dist_params);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_speed_bias() {
  graph_t graph;

  int imu_index = 0;
  vec3_t v{1.0, 2.0, 3.0};
  vec3_t ba{4.0, 5.0, 6.0};
  vec3_t bg{7.0, 8.0, 9.0};
  graph_add_speed_bias(graph, imu_index, v, ba, bg);

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  return 0;
}

int test_graph_add_pose_factor() {
  graph_t graph;

  timestamp_t ts = 0;
  mat4_t T_WS = I(4);
  auto pose_id = graph_add_pose(graph, ts, T_WS);
  graph_add_pose_factor(graph, pose_id, T_WS);


  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  MU_CHECK(graph.factors.size() == 1);
  MU_CHECK(graph.factors[0] != nullptr);

  auto pose_param = graph.factors[0]->params[0];
  MU_CHECK(graph.param_factor.size() == 1);
  MU_CHECK(graph.param_factor[pose_param].size() == 1);
  MU_CHECK(graph.param_factor[pose_param][0] == graph.factors[0]);

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
  const int resolution[2] = {640, 480};
  const vec4_t proj_params{640, 480, 320, 240};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  const auto cam_id = graph_add_camera(graph, cam_index, resolution,
                                       proj_params, dist_params);

  // BA factor
  const pinhole_radtan4_t cm{resolution, proj_params, dist_params};

  vec2_t z;
  cm.project(tf_point(T_WC.inverse(), p_W), z);
  graph_add_ba_factor<pinhole_radtan4_t>(
    graph,
    ts,
    cam_pose_id,
    landmark_id,
    cam_id,
    z
  );

  MU_CHECK(graph.params.size() == 3);
  MU_CHECK(graph.params[0] != nullptr);
  MU_CHECK(graph.params[1] != nullptr);
  MU_CHECK(graph.params[2] != nullptr);

  MU_CHECK(graph.factors.size() == 1);

  MU_CHECK(graph.param_factor.size() == 3);
  for (const auto &param : graph.factors[0]->params) {
    MU_CHECK(graph.param_factor[param].size() == 1);
    MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  }

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
  const int resolution[2] = {640, 480};
  const vec4_t proj_params{640, 480, 320, 240};
  const vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  const auto cam_id = graph_add_camera(graph, cam_index, resolution,
                                              proj_params, dist_params);

  // BA factor
  const pinhole_radtan4_t cm{resolution, proj_params, dist_params};
  vec2_t z;
  cm.project(tf_point((T_WS * T_SC).inverse(), p_W), z);
  graph_add_cam_factor<pinhole_radtan4_t>(
    graph,
    ts,
    sensor_pose_id,
    imucam_pose_id,
    landmark_id,
    cam_id,
    z
  );

  MU_CHECK(graph.params.size() == 4);
  MU_CHECK(graph.params[0] != nullptr);
  MU_CHECK(graph.params[1] != nullptr);
  MU_CHECK(graph.params[2] != nullptr);
  MU_CHECK(graph.params[3] != nullptr);

  MU_CHECK(graph.factors.size() == 1);

  MU_CHECK(graph.param_factor.size() == 4);
  for (const auto &param : graph.factors[0]->params) {
    MU_CHECK(graph.param_factor[param].size() == 1);
    MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  }

  return 0;
}

int test_graph_add_imu_factor() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Create graph
  graph_t graph;
  size_t nb_imu_meas = 10;
  timestamp_t t0 = sim_data.imu_ts[0];
  timestamp_t t1 = sim_data.imu_ts[nb_imu_meas];

  // -- Add sensor pose at i
  const mat4_t T_WS_i = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  auto pose0_id = graph_add_pose(graph, t0, T_WS_i);
  // -- Add speed and bias at i
  const vec3_t v_WS_i = sim_data.imu_vel[0];
  const vec3_t ba_i{0.0, 0.0, 0.0};
  const vec3_t bg_i{0.0, 0.0, 0.0};
  auto sb0_id = graph_add_speed_bias(graph, t0, v_WS_i, ba_i, bg_i);
  // -- Add sensor pose at j
  const mat4_t T_WS_j = tf(sim_data.imu_rot[nb_imu_meas], sim_data.imu_pos[nb_imu_meas]);
  auto pose1_id = graph_add_pose(graph, t1, T_WS_j);
  // -- Add speed and bias at j
  const vec3_t v_WS_j = sim_data.imu_vel[nb_imu_meas];
  const vec3_t ba_j{0.0, 0.0, 0.0};
  const vec3_t bg_j{0.0, 0.0, 0.0};
  auto sb1_id = graph_add_speed_bias(graph, t1, v_WS_j, ba_j, bg_j);
  // -- Add imu factor
  const int imu_index = 0;
  graph_add_imu_factor(graph,
                       imu_index,
                       slice(sim_data.imu_ts, 0, nb_imu_meas),
                       slice(sim_data.imu_acc, 0, nb_imu_meas),
                       slice(sim_data.imu_gyr, 0, nb_imu_meas),
                       pose0_id,
                       sb0_id,
                       pose1_id,
                       sb1_id);

  // Asserts
  MU_CHECK(graph.params.size() == 4);
  MU_CHECK(graph.factors.size() == 1);

  MU_CHECK(graph.param_factor.size() == 4);
  for (const auto &param : graph.factors[0]->params) {
    MU_CHECK(graph.param_factor[param].size() == 1);
    MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  }

  return 0;
}

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
  const int resolution[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(resolution[0], lens_hfov);
  const real_t fy = pinhole_focal(resolution[1], lens_vfov);
  const real_t cx = resolution[0] / 2.0;
  const real_t cy = resolution[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.01, 0.001, 0.0001, 0.0001};
  const pinhole_radtan4_t cam0{resolution, proj_params, dist_params};

  // Create graph
  graph_t graph;
  bool prior_set = false;

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    graph_add_landmark(graph, feature);
  }
  // -- Add cam0 parameters
  int cam_index = 0;
  const auto cam_id = graph_add_camera(graph, cam_index, resolution,
                                       proj_params, dist_params);

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
        graph_add_ba_factor<pinhole_radtan4_t>(graph, ts,
                                               cam0_pose_id,
                                               feature_id,
                                               cam_id,
                                               z);
      }
      printf("nb features: %zu\n", event.frame.feature_ids.size());

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
  // mat2csv("/tmp/r.csv", r);
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/J.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/r.csv");

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_sim.m");
  }

  return 0;
}

static mat3_t load_camera(const std::string &data_path) {
  // Setup csv path
  char cam_csv[1000] = {0};
  strcat(cam_csv, data_path.c_str());
  strcat(cam_csv, "/camera.csv");

  // Parse csv file
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **cam_K = csv_data(cam_csv, &nb_rows, &nb_cols);
  if (cam_K == NULL) {
    FATAL("Failed to load csv file [%s]!", cam_csv);
  }
  if (nb_rows != 3 || nb_cols != 3) {
    LOG_ERROR("Error while parsing camera file [%s]!", cam_csv);
    LOG_ERROR("-- Expected 3 rows got %d instead!", nb_rows);
    LOG_ERROR("-- Expected 3 cols got %d instead!", nb_cols);
    FATAL("Invalid camera file [%s]!", cam_csv);
  }

  // Flatten 2D array to 1D array
  mat3_t K;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      K(i, j) = cam_K[i][j];
    }
    free(cam_K[i]);
  }
  free(cam_K);

  return K;
}

static poses_t load_camera_poses(const std::string &data_path) {
  char cam_poses_csv[1000] = {0};
  strcat(cam_poses_csv, data_path.c_str());
  strcat(cam_poses_csv, "/camera_poses.csv");
  return load_poses(cam_poses_csv);
}

static poses_t load_target_pose(const std::string &data_path) {
  char target_pose_csv[1000] = {0};
  strcat(target_pose_csv, data_path.c_str());
  strcat(target_pose_csv, "/target_pose.csv");
  return load_poses(target_pose_csv);
}

static real_t **load_points(const std::string &data_path, int *nb_points) {
  char points_csv[1000] = {0};
  strcat(points_csv, data_path.c_str());
  strcat(points_csv, "/points.csv");

  // Initialize memory for points
  *nb_points = csv_rows(points_csv);
  real_t **points = (real_t **) malloc(sizeof(real_t *) * *nb_points);
  for (int i = 0; i < *nb_points; i++) {
    points[i] = (real_t *) malloc(sizeof(real_t) * 3);
  }

  // Load file
  FILE *infile = fopen(points_csv, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[1024] = {0};
  size_t len_max = 1024;
  int point_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        points[point_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    point_idx++;
  }

  // Cleanup
  fclose(infile);

  return points;
}

static int **load_point_ids(const std::string &data_path, int *nb_points) {
  char csv_path[1000] = {0};
  strcat(csv_path, data_path.c_str());
  strcat(csv_path, "/point_ids.csv");
  return load_iarrays(csv_path, nb_points);
}

struct ba_data_t {
  mat3_t cam_K;

  poses_t cam_poses;
  pose_t target_pose;
  int nb_frames;

  std::vector<keypoints_t> keypoints;
  int **point_ids;
  int nb_ids;

  real_t **points;
  int nb_points;

  ba_data_t(const std::string &data_path) {
    cam_K = load_camera(data_path);
    cam_poses = load_camera_poses(data_path);
    target_pose = load_target_pose(data_path)[0];
    nb_frames = cam_poses.size();
    keypoints = load_keypoints(data_path);
    point_ids = load_point_ids(data_path, &nb_ids);
    points = load_points(data_path, &nb_points);
  }

  ~ba_data_t() {
    // Point IDs
    for (int i = 0; i < nb_frames; i++) {
      free(point_ids[i]);
    }
    free(point_ids);

    // Points
    for (int i = 0; i < nb_points; i++) {
      free(points[i]);
    }
    free(points);
  }
};

int test_graph_solve_ba() {
  ba_data_t data{TEST_BA_DATA};

  // Create graph
  bool prior_set = false;
  graph_t graph;

  // -- Add landmarks
  for (int i = 0; i < data.nb_points; i++) {
    const vec3_t p{data.points[i]};
    graph_add_landmark(graph, p);
  }

  // -- Add cam0 parameters
  const int cam_index = 0;
  const int resolution[2] = {640, 480};
  const real_t fx = data.cam_K(0, 0);
  const real_t fy = data.cam_K(1, 1);
  const real_t cx = data.cam_K(0, 2);
  const real_t cy = data.cam_K(1, 2);
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  auto cam_id = graph_add_camera(graph, cam_index, resolution,
                                 proj_params, dist_params);

  for (int k = 0; k < data.nb_frames; k++) {
    const timestamp_t ts = k * 1e9;

    // -- Add cam0 pose
    const auto T_WC0 = data.cam_poses[k];
    const size_t cam0_pose_id = graph_add_pose(graph, ts, T_WC0.tf());
    if (prior_set == false) {
      graph_add_pose_factor(graph, cam0_pose_id, T_WC0.tf());
      prior_set = true;
    }

    // -- Add cam0 observations at timestep k
    int nb_points = data.point_ids[k][0];
    for (int i = 0; i < nb_points; i++) {
      const auto point_id = data.point_ids[k][i + 1];
      const auto z = data.keypoints[k][i];
      graph_add_ba_factor<pinhole_radtan4_t>(graph, ts,
                                             cam0_pose_id,
                                             point_id,
                                             cam_id,
                                             z);
    }
  }

  // Solve graph
  tiny_solver_t solver;
  solver.verbose = true;
  solver.time_limit = 10.0;
  solver.max_iter = 5;
  solver.lambda = 1e-4;
  solver.update_factor = 2;
  solver.solve(graph);
  printf("solver took: %fs\n", solver.solve_time);

  return 0;
}

int test_graph_solve_vo() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
  sim_data.save("/tmp/sim_data");

  // Create graph
  bool prior_set = false;
  graph_t graph;
  std::vector<size_t> pose_ids;

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    const vec3_t noise{randf(-0.1, 0.1), randf(-0.1, 0.1), randf(-0.1, 0.1)};
    graph_add_landmark(graph, feature + noise);
  }

  // -- Add cam0 parameters
  int cam_index = 0;
  const int resolution[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(resolution[0], lens_hfov);
  const real_t fy = pinhole_focal(resolution[1], lens_vfov);
  const real_t cx = resolution[0] / 2.0;
  const real_t cy = resolution[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  auto cam0_id = graph_add_camera(graph, cam_index, resolution,
                                  proj_params, dist_params);

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
      const vec3_t noise{randf(-0.05, 0.05), randf(-0.05, 0.05), randf(-0.05, 0.05)};
      const vec3_t r_WC0 = sim_data.cam_pos[pose_idx] + noise;
      const mat4_t T_WC0 = tf(q_WC0, r_WC0);
      const size_t cam0_pose_id = graph_add_pose(graph, ts, T_WC0);
      pose_ids.push_back(cam0_pose_id);
      pose_idx++;

      if (prior_set == false) {
        graph_add_pose_factor(graph, cam0_pose_id, T_WC0);
        prior_set = true;
      }

      // Add cam0 observations at ts
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_id = event.frame.feature_ids[i];
        const auto z = event.frame.keypoints[i];
        graph_add_ba_factor<pinhole_radtan4_t>(graph,
                                               ts,
                                               cam0_pose_id,
                                               feature_id,
                                               cam0_id,
                                               z);
      }

      cam_pose++;
      if (cam_pose == 30) {
        break;
      }
    }
  }

  tiny_solver_t solver;
  solver.max_iter = 10;
  solver.time_limit = 10.0;
	solver.verbose = true;
  solver.solve(graph);
  printf("solver took: %fs\n", solver.solve_time);

  FILE *est_csv = fopen("/tmp/sim_data/cam0_pose_est.csv", "w");
  for (const auto &id : pose_ids) {
    const auto ts = graph.params[id]->ts;
    const auto pose = graph.params[id]->param;
    const auto q = pose.head(4);
    const auto r = pose.tail(3);
		fprintf(est_csv, "%" PRIu64 ",", ts);
    fprintf(est_csv, "%f,%f,%f,%f,", q(0), q(1), q(2), q(3));
    fprintf(est_csv, "%f,%f,%f\n", r(0), r(1), r(2));
  }
  fclose(est_csv);

  // Debug
  const bool debug = true;
  // const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_test_vo.m");
  }

  return 0;
}

int test_graph_solve_vio() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
  sim_data.save("/tmp/sim_data");

  // Create graph
  graph_t graph;
  graph.param_order = {"pose_t",
                       "sb_params_t",
                       "camera_params_t",
                       "extrinsic_t",
                       "landmark_t"};

	vec3_t g{0.0, 0.0, -9.81};
  std::map<int, cam_frame_t> cam_frames;
  std::vector<size_t> pose_ids;
  size_t pose_id = 0;
  size_t sb_id = 0;

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    const vec3_t noise{randf(-0.1, 0.1), randf(-0.1, 0.1), randf(-0.1, 0.1)};
    graph_add_landmark(graph, feature + noise);
  }

  // -- Add cam0 parameters
  int cam_index = 0;
  const int resolution[2] = {640, 480};
  const real_t lens_hfov = 90.0;
  const real_t lens_vfov = 90.0;
  const real_t fx = pinhole_focal(resolution[0], lens_hfov);
  const real_t fy = pinhole_focal(resolution[1], lens_vfov);
  const real_t cx = resolution[0] / 2.0;
  const real_t cy = resolution[1] / 2.0;
  const vec4_t proj_params{fx, fy, cx, cy};
  const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
  const auto cam0_id = graph_add_camera(graph, cam_index, resolution,
                                  			proj_params, dist_params);

  // -- Add imu cam pose parameter
	const vec3_t rpy{-M_PI / 2.0, 0.0, -M_PI / 2.0};
	const mat3_t C_SC = euler321(rpy);
	const vec3_t r_SC{0.0, 0.0, 0.0};
	const mat4_t T_SC = tf(C_SC, r_SC);
	const auto imucam0_id = graph_add_extrinsic(graph, T_SC);

  // Add pose prior
  size_t pose_idx = 0;
  {
    // -- Add first pose parameter
    const auto ts = sim_data.imu_ts[0];
    const auto T_WS = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
    pose_id = graph_add_pose(graph, ts, T_WS);
    pose_ids.push_back(pose_id);
    pose_idx++;
    // -- Add first sb parameter
    const vec3_t v = sim_data.imu_vel[0];
    const vec3_t ba{0.0, 0.0, 0.0};
    const vec3_t bg{0.0, 0.0, 0.0};
    sb_id = graph_add_speed_bias(graph, ts, v, ba, bg);
    // -- Add pose prior
    graph_add_pose_factor(graph, pose_id, T_WS);
  }

  // -- Add cam0 poses and ba factors
  imu_data_t imu_data;
  std::set<size_t> unique_features;
  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    const real_t x = randf(-0.05, 0.05);
    const real_t y = randf(-0.05, 0.05);
    const real_t z = randf(-0.05, 0.05);
    const vec3_t noise{x, y, z};

    // Handle imu event
    if (event.type == sim_event_type_t::IMU) {
      imu_data.add(ts, event.imu.accel, event.imu.gyro);
    }

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      if (imu_data.size() < 2) {
        continue;
      }

      // Propagate imu measurements
      const size_t pose_i_id = pose_id;
      const size_t sb_i_id = sb_id;
      const vec_t<7> pose_i = graph.params[pose_id]->param;
      const vec_t<9> sb_i = graph.params[sb_id]->param;
      vec_t<7> pose_j = pose_i;
      vec_t<9> sb_j = sb_i;
      imu_propagate(imu_data, g, pose_i, sb_i, pose_j, sb_j);
      pose_id = graph_add_pose(graph, ts, pose_j);
      sb_id = graph_add_speed_bias(graph, ts, sb_j);
      pose_ids.push_back(pose_id);
      pose_idx++;

      // Add imu0 factor
      const int imu_idx = 0;
      graph_add_imu_factor(graph, imu_idx,
                           imu_data.timestamps, imu_data.accel, imu_data.gyro,
                           pose_i_id, sb_i_id, pose_id, sb_id);
      imu_data.clear();

      // Add cam0 factor
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_id = event.frame.feature_ids[i];
        unique_features.insert(feature_id);
        const auto z = event.frame.keypoints[i];
        graph_add_cam_factor<pinhole_radtan4_t>(graph,
                                                ts,
                                                pose_id,
                                                imucam0_id,
                                                feature_id,
                                                cam0_id,
                                                z);
      }

      if (pose_idx == 10) {
        break;
      }
    }
  }

  // Solve graph
  tiny_solver_t solver;
  solver.verbose = true;
  solver.max_iter = 30;
  solver.time_limit = 10.0;
  solver.solve(graph);

  camera_params_t *cam_params = (camera_params_t *) graph.params[cam0_id];
  print_vector("proj param", cam_params->proj_params());
  print_vector("dist param", cam_params->dist_params());
  printf("nb unique features: %zu\n", unique_features.size());

  FILE *est_csv = fopen("/tmp/sim_data/imu_pose_est.csv", "w");
  for (const auto &id : pose_ids) {
    const auto ts = graph.params[id]->ts;
    const auto pose = graph.params[id]->param;
    const auto q = pose.head(4);
    const auto r = pose.tail(3);
		fprintf(est_csv, "%" PRIu64 ",", ts);
    fprintf(est_csv, "%f,%f,%f,%f,", q(0), q(1), q(2), q(3));
    fprintf(est_csv, "%f,%f,%f\n", r(0), r(1), r(2));
  }
  fclose(est_csv);

  // Debug
  const bool debug = true;
  // const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_test_vio.m");
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
	MU_ADD_TEST(test_imu_propagate);
	MU_ADD_TEST(test_marg_factor);

  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_add_pose);
  MU_ADD_TEST(test_graph_add_landmark);
  MU_ADD_TEST(test_graph_add_camera);
  MU_ADD_TEST(test_graph_add_speed_bias);
  MU_ADD_TEST(test_graph_add_pose_factor);
  MU_ADD_TEST(test_graph_add_ba_factor);
  MU_ADD_TEST(test_graph_add_cam_factor);
  MU_ADD_TEST(test_graph_add_imu_factor);
  MU_ADD_TEST(test_graph_eval);
  MU_ADD_TEST(test_graph_solve_ba);
  MU_ADD_TEST(test_graph_solve_vo);
  MU_ADD_TEST(test_graph_solve_vio);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
