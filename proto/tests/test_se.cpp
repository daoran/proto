#include "munit.hpp"
#include "se.hpp"

namespace proto {

#define TEST_BA_DATA "./test_data/estimation/ba_data"
#define TEST_VO_CONFIG "./test_data/estimation/vo/config.yaml"
#define TEST_VIO_CONFIG "./test_data/estimation/vio/config.yaml"

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
  pose_factor_t factor(0, I(6), &pose);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_pose", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_extrinsic_factor_jacobians() {
  // Form measurement pose
  const vec3_t euler{randf(-0.5, 0.5), randf(-0.5, 0.5), randf(-0.5, 0.5)};
  const mat3_t C_WS = euler321(euler);
  const vec3_t r_WS{randf(-0.5, 0.5), randf(-0.5, 0.5), randf(-0.5, 0.5)};
  const mat4_t T_WS = tf(C_WS, r_WS);

  // Form estimation pose
  pose_t pose{0, 0, T_WS};

  // Pose factor
  extrinsic_factor_t factor(0, I(6), &pose);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_pose", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_speed_bias_factor_jacobians() {
  // Form estimation pose
  id_t id = 0;
  timestamp_t ts = 0;
  vec3_t v{0.1, 0.2, 0.3};
  vec3_t ba{0.1, 0.2, 0.3};
  vec3_t bg{0.1, 0.2, 0.3};
  sb_params_t sb{id, ts, v, ba, bg};

  // Speed bias factor
  speed_bias_factor_t factor(0, I(9), &sb);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_sb", step, threshold);
  MU_CHECK(retval == 0);

  return 0;
}

int test_camera_params_factor_jacobians() {
  // Form estimation pose
  id_t id = 0;
  int resolution[2] = {640, 480};
  vec4_t proj_params{640, 480, 320, 240};
  vec4_t dist_params{0.01, 0.001, 0.001, 0.001};
  camera_params_t cam_params{id, 0, resolution, proj_params, dist_params};

  // Speed bias factor
  camera_params_factor_t factor(0, I(8), &cam_params);

  // Check factor jacobian
  int retval = check_jacobians(&factor, 0, "J_cam_params", step, threshold);
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

int test_calib_mono_factor_jacobians() {
  // Setup parameters
  // -- Camera
  const int cam_index = 0;
  const int resolution[2] = {640, 480};
  const vec3_t rpy_WC{-90.0, 0.0, -90.0};
  const mat3_t C_WC = euler321(deg2rad(rpy_WC));
  const vec3_t r_WC{-10.0, 0.0, 0.0};
  const mat4_t T_WC = tf(C_WC, r_WC);
  pose_t cam_pose{0, 0, T_WC};
  // -- Fiducial Pose
  const vec3_t rpy_WF{90.0, 0.0, -90.0};
  const mat3_t C_WF = euler321(deg2rad(rpy_WF));
  const vec3_t r_WF{0.01, 0.02, 0.03};
  const mat4_t T_WF = tf(C_WF, r_WC);
  fiducial_pose_t fiducial_pose{1, T_WF};
  const int tag_id = 0;
  const int tag_corner = 0;
  const vec3_t r_FFi{0.01, 0.01, 0.0};
  // -- Camera intrinsics
  vec4_t proj_params{600.0, 600.0, 325.0, 240.0};
  vec4_t dist_params{0.15, -0.3, 0.0001, 0.001};
  camera_params_t cam_params{2, cam_index, resolution, proj_params, dist_params};

  // Create factor
  const timestamp_t ts = 0;
  const size_t id = 0;
  const vec2_t z{0.0, 0.0};
  std::vector<param_t *> params{
    &cam_pose,
    &fiducial_pose,
    &cam_params,
  };
  calib_mono_factor_t<pinhole_radtan4_t> factor{id, ts, tag_id, tag_corner,
                                                r_FFi, z, I(2), params};

  // Check ba factor parameter jacobians
  int retval = 0;
  // -- Check measurement model jacobian
  retval = check_J_h<pinhole_radtan4_t>(resolution, cam_params.param);
  MU_CHECK(retval == 0);
  // -- Check camera pose jacobian
  retval = check_jacobians(&factor, 0, "J_cam_pose", step, threshold);
  MU_CHECK(retval == 0);
  // -- Check landmark jacobian
  retval = check_jacobians(&factor, 1, "J_fiducial_pose", step, threshold);
  MU_CHECK(retval == 0);
  // // -- Check camera params jacobian
  // retval = check_jacobians(&factor, 2, "J_cam_params", step, threshold);
  // MU_CHECK(retval == 0);

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
    const vec3_t b_a = imu.b_a;
    const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
    r_WS += v_WS * dt_k;
    r_WS += 0.5 * g * dt_k_sq;
    r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_k_sq;
    // -- velocity at time k
    v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_k + g * dt_k;
    // -- Attitude at time k
    const vec3_t b_g = imu.b_g;
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
  const id_t factor_id = 0;
  const int imu_index = 0;
  imu_factor_t factor(factor_id, imu_index, imu_ts, imu_accel, imu_gyro, I(15), params);

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

  factor.eval();
  std::cout << factor.residuals.transpose() << std::endl;

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

  std::vector<param_t *> params = {
    new pose_t{0, imu_data.timestamps[0], sim_data.imu_poses.front()},
    new sb_params_t{1, imu_data.timestamps[0], sim_data.imu_vel.front(), zeros(3, 1), zeros(3, 1)},
    new pose_t{2, imu_data.timestamps.back(), sim_data.imu_poses.back()},
    new sb_params_t{3, imu_data.timestamps.back(), sim_data.imu_vel.back(), zeros(3, 1), zeros(3, 1)},
  };

  const id_t factor_id = 0;
  const int imu_index = 0;
  imu_factor_t factor(factor_id,
                      imu_index,
                      imu_data.timestamps,
                      imu_data.accel,
                      imu_data.gyro,
                      I(15),
                      params);
  factor.eval();
  print_vector("residuals", factor.residuals);

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
  id_t next_param_id = 0;
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
  const real_t fx = pinhole_focal(resolution[0], 90.0);
  const real_t fy = pinhole_focal(resolution[1], 90.0);
  const real_t cx = resolution[0] / 2.0;
  const real_t cy = resolution[1] / 2.0;
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
  cam_poses[0].marginalize = true;
  cam_poses[0].type = "marg_pose_t";
  landmarks[0].marginalize = true;
  landmarks[0].type = "marg_landmark_t";

  // marg_factor_t marg;
  // for (size_t i = 0; i < factors.size(); i++) {
  //   marg.add(factors[i]);
  // }
  // MU_CHECK(marg.marg_params.size() == 1);
  // MU_CHECK(marg.remain_params.size() == 2);

  // marg.eval();
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/J.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/H.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/b.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hmm.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hmr.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hrm.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/Hrr.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/H_marg.csv");
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
  auto factor_id = graph_add_pose_factor(graph, pose_id, I(6));

  MU_CHECK(graph.params.size() == 1);
  MU_CHECK(graph.params[0] != nullptr);

  MU_CHECK(graph.factors.size() == 1);
  MU_CHECK(graph.factors[0] != nullptr);

  MU_CHECK(graph.params[0]->factor_ids.size() == 1);
  MU_CHECK(graph.params[0]->factor_ids[0] == factor_id);

  // auto pose_param = graph.factors[0]->params[0];
  // MU_CHECK(graph.param_factor.size() == 1);
  // MU_CHECK(graph.param_factor[pose_param].size() == 1);
  // MU_CHECK(graph.param_factor[pose_param][0] == graph.factors[0]);

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
  auto factor_id = graph_add_ba_factor<pinhole_radtan4_t>(
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
  MU_CHECK(graph.params[0]->factor_ids.size() == 1);
  MU_CHECK(graph.params[1]->factor_ids.size() == 1);
  MU_CHECK(graph.params[2]->factor_ids.size() == 1);
  MU_CHECK(graph.params[0]->factor_ids[0] == factor_id);
  MU_CHECK(graph.params[1]->factor_ids[0] == factor_id);
  MU_CHECK(graph.params[2]->factor_ids[0] == factor_id);

  MU_CHECK(graph.factors.size() == 1);

  // MU_CHECK(graph.param_factor.size() == 3);
  // for (const auto &param : graph.factors[0]->params) {
  //   MU_CHECK(graph.param_factor[param].size() == 1);
  //   MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  // }

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
  auto factor_id = graph_add_cam_factor<pinhole_radtan4_t>(
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

  MU_CHECK(graph.params[0]->factor_ids.size() == 1);
  MU_CHECK(graph.params[1]->factor_ids.size() == 1);
  MU_CHECK(graph.params[2]->factor_ids.size() == 1);
  MU_CHECK(graph.params[3]->factor_ids.size() == 1);

  MU_CHECK(graph.params[0]->factor_ids[0] == factor_id);
  MU_CHECK(graph.params[1]->factor_ids[0] == factor_id);
  MU_CHECK(graph.params[2]->factor_ids[0] == factor_id);
  MU_CHECK(graph.params[3]->factor_ids[0] == factor_id);

  MU_CHECK(graph.factors.size() == 1);

  // MU_CHECK(graph.param_factor.size() == 4);
  // for (const auto &param : graph.factors[0]->params) {
  //   MU_CHECK(graph.param_factor[param].size() == 1);
  //   MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  // }

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

  // MU_CHECK(graph.param_factor.size() == 4);
  // for (const auto &param : graph.factors[0]->params) {
  //   MU_CHECK(graph.param_factor[param].size() == 1);
  //   MU_CHECK(graph.param_factor[param][0] == graph.factors[0]);
  // }

  return 0;
}

int test_graph_rm_param() {
  graph_t graph;

  // Sensor poses
  const mat3_t C_WS = I(3);
  const vec3_t r_WS = zeros(3, 1);
  const mat4_t T_WS = tf(C_WS, r_WS);
  const auto pose0_id = graph_add_pose(graph, 0, T_WS);
  const auto pose1_id = graph_add_pose(graph, 1, T_WS);
  const auto pose2_id = graph_add_pose(graph, 2, T_WS);
  const auto pose3_id = graph_add_pose(graph, 3, T_WS);
  MU_CHECK(graph.params.size() == 4);

  graph_rm_param(graph, pose0_id);
  MU_CHECK(graph.params.size() == 3);

  graph_rm_param(graph, pose1_id);
  MU_CHECK(graph.params.size() == 2);

  graph_rm_param(graph, pose2_id);
  MU_CHECK(graph.params.size() == 1);

  graph_rm_param(graph, pose3_id);
  MU_CHECK(graph.params.size() == 0);

  return 0;
}

int test_graph_rm_factor() {
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

  // Remove factor
  MU_CHECK(graph.factors.size() == 1);
  MU_CHECK(graph.params.size() == 4);
  MU_CHECK(graph.factors[0]->id == 0);
  graph_rm_factor(graph, 0);
  MU_CHECK(graph.factors.size() == 0);
  MU_CHECK(graph.params.size() == 4);

  return 0;
}

int test_graph_get_state() {
  graph_t graph;

  // Sensor poses
  const mat3_t C_WS = I(3);
  const vec3_t r_WS = zeros(3, 1);
  const mat4_t T_WS = tf(C_WS, r_WS);
  graph_add_pose(graph, 0, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 1, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 2, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 3, add_noise(T_WS, 0.1, 0.0));
  MU_CHECK(graph.params.size() == 4);

  const vecx_t x = graph_get_state(graph);
  // print_vector("x", x);
  MU_CHECK(x.size() == (7 * 4));

  return 0;
}

int test_graph_set_state() {
  graph_t graph;

  // Sensor poses
  const mat3_t C_WS = I(3);
  const vec3_t r_WS = zeros(3, 1);
  const mat4_t T_WS = tf(C_WS, r_WS);
  graph_add_pose(graph, 0, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 1, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 2, add_noise(T_WS, 0.1, 0.0));
  graph_add_pose(graph, 3, add_noise(T_WS, 0.1, 0.0));
  MU_CHECK(graph.params.size() == 4);

  vecx_t x = zeros((7 * 4), 1);
  for (int i = 0; i < 4; i++) {
    x(i * 7) = 1.0;
  }
  graph_set_state(graph, x);
  // print_vector("x", graph_get_state(graph));

  for (auto &kv : graph.params) {
    auto &param = kv.second;
    MU_CHECK_FLOAT(param->param(0), 1.0);
    MU_CHECK_FLOAT(param->param(1), 0.0);
    MU_CHECK_FLOAT(param->param(2), 0.0);
    MU_CHECK_FLOAT(param->param(3), 0.0);
  }

  return 0;
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
  // graph_add_camera_params_factor(graph, cam_id, I(8));

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
        graph_add_pose_factor(graph, cam0_pose_id, I(6));
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

      cam_pose++;
      if (cam_pose == 10) {
        break;
      }
    }
  }

  // Evaluate graph
  matx_t H;
  vecx_t g;
  size_t marg_size;
  size_t remain_size;
  graph_eval(graph, H, g, &marg_size, &remain_size);

  printf("rank(H): %ld\n", rank(H));
  printf("rows(H): %ld\n", H.rows());

  auto covar = H.llt().solve(I(H.rows()));
  // auto covar = H.householderQr().solve(I(H.rows()));
  printf("det(covar): %f\n", covar.determinant());

  // auto n = covar.rows();
  double k = pow((double) (2.0 * M_PI * 2.718281828459045), (double) 40.0);
  // auto det_covar = covar.determinant();
  // auto entropy = 0.5 * std::log2(k * det_covar);
  printf("k: %f\n", k);
  // printf("entropy: %f\n", entropy);

  // printf("marg_size: %zu\n", marg_size);
  // printf("remain_size: %zu\n", remain_size);
  mat2csv("/tmp/H.csv", H);
  // mat2csv("/tmp/g.csv", g);
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/H.csv");
  // OCTAVE_SCRIPT("scripts/estimation/plot_matrix.m /tmp/g.csv");

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_sim.m");
  }

  return 0;
}

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
      graph_add_pose_factor(graph, cam0_pose_id, I(6));
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

int test_swf_add_imu() {
  config_t config{TEST_VIO_CONFIG};

  swf_t swf;
  swf.add_imu(config);
  MU_CHECK(swf.imu_rate == 400);
  MU_CHECK(fltcmp(swf.g(0), 0.0) == 0);
  MU_CHECK(fltcmp(swf.g(1), 0.0) == 0);
  MU_CHECK(fltcmp(swf.g(2), -9.81) == 0);

  return 0;
}

int test_swf_add_camera() {
  config_t config{TEST_VIO_CONFIG};

  swf_t swf;
  swf.add_camera(config, 0);
  MU_CHECK(swf.camera_ids.size() == 1);
  MU_CHECK(swf.camera_ids.at(0) == 0);

  return 0;
}

int test_swf_add_extrinsics() {
  config_t config{TEST_VIO_CONFIG};

  swf_t swf;
  swf.add_camera(config, 0);
  swf.add_extrinsics(config, 0);
  MU_CHECK(swf.extrinsics_ids.size() == 1);
  MU_CHECK(swf.extrinsics_ids.at(0) == 1);

  return 0;
}

int test_swf_add_feature() {
  config_t config{TEST_VIO_CONFIG};

  swf_t swf;
  vec3_t feature{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0)};
  swf.add_feature(feature);
  MU_CHECK(swf.feature_ids.size() == 1);
  MU_CHECK(swf.feature_ids.at(0) == 0);

  return 0;
}

int test_swf_add_pose() {
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;

  mat4_t pose = I(4);
  auto pose_id = swf.add_pose(0, pose);

  MU_CHECK(swf.pose_ids.size() == 1);
  MU_CHECK(swf.pose_ids.at(0) == 0);

  MU_CHECK(swf.window.size() == 1);
  MU_CHECK(swf.window.back().ts == 0);
  MU_CHECK(swf.window.back().factor_ids.size() == 0);
  MU_CHECK(swf.window.back().feature_ids.size() == 0);
  MU_CHECK(swf.window.back().pose_id == pose_id);
  MU_CHECK(swf.window.back().sb_id == -1);

  return 0;
}

int test_swf_add_speed_bias() {
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;

  mat4_t pose = I(4);
  const auto pose_id = swf.add_pose(0, pose);
  MU_CHECK(swf.pose_ids.size() == 1);
  MU_CHECK(swf.pose_ids.at(0) == 0);
  MU_CHECK(swf.window.size() == 1);
  MU_CHECK(swf.window.front().ts == 0);

  const timestamp_t ts = 0;
  const vec3_t v{1.0, 2.0, 3.0};
  const vec3_t ba{1.0, 2.0, 3.0};
  const vec3_t bg{1.0, 2.0, 3.0};;
  const auto sb_id = swf.add_speed_bias(ts, v, ba, bg);

  MU_CHECK(swf.sb_ids.size() == 1);
  MU_CHECK(swf.sb_ids.at(0) == 1);

  MU_CHECK(swf.window.size() == 1);
  MU_CHECK(swf.window.front().ts == 0);
  MU_CHECK(swf.window.front().factor_ids.size() == 0);
  MU_CHECK(swf.window.front().feature_ids.size() == 0);
  MU_CHECK(swf.window.front().pose_id == pose_id);
  MU_CHECK(swf.window.front().sb_id == sb_id);

  return 0;
}

int test_swf_add_pose_prior() {
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;

  mat4_t pose = I(4);
  auto pose_id = swf.add_pose(0, pose);
  MU_CHECK(swf.pose_ids.size() == 1);
  MU_CHECK(swf.pose_ids.at(0) == 0);
  MU_CHECK(swf.window.size() == 1);
  MU_CHECK(swf.window.back().ts == 0);

  auto prior_id = swf.add_pose_prior(pose_id);
  MU_CHECK(swf.window.size() == 1);
  MU_CHECK(swf.window.back().ts == 0);
  MU_CHECK(swf.window.back().factor_ids.size() == 1);
  MU_CHECK(swf.window.back().factor_ids.front() == prior_id);
  MU_CHECK(swf.window.back().feature_ids.size() == 0);
  MU_CHECK(swf.window.back().pose_id == pose_id);
  MU_CHECK(swf.window.back().sb_id == -1);

  return 0;
}

int test_swf_add_ba_factor() {
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;

  const int cam_idx = 0;
  swf.add_camera(config, cam_idx);

  mat4_t pose = I(4);
  const id_t pose_id = swf.add_pose(0, pose);

  vec3_t feature{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0)};
  const id_t feature_id = swf.add_feature(feature);

  const timestamp_t ts = 0;
  const vec2_t z{0.0, 0.0};
  const id_t factor_id = swf.add_ba_factor(ts, cam_idx, pose_id, feature_id, z);

  swf.print_window();

  MU_CHECK(swf.nb_cams() == 1);
  MU_CHECK(swf.nb_features() == 1);

  MU_CHECK(swf.window_size() == 1);
  MU_CHECK(swf.window.back().ts == ts);
  MU_CHECK(swf.window.back().factor_ids.size() == 1);
  MU_CHECK(swf.window.back().factor_ids.back() == factor_id);
  MU_CHECK(swf.window.back().feature_ids.size() == 1);
  MU_CHECK(swf.window.back().feature_ids.back() == feature_id);
  MU_CHECK(swf.window.back().pose_id == pose_id);
  MU_CHECK(swf.window.back().sb_id == -1);

  return 0;
}

int test_swf_add_imu_factor() {
  // Simulation data
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Sliding window filter
  config_t config{TEST_VIO_CONFIG};
  imu_data_t imu_data;
  swf_t swf;
  // -- Add first pose parameter
  const auto ts = sim_data.imu_ts[0];
  const auto T_WS = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  const auto pose_id = swf.add_pose(ts, T_WS);
  // -- Add first sb parameter
  const vec3_t v = sim_data.imu_vel[0];
  const vec3_t ba{0.0, 0.0, 0.0};
  const vec3_t bg{0.0, 0.0, 0.0};
  const auto sb_id = swf.add_speed_bias(ts, v, ba, bg);
  // -- Add pose prior
  const auto pose_prior_id = swf.add_pose_prior(pose_id);
  // -- Obtain some imu measurements
  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;
    // Handle imu event
    if (event.type == sim_event_type_t::IMU) {
      imu_data.add(ts, event.imu.accel, event.imu.gyro);
    }

    // Collect 0.5s of imu data
    if (ts >= 0.5e9) {
      break;
    }
  }
  // -- Add imu factor
  const auto imu_factor_id = swf.add_imu_factor(0.5e9, imu_data);
  swf.print_window();

  MU_CHECK(swf.pose_ids.size() == 2);
  MU_CHECK(swf.pose_ids.front() == pose_id);
  MU_CHECK(swf.sb_ids.size() == 2);

  MU_CHECK(swf.window.size() == 2);
  MU_CHECK(swf.window.front().pose_id == pose_id);
  MU_CHECK(swf.window.front().sb_id == sb_id);
  MU_CHECK(swf.window.front().factor_ids.size() == 1);
  MU_CHECK(swf.window.front().factor_ids.front() == pose_prior_id);
  MU_CHECK(swf.window.front().feature_ids.size() == 0);

  MU_CHECK(swf.window.back().pose_id == sb_id + 1);
  MU_CHECK(swf.window.back().sb_id == sb_id + 2);
  MU_CHECK(swf.window.back().factor_ids.size() == 1);
  MU_CHECK(swf.window.back().factor_ids.front() == imu_factor_id);
  MU_CHECK(swf.window.back().feature_ids.size() == 0);

  return 0;
}

int test_swf_add_cam_factor() {
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;

  const int cam_idx = 0;
  swf.add_camera(config, cam_idx);

  mat4_t pose = I(4);
  const auto pose_id = swf.add_pose(0, pose);

  mat4_t ext = I(4);
  swf.add_extrinsics(0, ext);

  vec3_t feature{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0)};
  const auto feature_id = swf.add_feature(feature);

  const timestamp_t ts = 0;
  const vec2_t z{0.0, 0.0};
  const auto factor_id = swf.add_cam_factor(ts, cam_idx, pose_id, feature_id, z);

  swf.print_window();

  MU_CHECK(swf.nb_cams() == 1);
  MU_CHECK(swf.nb_features() == 1);
  MU_CHECK(swf.nb_extrinsics() == 1);

  MU_CHECK(swf.window_size() == 1);
  MU_CHECK(swf.window.back().ts == ts);
  MU_CHECK(swf.window.back().pose_id == pose_id);
  MU_CHECK(swf.window.back().sb_id == -1);
  MU_CHECK(swf.window.back().factor_ids.size() == 1);
  MU_CHECK(swf.window.back().factor_ids.back() == factor_id);
  MU_CHECK(swf.window.back().feature_ids.size() == 1);
  MU_CHECK(swf.window.back().feature_ids.back() == feature_id);

  return 0;
}

static void print_window(swf_t &swf) {
  int i = 0;
  for (auto &state : swf.window) {
    printf("state[%d]\n", i++);
    for (auto &factor_id : state.factor_ids) {
      const auto &factor = swf.graph.factors[factor_id];
      if (factor == nullptr) {
        continue;
      }


      printf("%s [%ld][%d]\t", factor->type.c_str(), factor->id, factor->marginalize);
      printf("{");
      for (const auto &param : factor->params) {
        printf("%s[%ld]\t", param->type.c_str(), param->id);
      }
      printf("}");
      printf("\n");
    }
    printf("\n");
  }
}

int test_swf_pre_marginalize() {
  // Simulation data
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Sliding window filter
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;
  swf.load_config(TEST_VIO_CONFIG);

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    swf.add_feature(add_noise(feature, 0.01));
  }

  // Initialize first pose
  // -- Add first pose parameter
  const auto ts = sim_data.imu_ts[0];
  const auto T_WS = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  const auto pose_id = swf.add_pose(ts, T_WS);
  // -- Add first sb parameter
  const vec3_t v = sim_data.imu_vel[0];
  const vec3_t ba{0.0, 0.0, 0.0};
  const vec3_t bg{0.0, 0.0, 0.0};
  swf.add_speed_bias(ts, v, ba, bg);
  // -- Add pose prior
  swf.add_pose_prior(pose_id);

  // -- Loop over data
  imu_data_t imu_data;

  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    // Handle imu event
    if (event.type == sim_event_type_t::IMU) {
      imu_data.add(ts, event.imu.accel, event.imu.gyro);
    }

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      if (imu_data.size() < 2) {
        continue;
      }

      // Add imu factor
      swf.add_imu_factor(ts, imu_data);
      imu_data.clear();

      // Add cam0 factors
      const auto cam_idx = event.sensor_id;
      auto pose_id = swf.window.back().pose_id;
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_idx = event.frame.feature_ids[i];
        const auto feature_id = swf.feature_ids.at(feature_idx);
        const auto z = event.frame.keypoints[i];
        swf.add_cam_factor(ts, cam_idx, pose_id, feature_id, z);
      }

      if (swf.window_size() >= 3) {
        break;
      }
    }
  }

  // printf("----------\n");
  // swf.print_info();
  // printf("\n");
  // swf.print_window();

  swf.pre_marginalize();
  print_window(swf);

  swf.marginalize();
  printf("\n");
  print_window(swf);
  printf("\n");
  printf("\n");

  swf.pre_marginalize();
  print_window(swf);

  swf.marginalize();
  printf("\n");
  print_window(swf);

  return 0;
}

int test_swf_marginalize() {
  // Simulation data
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);

  // Sliding window filter
  config_t config{TEST_VIO_CONFIG};
  swf_t swf;
  swf.load_config(TEST_VIO_CONFIG);

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    swf.add_feature(add_noise(feature, 0.01));
  }

  // Initialize first pose
  // -- Add first pose parameter
  const auto ts = sim_data.imu_ts[0];
  const auto T_WS = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  const auto pose_id = swf.add_pose(ts, T_WS);
  // -- Add first sb parameter
  const vec3_t v = sim_data.imu_vel[0];
  const vec3_t ba{0.0, 0.0, 0.0};
  const vec3_t bg{0.0, 0.0, 0.0};
  swf.add_speed_bias(ts, v, ba, bg);
  // -- Add pose prior
  swf.add_pose_prior(pose_id);

  // -- Loop over data
  imu_data_t imu_data;

  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    // Handle imu event
    if (event.type == sim_event_type_t::IMU) {
      imu_data.add(ts, event.imu.accel, event.imu.gyro);
    }

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      if (imu_data.size() < 2) {
        continue;
      }

      // Add imu factor
      swf.add_imu_factor(ts, imu_data);
      imu_data.clear();

      // Add cam0 factors
      const auto cam_idx = event.sensor_id;
      auto pose_id = swf.window.back().pose_id;
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        const auto feature_idx = event.frame.feature_ids[i];
        const auto feature_id = swf.feature_ids.at(feature_idx);
        const auto z = event.frame.keypoints[i];
        swf.add_cam_factor(ts, cam_idx, pose_id, feature_id, z);
      }

      if (swf.window_size() >= 5) {
        break;
      }
    }
  }

  printf("----------\n");
  swf.print_info();
  printf("\n");
  swf.print_window();

  swf.marginalize();

  printf("----------\n");
  swf.print_info();
  printf("\n");
  swf.print_window();

  printf("nb_factors: %ld\n", swf.graph.factors.size());
  printf("nb_params: %ld\n", swf.graph.params.size());

  return 0;
}

int test_swf_solve_vo() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
  sim_data.save("/tmp/sim_data");

  // Setup sliding window filter
  swf_t swf;
  swf.load_config(TEST_VO_CONFIG);

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    swf.add_feature(add_noise(feature, 0.01));
  }

  // -- Add cam0 poses and ba factors
  profiler_t profiler;
  profiler.start("solve_vo");

  bool prior_set = false;
  id_t pose_idx = 0;
  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;


    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      // Add pose
      const mat4_t T_WC = sim_data.cam_poses[pose_idx++];
      const id_t pose_id = swf.add_pose(ts, T_WC);

      // Add prior
      if (prior_set == false) {
        swf.add_pose_prior(pose_id);
        prior_set = true;
      }

      // Add cam0 observations at ts
      for (size_t i = 0; i < event.frame.feature_ids.size(); i++) {
        auto feature_index = event.frame.feature_ids[i];
        auto feature_id = swf.feature_ids.at(feature_index);
        auto z = event.frame.keypoints[i];
        swf.add_ba_factor(ts, 0, pose_id, feature_id, z);
      }

      swf.solve();
    }
  }
  profiler.stop("solve_vo", true);
  // swf.save_poses("/tmp/sim_data/cam0_pose_est.csv");

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/estimation/plot_test_vo.m");
  }

  return 0;
}

int test_swf_solve_vio() {
  vio_sim_data_t sim_data;
  sim_circle_trajectory(4.0, sim_data);
  sim_data.save("/tmp/sim_data");

  // Setup sliding window filter
  swf_t swf;
  swf.load_config(TEST_VIO_CONFIG);

  // -- Add landmarks
  for (const auto &feature : sim_data.features) {
    swf.add_feature(add_noise(feature, 0.01));
  }

  // Initialize first pose
  // -- Add first pose parameter
  const auto ts = sim_data.imu_ts[0];
  const auto T_WS = tf(sim_data.imu_rot[0], sim_data.imu_pos[0]);
  const auto pose_id = swf.add_pose(ts, T_WS);
  // -- Add first sb parameter
  const vec3_t v = sim_data.imu_vel[0];
  const vec3_t ba{0.0, 0.0, 0.0};
  const vec3_t bg{0.0, 0.0, 0.0};
  swf.add_speed_bias(ts, v, ba, bg);
  // -- Add pose prior
  swf.add_pose_prior(pose_id);

  // -- Loop over data
  imu_data_t imu_data;
  profiler_t profiler;
  profiler.start("solve_vio");

  bool frame_set = false;
  cam_frame_t frame;

  for (const auto &kv : sim_data.timeline) {
    const timestamp_t &ts = kv.first;
    const sim_event_t &event = kv.second;

    // Handle imu event
    if (event.type == sim_event_type_t::IMU) {
      printf("imu: %ld\n", ts);
      imu_data.add(ts, event.imu.accel, event.imu.gyro);
      if (imu_data.size() < 2 || frame_set == false) {
        continue;
      }
      // exit(0);

      // Add imu factor
      swf.add_imu_factor(ts, imu_data);
      imu_data.clear();

      // Add cam0 factors
      const auto cam_idx = event.sensor_id;
      auto pose_id = swf.window.back().pose_id;
      for (size_t i = 0; i < frame.feature_ids.size(); i++) {
        const auto feature_idx = frame.feature_ids[i];
        const auto feature_id = swf.feature_ids.at(feature_idx);
        const auto z = frame.keypoints[i];
        swf.add_cam_factor(ts, cam_idx, pose_id, feature_id, z);
      }

      swf.solve();
      frame_set = false;

      // if (swf.window_size() > 10) {
      //   goto end;
      // }
    }

    // Handle camera event
    if (event.type == sim_event_type_t::CAMERA) {
      printf("cam: %ld\n", ts);
      frame = event.frame;
      frame_set = true;
    }
  }

// end:
//   matx_t H;
//   vecx_t g;
//   size_t marg_size = 0;
//   size_t remain_size = 0;
//   graph_eval(swf.graph, H, g, &marg_size, &remain_size);
//
//   auto e = graph_residuals(swf.graph);
//   auto cost = 0.5 * e.transpose() * e;
//   printf("cost: %f\n", (double) cost);

  swf.solve();
  profiler.stop("solve_vio", true);
  swf.save_poses("/tmp/sim_data/imu_pose_est.csv");

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
  MU_ADD_TEST(test_extrinsic_factor_jacobians);
  MU_ADD_TEST(test_speed_bias_factor_jacobians);
  MU_ADD_TEST(test_camera_params_factor_jacobians);
  MU_ADD_TEST(test_ba_factor_jacobians);
  MU_ADD_TEST(test_calib_mono_factor_jacobians);
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
  MU_ADD_TEST(test_graph_rm_param);
  MU_ADD_TEST(test_graph_rm_factor);
  MU_ADD_TEST(test_graph_get_state);
  MU_ADD_TEST(test_graph_set_state);
  MU_ADD_TEST(test_graph_eval);
  MU_ADD_TEST(test_graph_solve_ba);

  MU_ADD_TEST(test_swf_add_imu);
  MU_ADD_TEST(test_swf_add_camera);
  MU_ADD_TEST(test_swf_add_extrinsics);
  MU_ADD_TEST(test_swf_add_feature);
  MU_ADD_TEST(test_swf_add_pose);
  MU_ADD_TEST(test_swf_add_speed_bias);
  MU_ADD_TEST(test_swf_add_pose_prior);
  MU_ADD_TEST(test_swf_add_ba_factor);
  MU_ADD_TEST(test_swf_add_imu_factor);
  MU_ADD_TEST(test_swf_add_cam_factor);
  MU_ADD_TEST(test_swf_pre_marginalize);
  MU_ADD_TEST(test_swf_marginalize);

  MU_ADD_TEST(test_swf_solve_vo);
  MU_ADD_TEST(test_swf_solve_vio);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
