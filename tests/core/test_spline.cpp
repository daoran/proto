#include <fstream>

#include "proto/munit.hpp"
#include "proto/core/tf.hpp"
#include "proto/core/spline.hpp"

namespace proto {

void generate_trajectory(timestamps_t &timestamps,
                         vec3s_t &positions,
                         quats_t &orientations) {
  timestamp_t ts_k = 0;
  const timestamp_t ts_end = 5.0 * 1e9;
  const double f = 100.0;
  const timestamp_t dt = (1 / f) * 1e9;

  while (ts_k <= ts_end) {
    // Time
    timestamps.push_back(ts_k);

    // Position
    const double ts_s_k = ts2sec(ts_k);
    positions.emplace_back(
      sin(2 * M_PI * 2 * ts_s_k) + 1.0,
      sin(2 * M_PI * 2 * ts_s_k) + 2.0,
      sin(2 * M_PI * 2 * ts_s_k) + 3.0
    );

    // Orientation
    const vec3_t rpy{
      sin(2 * M_PI * 2 * ts_s_k),
      sin(2 * M_PI * 2 * ts_s_k + M_PI / 4),
      sin(2 * M_PI * 2 * ts_s_k + M_PI / 2)
    };
    orientations.emplace_back(euler321(rpy));

    // Update
    ts_k += dt;
  }
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

static mat3_t so3_exp(const vec3_t &phi) {
  const double norm = phi.norm();
  if (norm < 1e-3) {
    return mat3_t{I(3) + skew(phi)};
  }

  const mat3_t phi_skew = skew(phi);
  mat3_t C = I(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

int test_ctraj() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/positions.csv", timestamps, positions);
  save_data("/tmp/orientations.csv", timestamps, orientations);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_data.m "
                  "/tmp/pos_data.csv "
                  "/tmp/att_data.csv ");
  }

  return 0;
}

int test_ctraj_get_pose() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t r;
    quats_t q;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);

      const auto T_WS = ctraj_get_pose(ctraj, ts_k);
      r.push_back(tf_trans(T_WS));
      q.push_back(tf_quat(T_WS));

      ts_k += dt;
    }

    save_data("/tmp/pos_interp.csv", t, r);
    save_data("/tmp/att_interp.csv", t, q);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_pose.m "
                  "/tmp/pos_data.csv "
                  "/tmp/att_data.csv "
                  "/tmp/pos_interp.csv "
                  "/tmp/att_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);

  {
    timestamps_t t;
    vec3s_t v;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      v.push_back(ctraj_get_velocity(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/vel_interp.csv", t, v);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_velocity.m "
                  "/tmp/pos.csv "
                  "/tmp/vel_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_acceleration() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t a;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = timestamps.back();
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      a.push_back(ctraj_get_acceleration(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/acc_interp.csv", t, a);
  }

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_acceleration.m "
                  "/tmp/pos_data.csv "
                  "/tmp/acc_interp.csv ");
  }

  return 0;
}

int test_ctraj_get_angular_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/att_data.csv", timestamps, orientations);

  // Setup
  timestamps_t t_hist;
  quats_t q_hist;
  quats_t q_prop_hist;
  vec3s_t w_hist;

  timestamp_t ts_k = 0;
  const timestamp_t ts_end = timestamps.back();
  const double f = 1000.0;
  const timestamp_t dt = (1 / f) * 1e9;

  // Initialize first attitude
  auto T_WB = ctraj_get_pose(ctraj, 0.0);
  mat3_t C_WB = tf_rot(T_WB);

  // Interpolate pose, angular velocity
  while (ts_k <= ts_end) {
    t_hist.push_back(ts_k);

    // Attitude at time k
    T_WB = ctraj_get_pose(ctraj, ts_k);
    const auto q_WB_k = tf_quat(T_WB);
    q_hist.push_back(q_WB_k);

    // Angular velocity at time k
    const auto w_WB_k = ctraj_get_angular_velocity(ctraj, ts_k);
    w_hist.push_back(w_WB_k);

    // Propagate angular velocity to obtain attitude at time k
    const mat3_t C_BW = tf_rot(T_WB).inverse();
    C_WB = C_WB * so3_exp(C_BW * w_WB_k * ts2sec(dt));
    q_prop_hist.emplace_back(quat_t{C_WB});

    ts_k += dt;
  }
  save_data("/tmp/att_interp.csv", t_hist, q_hist);
  save_data("/tmp/att_prop.csv", t_hist, q_prop_hist);
  save_data("/tmp/avel_interp.csv", t_hist, w_hist);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_angular_velocity.m "
                  "/tmp/att_data.csv "
                  "/tmp/att_interp.csv "
                  "/tmp/att_prop.csv "
                  "/tmp/avel_interp.csv ");
  }

  return 0;
}

int test_sim_imu_measurement() {
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

  // Generate trajectory
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;
  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/pos_data.csv", timestamps, positions);
  save_data("/tmp/att_data.csv", timestamps, orientations);

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
    sim_imu_measurement(
      imu,
      rndeng,
      ts_k,
      T_WS_W,
      w_WS_W,
      a_WS_W,
      a_WS_S,
      w_WS_S
    );

    // Propagate simulated IMU measurements
    const double dt_s = ts2sec(dt);
    const double dt_s_sq = dt_s * dt_s;
    const vec3_t g{0.0, 0.0, -imu.g};
    // -- Position at time k
    const vec3_t b_a = ones(3, 1) * imu.b_a;
    const vec3_t n_a = ones(3, 1) * imu.sigma_a_c;
    r_WS += v_WS * dt_s;
    r_WS += 0.5 * g * dt_s_sq;
    r_WS += 0.5 * C_WS * (a_WS_S - b_a - n_a) * dt_s_sq;
    // -- velocity at time k
    v_WS += C_WS * (a_WS_S - b_a - n_a) * dt_s + g * dt_s;
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

void test_suite() {
  MU_ADD_TEST(test_ctraj);
  MU_ADD_TEST(test_ctraj_get_pose);
  MU_ADD_TEST(test_ctraj_get_velocity);
  MU_ADD_TEST(test_ctraj_get_acceleration);
  MU_ADD_TEST(test_ctraj_get_angular_velocity);
  MU_ADD_TEST(test_sim_imu_measurement);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
