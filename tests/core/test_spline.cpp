#include <fstream>

#include "proto/munit.hpp"
#include "proto/core/tf.hpp"
#include "proto/core/spline.hpp"

namespace proto {

void generate_trajectory(timestamps_t &timestamps,
                         vec3s_t &positions,
                         quats_t &orientations) {
  timestamp_t ts_k = 0;
  const timestamp_t ts_end = 10.0 * 1e9;
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
      // sin(2 * M_PI * 2 * ts_s_k + M_PI),
      // sin(2 * M_PI * 2 * ts_s_k + M_PI / 2)
      // 0, 0
      // 0, 0
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

// function C = so3_exp(phi)
//   if (phi < 1e-3)
//     C = eye(3) + skew(phi);
//   else
//     C = eye(3);
//     C += (sin(norm(phi)) / norm(phi)) * skew(phi);
//     C += ((1 - cos(norm(phi))) / norm(phi)^2) * skew(phi)^2;
//   endif
// endfunction

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

// function x_imu = imu_update(x_imu, a_B, w_B, dt)
//   b_a = x_imu.b_a;
//   b_g = x_imu.b_g;
//   g = x_imu.g;
//   n_a = zeros(3, 1);
//   n_g = zeros(3, 1);
//
//   C_WS = x_imu.C_WS;
//   v_WS = x_imu.v_WS;
//   p_WS = x_imu.p_WS;
//
//   x_imu.C_WS = C_WS * so3_exp((w_B - b_g - n_g) * dt);
//   x_imu.v_WS += (g * dt) + (C_WS * (a_B - b_a - n_a) * dt);
//   x_imu.p_WS += (v_WS * dt) + (0.5 * g * dt^2) + (0.5 * C_WS * (a_B - b_a - n_a) * dt^2);
// endfunction

int test_ctraj() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/desired_positions.csv", timestamps, positions);
  save_data("/tmp/desired_orientations.csv", timestamps, orientations);

  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_data.m "
                  "/tmp/desired_positions.csv "
                  "/tmp/desired_orientations.csv ");
  }

  return 0;
}

int test_ctraj_get_pose() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/desired_positions.csv", timestamps, positions);
  save_data("/tmp/desired_orientations.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t r;
    quats_t q;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = 10.0 * 1e9;
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);

      const auto T_WS = ctraj_get_pose(ctraj, ts_k);
      r.push_back(tf_trans(T_WS));
      q.push_back(tf_quat(T_WS));

      ts_k += dt;
    }

    save_data("/tmp/interp_positions.csv", t, r);
    save_data("/tmp/interp_orientations.csv", t, q);
  }

  const bool debug = true;
  // const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_pose.m "
                  "/tmp/desired_positions.csv "
                  "/tmp/desired_orientations.csv "
                  "/tmp/interp_positions.csv "
                  "/tmp/interp_orientations.csv ");
  }

  return 0;
}

int test_ctraj_get_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/desired_positions.csv", timestamps, positions);
  save_data("/tmp/desired_orientations.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t v;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = 10.0 * 1e9;
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      v.push_back(ctraj_get_velocity(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/velocity.csv", t, v);
  }

  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_velocity.m "
                  "/tmp/desired_positions.csv "
                  "/tmp/velocity.csv ");
  }

  return 0;
}

int test_ctraj_get_acceleration() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/desired_positions.csv", timestamps, positions);
  save_data("/tmp/desired_orientations.csv", timestamps, orientations);

  {
    timestamps_t t;
    vec3s_t a;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = 10.0 * 1e9;
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    while (ts_k <= ts_end) {
      t.push_back(ts_k);
      a.push_back(ctraj_get_acceleration(ctraj, ts_k));
      ts_k += dt;
    }

    save_data("/tmp/acceleration.csv", t, a);
  }

  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_acceleration.m "
                  "/tmp/desired_positions.csv "
                  "/tmp/acceleration.csv ");
  }

  return 0;
}

int test_ctraj_get_angular_velocity() {
  timestamps_t timestamps;
  vec3s_t positions;
  quats_t orientations;

  generate_trajectory(timestamps, positions, orientations);
  ctraj_t ctraj(timestamps, positions, orientations);
  save_data("/tmp/desired_orientations.csv", timestamps, orientations);

  {
    timestamps_t t_hist;
    quats_t q_hist;
    quats_t q_prop_hist;
    vec3s_t w_hist;

    timestamp_t ts_k = 0;
    const timestamp_t ts_end = 10.0 * 1e9;
    const double f = 1000.0;
    const timestamp_t dt = (1 / f) * 1e9;

    const auto pose = ctraj_get_pose(ctraj, 0.0);
    mat3_t C = tf_rot(pose);

    while (ts_k <= ts_end) {
      t_hist.push_back(ts_k);

      const auto pose = ctraj_get_pose(ctraj, ts_k);
      q_hist.push_back(tf_quat(pose));

      const auto w = ctraj_get_angular_velocity(ctraj, ts_k);
      w_hist.push_back(w);

      C = C * so3_exp(tf_rot(pose).inverse() * w * ts2sec(dt));
      q_prop_hist.push_back(quat_t{C});

      ts_k += dt;
    }

    save_data("/tmp/orientations.csv", t_hist, q_hist);
    save_data("/tmp/orientations_prop.csv", t_hist, q_prop_hist);
    save_data("/tmp/angular_velocity.csv", t_hist, w_hist);
  }

  const bool debug = true;
  // const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_spline_angular_velocity.m "
                  "/tmp/desired_orientations.csv "
                  "/tmp/orientations.csv "
                  "/tmp/angular_velocity.csv ");
    OCTAVE_SCRIPT("scripts/core/plot_spline_prop.m "
                  "/tmp/orientations.csv "
                  "/tmp/orientations_prop.csv ");
  }

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_ctraj);
  // MU_ADD_TEST(test_ctraj_get_pose);
  // MU_ADD_TEST(test_ctraj_get_velocity);
  // MU_ADD_TEST(test_ctraj_get_acceleration);
  MU_ADD_TEST(test_ctraj_get_angular_velocity);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
