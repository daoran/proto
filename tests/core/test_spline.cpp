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

void test_suite() {
  MU_ADD_TEST(test_ctraj);
  MU_ADD_TEST(test_ctraj_get_pose);
  MU_ADD_TEST(test_ctraj_get_velocity);
  MU_ADD_TEST(test_ctraj_get_acceleration);
  MU_ADD_TEST(test_ctraj_get_angular_velocity);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
