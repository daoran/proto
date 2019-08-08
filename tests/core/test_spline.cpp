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
      sin(2 * M_PI * 2 * ts_s_k) + 0.1,
      sin(2 * M_PI * 2 * ts_s_k) + 0.2,
      sin(2 * M_PI * 2 * ts_s_k) + 0.3
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

void test_suite() {
  MU_ADD_TEST(test_ctraj);
  MU_ADD_TEST(test_ctraj_get_pose);
  MU_ADD_TEST(test_ctraj_get_velocity);
  MU_ADD_TEST(test_ctraj_get_acceleration);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
