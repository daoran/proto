#include "prototype/munit.hpp"
#include "prototype/core/data.hpp"

#define TEST_DATA "test_data/core/data/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"

namespace proto {

int test_csvrows() {
  int rows;
  rows = csvrows(TEST_DATA);
  MU_CHECK_EQ(281, rows);
  return 0;
}

int test_csvcols() {
  int cols;
  cols = csvcols(TEST_DATA);
  MU_CHECK_EQ(2, cols);
  return 0;
}

int test_csv2mat() {
  matx_t data;

  csv2mat(TEST_DATA, true, data);
  MU_CHECK_EQ(280, data.rows());
  MU_CHECK_EQ(2, data.cols());
  MU_CHECK_FLOAT(-2.22482078596, data(0, 0));
  MU_CHECK_FLOAT(9.9625789766, data(0, 1));
  MU_CHECK_FLOAT(47.0485650525, data(279, 0));
  MU_CHECK_FLOAT(613.503760567, data(279, 1));

  return 0;
}

int test_mat2csv() {
  matx_t x;
  matx_t y;

  csv2mat(TEST_DATA, true, x);
  mat2csv(TEST_OUTPUT, x);
  csv2mat(TEST_OUTPUT, false, y);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      MU_CHECK_NEAR(x(i, j), y(i, j), 0.1);
    }
  }

  return 0;
}

int test_slerp() {
  for (int i = 0; i < 1000; i++) {
    const double roll_start = randf(-1.0, 1.0);
    const double pitch_start = randf(-1.0, 1.0);
    const double yaw_start = randf(-1.0, 1.0);
    const vec3_t rpy_start{roll_start, pitch_start, yaw_start};

    const double roll_end = randf(-1.0, 1.0);
    const double pitch_end = randf(-1.0, 1.0);
    const double yaw_end = randf(-1.0, 1.0);
    const vec3_t rpy_end{roll_end, pitch_end, yaw_end};

    const double alpha = randf(0.0, 1.0);
    const auto q0 = quat_t{euler321(rpy_start)};
    const auto q1 = quat_t{euler321(rpy_end)};
    const auto expect = q0.slerp(alpha, q1);
    const auto actual = slerp(q0, q1, alpha);

    MU_CHECK_FLOAT(expect.w(), actual.w());
    MU_CHECK_FLOAT(expect.x(), actual.x());
    MU_CHECK_FLOAT(expect.y(), actual.y());
    MU_CHECK_FLOAT(expect.z(), actual.z());
  }

  return 0;
}

int test_interp_pose() {
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(10.0), deg2rad(0.0), deg2rad(0.0)};

  const auto pose_start = tf(euler321(rpy_start), trans_start);
  const auto pose_end = tf(euler321(rpy_end), trans_end);
  const auto pose_interp = interp_pose(pose_start, pose_end, 0.5);

  std::cout << "pose_start:\n" << pose_start << std::endl << std::endl;
  std::cout << "pose_end:\n" << pose_end << std::endl << std::endl;
  std::cout << "pose_interp:\n" << pose_interp << std::endl << std::endl;

  MU_CHECK((tf_trans(pose_interp) - vec3_t{0.5, 1.0, 1.5}).norm() - 1e-5);

  return 0;
}

int test_interp_poses() {
  // Create timestamps
  timestamps_t timestamps;
  timestamps.push_back(1500000000000000000);
  timestamps.push_back(1500000000200000000);
  timestamps.push_back(1500000000400000000);
  timestamps.push_back(1500000000600000000);
  timestamps.push_back(1500000000800000000);
  timestamps.push_back(1500000001000000000);
  timestamps.push_back(1500000001200000000);
  timestamps.push_back(1500000001400000000);
  timestamps.push_back(1500000001600000000);
  timestamps.push_back(1500000001800000000);

  // Create poses
  mat4s_t poses;
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(90.0), deg2rad(90.0), deg2rad(90.0)};

  const size_t nb_timestamps = timestamps.size();
  const double step = 1.0 / nb_timestamps;
  vec3s_t trans_interp_gnd;
  for (size_t i = 0; i < nb_timestamps; i++) {
    const auto trans_interp = lerp(trans_start, trans_end, step * i);
    const auto rpy_interp = lerp(rpy_start, rpy_end, step * i);
    const auto rot = euler321(rpy_interp);
    const auto T = tf(rot, trans_interp);

    poses.push_back(T);
  }

  // Create interpolate points in time
  timestamps_t interp_ts;
  // interp_ts.push_back(1500000000100000000);
  // interp_ts.push_back(1500000000300000000);
  // interp_ts.push_back(1500000000500000000);
  // interp_ts.push_back(1500000000700000000);
  // interp_ts.push_back(1500000000900000000);
  interp_ts.push_back(1500000001100000000);
  // interp_ts.push_back(1500000001300000000);
  // interp_ts.push_back(1500000001500000000);
  // interp_ts.push_back(1500000001700000000);

  // Interpolate poses
  mat4s_t interped_poses;
  interp_poses(timestamps, poses, interp_ts, interped_poses);
  MU_CHECK(interped_poses.size() == poses.size());

  return 0;
}

int test_closest_poses() {
  // Create timestamps
  timestamps_t timestamps;
  timestamps.push_back(1500000000000000000);
  timestamps.push_back(1500000000200000000);
  timestamps.push_back(1500000000400000000);
  timestamps.push_back(1500000000600000000);
  timestamps.push_back(1500000000800000000);
  timestamps.push_back(1500000001000000000);
  timestamps.push_back(1500000001200000000);
  timestamps.push_back(1500000001400000000);
  timestamps.push_back(1500000001600000000);
  timestamps.push_back(1500000001800000000);

  // Create poses
  mat4s_t poses;
  const vec3_t trans_start{0.0, 0.0, 0.0};
  const vec3_t trans_end{1.0, 2.0, 3.0};
  const vec3_t rpy_start{0.0, 0.0, 0.0};
  const vec3_t rpy_end{deg2rad(1.0), deg2rad(2.0), deg2rad(3.0)};

  const size_t nb_timestamps = timestamps.size();
  const double step = 1.0 / nb_timestamps;
  vec3s_t trans_interp_gnd;
  for (size_t i = 0; i < nb_timestamps; i++) {
    const auto trans_interp = lerp(trans_start, trans_end, step * i);
    const auto rpy_interp = lerp(rpy_start, rpy_end, step * i);
    const auto rot = euler321(rpy_interp);
    const auto T = tf(rot, trans_interp);
    poses.push_back(T);
  }

  // Create interpolate points in time
  timestamps_t target_ts;
  target_ts.push_back(1500000000100000000);
  // target_ts.push_back(1500000000300000000);
  // target_ts.push_back(1500000000500000000);
  target_ts.push_back(1500000000700000000);
  // target_ts.push_back(1500000000900000000);
  // target_ts.push_back(1500000001100000000);
  // target_ts.push_back(1500000001300000000);
  // target_ts.push_back(1500000001500000000);
  // target_ts.push_back(1500000001700000000);

  // Interpolate poses
  mat4s_t result;
  closest_poses(timestamps, poses, target_ts, result);

  printf("Poses:\n");
  printf("------------------------------------------------\n");
  int index = 0;
  for (const auto &tf : poses) {
    printf("index[%d]\ttimestamp[%ld]\n", index, timestamps[index]);
    print_quaternion("rot:", quat_t{tf_rot(tf)});
    print_vector("trans: ", tf_trans(tf));
    printf("\n");
    index++;
  }

  printf("Result:\n");
  printf("------------------------------------------------\n");
  index = 0;
  for (const auto &tf : result) {
    printf("index[%d]\ttimestamp[%ld]\n", index, target_ts[index]);
    print_quaternion("rot:", quat_t{tf_rot(tf)});
    print_vector("trans: ", tf_trans(tf));
    printf("\n");
    index++;
  }

  return 0;
}

int test_intersection() {
  std::vector<int> v1 = {1, 2, 3, 4, 5};
  std::vector<int> v2 = {2, 4, 6};
  std::vector<int> v3 = {1, 4};
  std::list<std::vector<int>> data = {v1, v2, v3};

  auto result = intersection(data);
  MU_CHECK(result.size() == 1);
  MU_CHECK(result.find(4) != result.end());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_csvrows);
  MU_ADD_TEST(test_csvcols);
  MU_ADD_TEST(test_csv2mat);
  MU_ADD_TEST(test_mat2csv);
  MU_ADD_TEST(test_slerp);
  MU_ADD_TEST(test_interp_pose);
  MU_ADD_TEST(test_interp_poses);
  MU_ADD_TEST(test_closest_poses);
  MU_ADD_TEST(test_intersection);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
