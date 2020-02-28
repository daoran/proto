#include "proto/munit.hpp"
#include "proto/estimation/ba.hpp"

namespace proto {

#define TEST_DATA "./test_data/estimation/ba_data"

int test_parse_keypoints_line() {
  std::string line = "4,1,2,3,4\n";
  keypoints_t keypoints = parse_keypoints_line(line.c_str());

  keypoints_print(keypoints);
  MU_CHECK(keypoints.size() == 2);
  MU_CHECK(fltcmp(keypoints[0](0), 1.0) == 0);
  MU_CHECK(fltcmp(keypoints[0](1), 2.0) == 0);
  MU_CHECK(fltcmp(keypoints[1](0), 3.0) == 0);
  MU_CHECK(fltcmp(keypoints[1](1), 4.0) == 0);

  return 0;
}

int test_load_keypoints() {
  std::vector<keypoints_t> keypoints = load_keypoints(TEST_DATA);

  for (const auto &kp : keypoints) {
    MU_CHECK(kp.size() > 0);
    /* printf("frame[%d]\n", i); */
    /* keypoints_print(keypoints[i]); */
  }

  return 0;
}

int test_ba_residuals() {
  ba_data_t data{TEST_DATA};

  vecx_t r = ba_residuals(data);
  for (int i = 0; i < r.rows(); i++) {
    MU_CHECK(r[i] < 0.01);
  }

  const double cost = ba_cost(r);
  printf("Cost: %f\n", cost);

  return 0;
}

int test_ba_jacobian() {
  ba_data_t data{TEST_DATA};

  // for (const auto pose : data.cam_poses) {
  //   std::cout << pose.q.w() << ", ";
  //   std::cout << pose.q.x() << ", ";
  //   std::cout << pose.q.y() << ", ";
  //   std::cout << pose.q.z() << std::endl;
  // }

  matx_t J = ba_jacobian(data);

  mat2csv("/tmp/J.csv", J);

  return 0;
}

int test_ba_update() {
  ba_data_t data{TEST_DATA};
  const vecx_t e = ba_residuals(data);
  const matx_t E = ba_jacobian(data);
  ba_update(data, e, E);

  return 0;
}

int test_ba_cost() {
  vecx_t e{5};
  e << 1.0, 2.0, 3.0, 4.0, 5.0;
  const double cost = ba_cost(e);

  printf("Cost: %f\n", cost);
  MU_CHECK(fltcmp(cost, 27.50) == 0);

  return 0;
}

int test_ba_solve() {
  struct timespec t_start = tic();
  ba_data_t data{TEST_DATA};
  ba_solve(data);
  printf("time taken: %fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data.nb_frames);
  printf("nb_points: %d\n", data.nb_points);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_parse_keypoints_line);
  MU_ADD_TEST(test_load_keypoints);
  MU_ADD_TEST(test_ba_residuals);
  MU_ADD_TEST(test_ba_jacobian);
  MU_ADD_TEST(test_ba_update);
  MU_ADD_TEST(test_ba_cost);
  MU_ADD_TEST(test_ba_solve);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
