#include "prototype/munit.hpp"
#include "prototype/dataset/kitti.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "test_data/dataset/kitti/raw"
#define TEST_DATA_PATH "test_data/dataset/kitti/raw/2011_09_26"

int test_kitti_parse_string() {
  std::string value = kitti_parse_string("X: Hello World");
  MU_CHECK_EQ("Hello World", value);
  return 0;
}

int test_kitti_parse_double() {
  double value = kitti_parse_double("X: 1.23");
  MU_CHECK_FLOAT(1.23, value);
  return 0;
}

int test_kitti_parse_array() {
  std::vector<double> value = kitti_parse_array("X: 1.0 2.0 3.0");

  MU_CHECK_EQ(3, (int) value.size());
  MU_CHECK_FLOAT(1.0, value[0]);
  MU_CHECK_FLOAT(2.0, value[1]);
  MU_CHECK_FLOAT(3.0, value[2]);

  return 0;
}

int test_kitti_parse_vec2() {
  vec2_t value = kitti_parse_vec2("X: 1.0 2.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));

  return 0;
}

int test_kitti_parse_vec3() {
  vec3_t value = kitti_parse_vec3("X: 1.0 2.0 3.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));

  return 0;
}

int test_kitti_parse_vecx() {
  vecx_t value = kitti_parse_vecx("X: 1.0 2.0 3.0 4.0 5.0 6.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));
  MU_CHECK_FLOAT(4.0, value(3));
  MU_CHECK_FLOAT(5.0, value(4));
  MU_CHECK_FLOAT(6.0, value(5));

  return 0;
}

int test_kitti_parse_mat3() {
  mat3_t value = kitti_parse_mat3("X: 1 2 3 4 5 6 7 8 9");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(1, 0));
  MU_CHECK_FLOAT(5.0, value(1, 1));
  MU_CHECK_FLOAT(6.0, value(1, 2));
  MU_CHECK_FLOAT(7.0, value(2, 0));
  MU_CHECK_FLOAT(8.0, value(2, 1));
  MU_CHECK_FLOAT(9.0, value(2, 2));

  return 0;
}

int test_kitti_parse_mat34() {
  mat34_t value = kitti_parse_mat34("X: 1 2 3 4 5 6 7 8 9 10 11 12");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(0, 3));

  MU_CHECK_FLOAT(5.0, value(1, 0));
  MU_CHECK_FLOAT(6.0, value(1, 1));
  MU_CHECK_FLOAT(7.0, value(1, 2));
  MU_CHECK_FLOAT(8.0, value(1, 3));

  MU_CHECK_FLOAT(9.0, value(2, 0));
  MU_CHECK_FLOAT(10.0, value(2, 1));
  MU_CHECK_FLOAT(11.0, value(2, 2));
  MU_CHECK_FLOAT(12.0, value(2, 3));

  return 0;
}

int test_calib_cam2cam_load() {
  calib_cam2cam_t calib{TEST_DATA_PATH "/calib_cam_to_cam.txt"};

  int retval = calib_cam2cam_load(calib);
  MU_CHECK_EQ(retval, 0);

  const vec2_t S_00_exp{1.392000e+03, 5.120000e+02};
  MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

int test_calib_imu2velo_load() {
  calib_imu2velo_t calib{TEST_DATA_PATH "/calib_imu_to_velo.txt"};

  int retval = calib_imu2velo_load(calib);
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_calib_velo2cam_load() {
  calib_velo2cam_t calib{TEST_DATA_PATH "/calib_velo_to_cam.txt"};

  int retval = calib_velo2cam_load(calib);
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_oxts_load() {
  oxts_t oxts{TEST_DATA_PATH "/oxts"};
  oxts_load(oxts);

  for (size_t i = 0; i < oxts.timestamps.size(); i++) {
    MU_CHECK(oxts.time[i] >= 0.0);
  }

  return 0;
}

int test_kitti_raw_load() {
  kitti_raw_t dataset(TEST_DATA_BASEPATH, "2011_09_26", "0001");
  MU_CHECK(kitti_raw_load(dataset) == 0);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_kitti_parse_string);
  MU_ADD_TEST(test_kitti_parse_double);
  MU_ADD_TEST(test_kitti_parse_array);
  MU_ADD_TEST(test_kitti_parse_vec2);
  MU_ADD_TEST(test_kitti_parse_vec3);
  MU_ADD_TEST(test_kitti_parse_vecx);
  MU_ADD_TEST(test_kitti_parse_mat3);
  MU_ADD_TEST(test_kitti_parse_mat34);

  MU_ADD_TEST(test_calib_cam2cam_load);
  MU_ADD_TEST(test_calib_imu2velo_load);
  MU_ADD_TEST(test_calib_velo2cam_load);
  MU_ADD_TEST(test_oxts_load);

  MU_ADD_TEST(test_kitti_raw_load);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
