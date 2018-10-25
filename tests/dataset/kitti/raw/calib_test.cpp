#include "prototype/munit.hpp"
#include "prototype/dataset/kitti.hpp"
#include "prototype/core/quaternion/jpl.hpp"

namespace prototype {

#define TEST_DATA "test_data/kitti/raw/2011_09_26"

int test_calib_cam2cam_load() {
  calib_cam2cam_t calib;

  int retval = calib_cam2cam_load(calib, TEST_DATA "/calib_cam_to_cam.txt");
  MU_CHECK_EQ(retval, 0);

  const vec2_t S_00_exp{1.392000e+03, 5.120000e+02};
  MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

int test_calib_imu2velo_load() {
  calib_imu2velo_t calib;

  int retval = calib_imu2velo_load(calib, TEST_DATA "/calib_imu_to_velo.txt");
  MU_CHECK_EQ(retval, 0);

  std::cout << calib.R << std::endl;

  // const vec2_t S_00_exp{1.392000e+03, 5.120000e+02};
  // MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  // MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

int test_calib_velo2cam_load() {
  calib_velo2cam_t calib;

  int retval = calib_velo2cam_load(calib, TEST_DATA "/calib_velo_to_cam.txt");
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_sandbox() {
  calib_imu2velo_t imu2velo;
  calib_velo2cam_t velo2cam;

  calib_imu2velo_load(imu2velo, TEST_DATA "/calib_imu_to_velo.txt");
  calib_velo2cam_load(velo2cam, TEST_DATA "/calib_velo_to_cam.txt");

  // Create transform matrices
  const mat4_t T_cam_imu = velo2cam.T_cam_velo * imu2velo.T_velo_imu;
  const mat4_t T_imu_cam = T_cam_imu.inverse();
  const vec3_t X{0.0, 10.0, 0.0};
  // std::cout << T_cam_imu * X.homogeneous() << std::endl;

  const vec4_t q = rot2quat(T_imu_cam.block(0, 0, 3, 3).transpose());
  std::cout << q.transpose() << std::endl;
  // std::cout << C(q) * X << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_cam2cam_load);
  MU_ADD_TEST(test_calib_imu2velo_load);
  MU_ADD_TEST(test_calib_velo2cam_load);
  MU_ADD_TEST(test_sandbox);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
