#include "prototype/dataset/euroc_mav.hpp"
#include "prototype/munit.hpp"

namespace prototype {
namespace euroc {

#define TEST_CAM0_DATA "test_data/euroc/cam0"
#define TEST_IMU_DATA "test_data/euroc/imu0"
#define TEST_GROUND_TRUTH_DATA "test_data/euroc/state_groundtruth_estimate0"

int test_imu_data_constructor() {
  imu_data_t imu_data;

  // Data
  MU_CHECK_EQ(0, imu_data.timestamps.size());
  MU_CHECK_EQ(0, imu_data.time.size());
  MU_CHECK_EQ(0, imu_data.w_B.size());
  MU_CHECK_EQ(0, imu_data.a_B.size());

  // Sensor properties
  MU_CHECK_EQ("", imu_data.sensor_type);
  MU_CHECK_EQ("", imu_data.comment);
  MU_CHECK(imu_data.T_BS.isApprox(I(4)));
  MU_CHECK_FLOAT(0.0, imu_data.rate_hz);
  MU_CHECK_FLOAT(0.0, imu_data.gyro_noise_density);
  MU_CHECK_FLOAT(0.0, imu_data.gyro_random_walk);
  MU_CHECK_FLOAT(0.0, imu_data.accel_noise_density);
  MU_CHECK_FLOAT(0.0, imu_data.accel_random_walk);

  return 0;
}

int test_imu_data_load() {
  imu_data_t imu_data;

  int retval = imu_data_load(imu_data, TEST_IMU_DATA);
  MU_CHECK_EQ(0, retval);

  MU_CHECK_EQ(91, imu_data.timestamps.size());
  MU_CHECK_EQ(91, imu_data.time.size());
  MU_CHECK_EQ(91, imu_data.w_B.size());
  MU_CHECK_EQ(91, imu_data.a_B.size());

  MU_CHECK_EQ("imu", imu_data.sensor_type);
  MU_CHECK_EQ("VI-Sensor IMU (ADIS16448)", imu_data.comment);
  MU_CHECK_FLOAT(1.6968e-04, imu_data.gyro_noise_density);
  MU_CHECK_FLOAT(1.9393e-05, imu_data.gyro_random_walk);
  MU_CHECK_FLOAT(2.0000e-3, imu_data.accel_noise_density);
  MU_CHECK_FLOAT(3.0000e-3, imu_data.accel_random_walk);

  return 0;
}


int test_camera_data_constructor() {
  camera_data_t camera_data;

  MU_CHECK_EQ(0, camera_data.timestamps.size());
  MU_CHECK_EQ(0, camera_data.time.size());
  MU_CHECK_EQ(0, camera_data.image_paths.size());

  MU_CHECK_EQ("", camera_data.sensor_type);
  MU_CHECK_EQ("", camera_data.comment);
  MU_CHECK(camera_data.T_BS.isApprox(I(4)));
  MU_CHECK_FLOAT(0.0, camera_data.rate_hz);
  MU_CHECK(vec2_t::Zero().isApprox(camera_data.resolution));
  MU_CHECK_EQ("", camera_data.camera_model);
  MU_CHECK(vec4_t::Zero().isApprox(camera_data.intrinsics));
  MU_CHECK_EQ("", camera_data.distortion_model);
  MU_CHECK(vec4_t::Zero().isApprox(camera_data.distortion_coefficients));

  return 0;
}

int test_camera_data_load() {
  camera_data_t camera_data;

  // Sensor extrinsics wrt. the body-frame.
  // clang-format off
  mat4_t T_BS_expected;
  T_BS_expected << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                   0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                   -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                   0.0, 0.0, 0.0, 1.0;
  const vec2_t resolution_expected(752.0, 480.0);
  const vec4_t intrinsics_expected(458.654, 457.296, 367.215, 248.375);
  const vec4_t distortion_expected(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
  // clang-format on

  int retval = camera_data_load(camera_data, TEST_CAM0_DATA);
  MU_CHECK_EQ(0.0, retval);
  MU_CHECK_EQ(10, camera_data.timestamps.size());
  MU_CHECK_EQ(10, camera_data.time.size());
  MU_CHECK_EQ(10, camera_data.image_paths.size());

  MU_CHECK_EQ("camera", camera_data.sensor_type);
  MU_CHECK_EQ("VI-Sensor cam0 (MT9M034)", camera_data.comment);
  MU_CHECK(T_BS_expected.isApprox(camera_data.T_BS));
  MU_CHECK_FLOAT(20.0, camera_data.rate_hz);
  MU_CHECK(resolution_expected.isApprox(camera_data.resolution));
  MU_CHECK_EQ("pinhole", camera_data.camera_model);
  MU_CHECK(intrinsics_expected.isApprox(camera_data.intrinsics));
  MU_CHECK_EQ("radial-tangential", camera_data.distortion_model);
  MU_CHECK(distortion_expected.isApprox(camera_data.distortion_coefficients));

  return 0;
}

int test_ground_truth_constructor() {
  ground_truth_t ground_truth;

  MU_CHECK_EQ(0, ground_truth.timestamps.size());
  MU_CHECK_EQ(0, ground_truth.time.size());
  MU_CHECK_EQ(0, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.q_RS.size());
  MU_CHECK_EQ(0, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(0, ground_truth.b_a_RS_S.size());

  return 0;
}

int test_ground_truth_load() {
  ground_truth_t ground_truth;

  int retval = ground_truth_load(ground_truth, TEST_GROUND_TRUTH_DATA);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(999, ground_truth.timestamps.size());
  MU_CHECK_EQ(999, ground_truth.time.size());
  MU_CHECK_EQ(999, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.q_RS.size());
  MU_CHECK_EQ(999, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(999, ground_truth.b_a_RS_S.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_imu_data_constructor);
  MU_ADD_TEST(test_imu_data_load);
  MU_ADD_TEST(test_camera_data_constructor);
  MU_ADD_TEST(test_camera_data_load);
  MU_ADD_TEST(test_ground_truth_constructor);
  MU_ADD_TEST(test_ground_truth_load);
}

} // namespace euroc
} // namespace prototype

MU_RUN_TESTS(prototype::euroc::test_suite);
