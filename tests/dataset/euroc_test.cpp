#include "prototype/munit.hpp"
#include "prototype/dataset/euroc.hpp"

namespace proto {

#define TEST_DATA "test_data/dataset/euroc/V1_01_easy"
#define TEST_IMU_DATA "test_data/dataset/euroc/V1_01_easy/mav0/imu0"
#define TEST_CAM0_DATA "test_data/dataset/euroc/V1_01_easy/mav0/cam0"
#define TEST_GROUND_TRUTH_DATA \
  "test_data/dataset/euroc/V1_01_easy/mav0/state_groundtruth_estimate0"
#define TEST_CALIB_DATA "test_data/dataset/euroc/cam_april"

int test_euroc_imu_constructor() {
  euroc_imu_t imu_data;

  // Data
  MU_CHECK_EQ(0, imu_data.timestamps.size());
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

int test_euroc_imu_load() {
  euroc_imu_t imu_data;

  int retval = euroc_imu_load(imu_data, TEST_IMU_DATA);
  MU_CHECK_EQ(0, retval);
  MU_CHECK(imu_data.timestamps.size());
  MU_CHECK(imu_data.w_B.size());
  MU_CHECK(imu_data.a_B.size());

  const vec3_t w_B_gnd{-0.0020943951023931952,
                       0.017453292519943295,
                       0.07749261878854824};
  const vec3_t a_B_gnd{9.0874956666666655,
                       0.13075533333333333,
                       -3.6938381666666662};
  MU_CHECK((w_B_gnd - imu_data.w_B[0]).norm() < 1.0e-5);
  MU_CHECK((a_B_gnd - imu_data.a_B[0]).norm() < 1.0e-5);

  MU_CHECK_EQ("imu", imu_data.sensor_type);
  MU_CHECK_EQ("VI-Sensor IMU (ADIS16448)", imu_data.comment);
  MU_CHECK_FLOAT(1.6968e-04, imu_data.gyro_noise_density);
  MU_CHECK_FLOAT(1.9393e-05, imu_data.gyro_random_walk);
  MU_CHECK_FLOAT(2.0000e-3, imu_data.accel_noise_density);
  MU_CHECK_FLOAT(3.0000e-3, imu_data.accel_random_walk);

  return 0;
}

int test_euroc_camera_constructor() {
  euroc_camera_t camera_data;

  MU_CHECK_EQ(0, camera_data.timestamps.size());
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

int test_euroc_camera_load() {
  euroc_camera_t camera_data;

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

  int retval = euroc_camera_load(camera_data, TEST_CAM0_DATA);
  MU_CHECK_EQ(0.0, retval);
  MU_CHECK(camera_data.timestamps.size());
  MU_CHECK(camera_data.image_paths.size());
  MU_CHECK(camera_data.timestamps[0] == 1403715273262142976);

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

int test_euroc_ground_truth_constructor() {
  euroc_ground_truth_t ground_truth;

  MU_CHECK_EQ(0, ground_truth.timestamps.size());
  MU_CHECK_EQ(0, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.q_RS.size());
  MU_CHECK_EQ(0, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(0, ground_truth.b_a_RS_S.size());

  return 0;
}

int test_euroc_ground_truth_load() {
  euroc_ground_truth_t ground_truth;

  int retval = euroc_ground_truth_load(ground_truth, TEST_GROUND_TRUTH_DATA);
  MU_CHECK_EQ(0, retval);
  MU_CHECK(ground_truth.timestamps.size());
  MU_CHECK(ground_truth.p_RS_R.size());
  MU_CHECK(ground_truth.q_RS.size());
  MU_CHECK(ground_truth.v_RS_R.size());
  MU_CHECK(ground_truth.b_w_RS_S.size());
  MU_CHECK(ground_truth.b_a_RS_S.size());

  const timestamp_t ts_gnd = 1403715274302142976;
  const vec3_t p_gnd{0.878612,2.142470,0.947262};
  const vec3_t v_gnd{0.009474,-0.014009,-0.002145};
  const vec3_t gyro_bias{-0.002229,0.020700,0.076350};
  const vec3_t accel_bias{-0.012492,0.547666,0.069073};
  MU_CHECK(ts_gnd == ground_truth.timestamps[0]);
  MU_CHECK((p_gnd - ground_truth.p_RS_R[0]).norm() < 1e-5);
  MU_CHECK((v_gnd - ground_truth.v_RS_R[0]).norm() < 1e-5);
  MU_CHECK((gyro_bias - ground_truth.b_w_RS_S[0]).norm() < 1e-5);
  MU_CHECK((accel_bias - ground_truth.b_a_RS_S[0]).norm() < 1e-5);

  return 0;
}

int test_euroc_data_constructor() {
  euroc_data_t data("/tmp");
  MU_CHECK(data.ok == false);
  MU_CHECK_EQ("/tmp", data.data_path);

  return 0;
}

int test_euroc_data_load() {
  euroc_data_t data;
  MU_CHECK_EQ(0, euroc_data_load(data, TEST_DATA));

  return 0;
}

int test_euroc_calib_constructor() {
  euroc_calib_t data(TEST_CALIB_DATA);
  MU_CHECK_EQ(TEST_CALIB_DATA, data.data_path);

  return 0;
}

int test_euroc_calib_load() {
  {
    euroc_calib_t data;
    MU_CHECK(data.ok == false);
    MU_CHECK_EQ(0, euroc_calib_load(data, TEST_CALIB_DATA));
    MU_CHECK(data.ok);
  }

  {
    euroc_calib_t data{TEST_CALIB_DATA};
    MU_CHECK(data.ok);
  }

  return 0;
}

// int test_process_stereo_images() {
//   // Load calibration data
//   euroc_calib_t calib_data{"/data/euroc_mav/imu_april"};
//   MU_CHECK(calib_data.ok);
//
//   // Process stereo images
//   const std::string preprocess_path = "/tmp/stereo_data";
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//   int retval = process_stereo_images(calib_data,
//                                      preprocess_path,
//                                      cam0_grids,
//                                      cam1_grids);
//   MU_CHECK(cam0_grids.size() > 0);
//   MU_CHECK(cam1_grids.size() > 0);
//   MU_CHECK(cam0_grids.size() == cam1_grids.size());
//   MU_CHECK(retval == 0);
//
//   return 0;
// }
//
// int test_create_timeline() {
//   // Load calibration data
//   euroc_calib_t calib_data{"/data/euroc_mav/imu_april"};
//   MU_CHECK(calib_data.ok);
//
//   // Process stereo images
//   const std::string preprocess_path = "/tmp/stereo_data";
//   aprilgrids_t cam0_grids;
//   aprilgrids_t cam1_grids;
//   process_stereo_images(calib_data, preprocess_path, cam0_grids, cam1_grids);
//
//   // Create timeline
//   mat4_t T_SC0;
//   mat4s_t T_WS;
//   mat4_t T_WF;
//   timestamp_t t0;
//   const auto timeline = create_timeline(calib_data,
//                                         cam0_grids,
//                                         cam1_grids,
//                                         T_SC0,
//                                         T_WS,
//                                         T_WF,
//                                         t0);
//   MU_CHECK(timeline.data.size() > 0);
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_euroc_imu_constructor);
  MU_ADD_TEST(test_euroc_imu_load);

  MU_ADD_TEST(test_euroc_camera_constructor);
  MU_ADD_TEST(test_euroc_camera_load);

  MU_ADD_TEST(test_euroc_ground_truth_constructor);
  MU_ADD_TEST(test_euroc_ground_truth_load);

  MU_ADD_TEST(test_euroc_data_constructor);
  MU_ADD_TEST(test_euroc_data_load);

  MU_ADD_TEST(test_euroc_calib_constructor);
  MU_ADD_TEST(test_euroc_calib_load);

  // MU_ADD_TEST(test_process_stereo_images);
  // MU_ADD_TEST(test_create_timeline);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
