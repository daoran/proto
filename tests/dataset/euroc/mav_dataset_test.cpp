#include "prototype/core/jpl.hpp"
#include "prototype/dataset/euroc/mav_dataset.hpp"
#include "prototype/munit.hpp"

namespace prototype {

#define TEST_DATA "test_data/euroc"

int mono_camera_cb(const cv::Mat &frame) {
  UNUSED(frame);
  // cv::imshow("Mono camera", frame);
  // cv::waitKey(1);
  return 0;
}

int stereo_camera_cb(const cv::Mat &frame0, const cv::Mat &frame1) {
  cv::Mat stereo_img;
  cv::hconcat(frame0, frame1, stereo_img);

  cv::imshow("Stereo", stereo_img);
  cv::waitKey(1);
  return 0;
}

int test_mav_dataset_constructor() {
  mav_dataset_t ds("/tmp");

  MU_CHECK(ds.ok == false);
  MU_CHECK_EQ("/tmp", ds.data_path);

  return 0;
}

int test_mav_dataset_load() {
  mav_dataset_t ds{TEST_DATA};

  int retval = mav_dataset_load(ds);
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_mav_dataset_sandbox() {
  mav_dataset_t ds{TEST_DATA};

  int retval = mav_dataset_load(ds);
  MU_CHECK_EQ(0, retval);

  const mat4_t T_imu_cam0 = ds.cam0_data.T_BS;
  const mat4_t T_imu_cam1 = ds.cam1_data.T_BS;
  const vec3_t X{0.0, 0.0, 10.0};

  // clang-format off
  mat4_t T_body_imu;
  T_body_imu << 0.0, 0.0, 1.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  // clang-format on
  const mat4_t T_body_cam0 = T_body_imu * T_imu_cam0;
  const mat4_t T_cam0_body = T_body_cam0.inverse();

  // std::cout << T_body_cam0 * X.homogeneous() << std::endl;

  const vec4_t q_CI = rot2quat(T_cam0_body.block(0, 0, 3, 3));
  std::cout << "q_CI: " << q_CI.transpose() << std::endl;
  const vec3_t p_I_CI = T_cam0_body.block(0, 3, 3, 1);
  std::cout << "p_I_CI: " << p_I_CI.transpose() << std::endl;

  // std::cout << T_imu_cam0 << std::endl;
  // std::cout << T_imu_cam1 << std::endl;
  // std::cout << T_body_imu << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_mav_dataset_constructor);
  MU_ADD_TEST(test_mav_dataset_load);
  // MU_ADD_TEST(test_mav_dataset_sandbox);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
