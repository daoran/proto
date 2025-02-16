#include "munit.h"
#include "xyz_euroc.h"

int test_euroc_imu_load(void) {
  const char *data_dir = "/data/euroc/imu_april/mav0/imu0";
  euroc_imu_t *data = euroc_imu_load(data_dir);
  // euroc_imu_print(data);
  euroc_imu_free(data);
  return 0;
}

int test_euroc_camera_load(void) {
  const char *data_dir = "/data/euroc/imu_april/mav0/cam0";
  euroc_camera_t *data = euroc_camera_load(data_dir, 1);
  // euroc_camera_print(data);
  euroc_camera_free(data);
  return 0;
}

int test_euroc_ground_truth_load(void) {
  const char *data_dir = "/data/euroc/V1_01/mav0/state_groundtruth_estimate0";
  euroc_ground_truth_t *data = euroc_ground_truth_load(data_dir);
  // euroc_ground_truth_print(data);
  euroc_ground_truth_free(data);
  return 0;
}

int test_euroc_data_load(void) {
  const char *data_dir = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_dir);
  euroc_data_free(data);
  return 0;
}

int test_euroc_calib_target_load(void) {
  const char *config_path = "/data/euroc/imu_april/april_6x6.yaml";
  euroc_calib_target_t *data = euroc_calib_target_load(config_path);
  // euroc_calib_target_print(data);
  euroc_calib_target_free(data);
  return 0;
}

int test_euroc_calib_load(void) {
  const char *config_path = "/data/euroc/imu_april";
  euroc_calib_t *data = euroc_calib_load(config_path);
  euroc_calib_free(data);
  return 0;
}

void test_suite(void) {
#if CI_MODE == 0
  MU_ADD_TEST(test_euroc_imu_load);
  MU_ADD_TEST(test_euroc_camera_load);
  MU_ADD_TEST(test_euroc_ground_truth_load);
  MU_ADD_TEST(test_euroc_data_load);
  MU_ADD_TEST(test_euroc_calib_target_load);
  // MU_ADD_TEST(test_euroc_calib_load);
#endif
}
MU_RUN_TESTS(test_suite)
