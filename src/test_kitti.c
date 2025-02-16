#include "munit.h"
#include "xyz_kitti.h"

#define TEST_DATA "/data/kitti_raw/2011_09_26"
#define TEST_SEQ_NAME "2011_09_26_drive_0001_sync"
#define TEST_SEQ TEST_DATA "/" TEST_SEQ_NAME

int test_kitti_camera_load(void) {
  const char *data_dir = TEST_SEQ "/image_00";
  kitti_camera_t *data = kitti_camera_load(data_dir);
  kitti_camera_free(data);
  return 0;
}

int test_kitti_oxts_load(void) {
  const char *data_dir = TEST_SEQ "/oxts";
  kitti_oxts_t *data = kitti_oxts_load(data_dir);
  kitti_oxts_free(data);
  return 0;
}

int test_kitti_velodyne_load(void) {
  const char *data_dir = TEST_SEQ "/velodyne_points";
  kitti_velodyne_t *data = kitti_velodyne_load(data_dir);
  kitti_velodyne_free(data);
  return 0;
}

int test_kitti_calib_load(void) {
  const char *data_dir = TEST_DATA;
  kitti_calib_t *data = kitti_calib_load(data_dir);
  // kitti_calib_print(data);
  kitti_calib_free(data);
  return 0;
}

int test_kitti_raw_load(void) {
  kitti_raw_t *data = kitti_raw_load(TEST_DATA, TEST_SEQ_NAME);
  kitti_raw_free(data);
  return 0;
}

void test_suite(void) {
#if CI_MODE == 0
  MU_ADD_TEST(test_kitti_camera_load);
  MU_ADD_TEST(test_kitti_oxts_load);
  MU_ADD_TEST(test_kitti_velodyne_load);
  MU_ADD_TEST(test_kitti_calib_load);
  MU_ADD_TEST(test_kitti_raw_load);
#endif
}
MU_RUN_TESTS(test_suite)
