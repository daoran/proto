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

  printf("calib_time_cam_to_cam: %s\n", data->calib_time_cam_to_cam);
  printf("calib_time_imu_to_velo: %s\n", data->calib_time_imu_to_velo);
  printf("calib_time_velo_to_cam: %s\n", data->calib_time_velo_to_cam);
  printf("corner_dist: %f\n", data->corner_dist);
  print_double_array("S_00", data->S_00, 2);
  print_double_array("K_00", data->K_00, 9);
  print_double_array("D_00", data->D_00, 5);
  print_double_array("R_00", data->R_00, 9);
  print_double_array("T_00", data->T_00, 3);
  print_double_array("S_rect_00", data->S_rect_00, 2);
  print_double_array("R_rect_00", data->R_rect_00, 9);
  print_double_array("P_rect_00", data->P_rect_00, 12);
  printf("\n");

  print_double_array("S_01", data->S_01, 2);
  print_double_array("K_01", data->K_01, 9);
  print_double_array("D_01", data->D_01, 5);
  print_double_array("R_01", data->R_01, 9);
  print_double_array("T_01", data->T_01, 3);
  print_double_array("S_rect_01", data->S_rect_01, 2);
  print_double_array("R_rect_01", data->R_rect_01, 9);
  print_double_array("P_rect_01", data->P_rect_01, 12);
  printf("\n");

  print_double_array("S_02", data->S_02, 2);
  print_double_array("K_02", data->K_02, 9);
  print_double_array("D_02", data->D_02, 5);
  print_double_array("R_02", data->R_02, 9);
  print_double_array("T_02", data->T_02, 3);
  print_double_array("S_rect_02", data->S_rect_02, 2);
  print_double_array("R_rect_02", data->R_rect_02, 9);
  print_double_array("P_rect_02", data->P_rect_02, 12);
  printf("\n");

  print_double_array("S_03", data->S_03, 2);
  print_double_array("K_03", data->K_03, 9);
  print_double_array("D_03", data->D_03, 5);
  print_double_array("R_03", data->R_03, 9);
  print_double_array("T_03", data->T_03, 3);
  print_double_array("S_rect_03", data->S_rect_03, 2);
  print_double_array("R_rect_03", data->R_rect_03, 9);
  print_double_array("P_rect_03", data->P_rect_03, 12);
  printf("\n");

  print_double_array("R_velo_imu", data->R_velo_imu, 9);
  print_double_array("T_velo_imu", data->T_velo_imu, 3);
  printf("\n");

  print_double_array("R_cam_velo", data->R_cam_velo, 9);
  print_double_array("T_cam_velo", data->T_cam_velo, 3);
  print_double_array("delta_f", data->delta_f, 2);
  print_double_array("delta_c", data->delta_c, 2);
  printf("\n");

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
