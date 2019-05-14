#include "prototype/munit.hpp"
#include "prototype/calib/calib_camera_nbv.hpp"

namespace proto {

#define APRILGRID_CONF "test_data/calib/aprilgrid/target2.yaml"
#define APRILGRID_IMAGE "test_data/calib/aprilgrid/aprilgrid.png"

int test_nbv_create_aprilgrid() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // Create NBV AprilGrid
  pinhole_t pinhole;
  radtan4_t radtan;
  camera_geometry_t <pinhole_t, radtan4_t> camera{pinhole, radtan};
  mat4_t T_CF;
  aprilgrid_t aprilgrid = nbv_create_aprilgrid( target, camera, T_CF);

  return 0;
}

int test_nbv_draw_aprilgrid() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // Using EuRoC MAV dataset
  cv::Mat frame = cv::imread(APRILGRID_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  const pinhole_t pinhole{K};
  const radtan4_t radtan{D};

  // Detect AprilGrid
  aprilgrid_t grid;
  aprilgrid_set_properties(grid,
                           target.tag_rows,
                           target.tag_cols,
                           target.tag_size,
                           target.tag_spacing);
  const auto detector = aprilgrid_detector_t();
  aprilgrid_detect(grid, detector, frame, K, D);

  // Draw AprilGrid
  nbv_draw_aprilgrid(grid, pinhole, radtan, grid.T_CF, frame);

  // bool debug = true;
  bool debug = false;
  if (debug) {
    cv::imshow("AprilGrid", frame);
    cv::waitKey(0);
  }

  return 0;
}

int test_nbv_find() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // Find NBV
  aprilgrids_t aprilgrids;
  pinhole_t pinhole;
  radtan4_t radtan;
  mat4_t nbv_pose;
  aprilgrid_t nbv_grid;
  nbv_find(target, aprilgrids, pinhole, radtan, nbv_pose, nbv_grid);

  return 0;
}

int test_calib_camera_nbv() {
  calib_camera_nbv(APRILGRID_CONF);
  return 0;
}

int test_calib_camera_batch() {
  calib_camera_batch(APRILGRID_CONF);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_nbv_create_aprilgrid);
  MU_ADD_TEST(test_nbv_draw_aprilgrid);
  // MU_ADD_TEST(test_nbv_find);
  MU_ADD_TEST(test_calib_camera_nbv);
  // MU_ADD_TEST(test_calib_camera_batch);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
