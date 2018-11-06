#include "prototype/munit.hpp"
#include "prototype/calib/calib.hpp"
#include "prototype/calib/calib_camera.hpp"

namespace prototype {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF "test_data/calib/aprilgrid/target.yaml"

int test_calib_camera_solve() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // // Test preprocess data
  // const std::string image_dir = IMAGE_DIR;
  // const vec2_t image_size{752, 480};
  // const double lens_hfov = 98.0;
  // const double lens_vfov = 73.0;
  const std::string output_dir = "/tmp/aprilgrid_test";
  // preprocess_camera_data(target,
  //                        image_dir,
  //                        image_size,
  //                        lens_hfov,
  //                        lens_vfov,
  //                        output_dir);

  // Test load
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_camera_calib_data(output_dir, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const double fx = pinhole_focal_length(image_size(0), lens_hfov);
  const double fy = pinhole_focal_length(image_size(1), lens_vfov);
  const double cx = image_size(0) / 2.0;
  const double cy = image_size(1) / 2.0;
  pinhole_t pinhole{fx, fy, cx, cy};
  radtan4_t radtan{0.01, 0.0001, 0.0001, 0.0001};

  const auto grid = aprilgrids[0];
  const auto tag_id = grid.ids[0];
  const int corner_id = 0;
  const auto kp = grid.keypoints[0];
  const auto T_CF = grid.T_CF;

  // vec3_t object_point;
  // if (aprilgrid_object_point(grid, tag_id, corner_id, object_point) != 0) {
  //   LOG_ERROR("Failed to calculate AprilGrid object point!");
  // }
  //
  // const double intrinsics[4] = {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
  // const double distortion[4] = {radtan.k1, radtan.k2, radtan.p1, radtan.p2};
  // const quat_t q_CF{T_CF.block<3, 3>(0, 0)};
  // const vec3_t t_CF{T_CF.block<3, 1>(0, 3)};
  // const double q_CF_data[4] = {q_CF.w(), q_CF.x(), q_CF.y(), q_CF.z()};
  // const double t_CF_data[3] = {t_CF(0), t_CF(1), t_CF(2)};
  // double result[2] = {0.0, 0.0};
  // const pinhole_radtan4_residual_t residual{kp, object_point};
  //
  // residual(intrinsics, distortion, q_CF_data, t_CF_data, result);

  calib_camera_solve(aprilgrids, pinhole, radtan);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_calib_camera_solve);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
