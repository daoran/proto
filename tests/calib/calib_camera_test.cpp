#include "prototype/munit.hpp"
#include "prototype/calib/calib.hpp"
#include "prototype/calib/calib_camera.hpp"

namespace prototype {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF "test_data/calib/aprilgrid/target.yaml"
#define APRILGRID_DATA "/tmp/aprilgrid_test"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
  }

  // Test preprocess data
  const std::string image_dir = IMAGE_DIR;
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  int retval = preprocess_camera_data(target,
                                      image_dir,
                                      image_size,
                                      lens_hfov,
                                      lens_vfov,
                                      APRILGRID_DATA);
  if (retval == -1) {
    FATAL("Failed to preprocess camera data!");
  }
}

int test_pinhole_radtan4_residual() {
  // Test load
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup intrinsic and distortion initialization
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const double fx = pinhole_focal_length(image_size(0), lens_hfov);
  const double fy = pinhole_focal_length(image_size(1), lens_vfov);
  const double cx = image_size(0) / 2.0;
  const double cy = image_size(1) / 2.0;
  const double intrinsics[4] = {fx, fy, cx, cy};
  const double distortion[4] = {0.01, 0.0001, 0.0001, 0.0001};

  for (size_t i = 0; i < aprilgrids.size(); i++) {
    // Setup
    const auto grid = aprilgrids[i];
    const auto tag_id = grid.ids[0];
    const int corner_id = 0;
    const auto kp = grid.keypoints[0];
    const auto T_CF = grid.T_CF;

    // Get object point
    vec3_t object_point;
    if (aprilgrid_object_point(grid, tag_id, corner_id, object_point) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object point!");
    }

    // Form residual and call the functor
    const quat_t q_CF{T_CF.block<3, 3>(0, 0)};
    const vec3_t t_CF{T_CF.block<3, 1>(0, 3)};
    const double q_CF_data[4] = {q_CF.x(), q_CF.y(), q_CF.z(), q_CF.w()};
    const double t_CF_data[3] = {t_CF(0), t_CF(1), t_CF(2)};
    double result[2] = {0.0, 0.0};
    const pinhole_radtan4_residual_t residual{kp, object_point};
    residual(intrinsics, distortion, q_CF_data, t_CF_data, result);

    // Just some arbitrary test to make sure reprojection error is not larger
    // than 100pixels in x or y direction. But often this can be the case ...
    MU_CHECK(result[0] < 100.0);
    MU_CHECK(result[1] < 100.0);
  }

  return 0;
}

int test_calib_camera_solve() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const double fx = pinhole_focal_length(image_size(0), lens_hfov);
  const double fy = pinhole_focal_length(image_size(1), lens_vfov);
  const double cx = image_size(0) / 2.0;
  const double cy = image_size(1) / 2.0;
  pinhole_t pinhole{fx, fy, cx, cy};
  radtan4_t radtan{0.01, 0.0001, 0.0001, 0.0001};

  // Test
  std::vector<mat4_t> poses;
  MU_CHECK_EQ(0, calib_camera_solve(aprilgrids, pinhole, radtan, poses));
  MU_CHECK_EQ(aprilgrids.size(), poses.size());

  // Show results
  std::cout << "Optimized intrinsics and distortions:" << std::endl;
  std::cout << pinhole << std::endl;
  std::cout << radtan << std::endl;

  return 0;
}

int test_calib_camera_stats() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  // Setup camera intrinsics and distortion
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const double fx = pinhole_focal_length(image_size(0), lens_hfov);
  const double fy = pinhole_focal_length(image_size(1), lens_vfov);
  const double cx = image_size(0) / 2.0;
  const double cy = image_size(1) / 2.0;
  pinhole_t pinhole{fx, fy, cx, cy};
  radtan4_t radtan{0.01, 0.0001, 0.0001, 0.0001};

  // Test
  std::vector<mat4_t> poses;
  MU_CHECK_EQ(0, calib_camera_solve(aprilgrids, pinhole, radtan, poses));
  MU_CHECK_EQ(aprilgrids.size(), poses.size());

  const double intrinsics[4] = {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
  const double distortion[4] = {radtan.k1, radtan.k2, radtan.p1, radtan.p2};
  calib_camera_stats<pinhole_radtan4_residual_t>(aprilgrids,
                                                 intrinsics,
                                                 distortion,
                                                 poses,
                                                 "");

  return 0;
}

void test_suite() {
  test_setup();
  MU_ADD_TEST(test_pinhole_radtan4_residual);
  MU_ADD_TEST(test_calib_camera_solve);
  MU_ADD_TEST(test_calib_camera_stats);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
