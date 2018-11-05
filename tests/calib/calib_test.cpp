#include "prototype/calib/calib.hpp"
#include "prototype/munit.hpp"

namespace prototype {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define TARGET_CONF "test_data/calib/aprilgrid/target.yaml"
#define TARGET_IMAGE "test_data/calib/aprilgrid/aprilgrid.png"

int test_preprocess_and_load_camera_data() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, TARGET_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", TARGET_CONF);
    return -1;
  }

  // Test preprocess data
  const std::string image_dir = IMAGE_DIR;
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const std::string output_dir = "/tmp/aprilgrid_test";
  preprocess_camera_data(target,
                         image_dir,
                         image_size,
                         lens_hfov,
                         lens_vfov,
                         output_dir);

  // Test load
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_camera_calib_data(output_dir, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  return 0;
}

int test_draw_calib_validation() {
  // Setup camera geometry
  // -- Camera model
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const pinhole_t camera_model{fx, fy, cx, cy};
  const mat3_t cam_K = camera_model.K;
  // -- Distortion model
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;
  const radtan4_t distortion_model{k1, k2, p1, p2};
  const vec4_t cam_D{k1, k2, p1, p2};
  // -- Camera Geometry
  const camera_geometry_t<pinhole_t, radtan4_t>
    camera{camera_model, distortion_model};

  // Load aprilgrid
  aprilgrid_t aprilgrid;
  if (aprilgrid_configure(aprilgrid, TARGET_CONF) != 0) {
    LOG_ERROR("Failed to configure aprilgrid!");
    return -1;
  }

  // Detect AprilGrid
  const cv::Mat image = cv::imread(TARGET_IMAGE);
  aprilgrid_detect(aprilgrid, image, cam_K, cam_D);

  // Get measured keypoints and project points
  std::vector<vec2_t> measured;
  std::vector<vec2_t> projected;

  for (size_t i = 0; i < aprilgrid.keypoints.size(); i++) {
    const auto &kp = aprilgrid.keypoints[i];
    measured.emplace_back(kp);

    const auto &point = aprilgrid.points_CF[i];
    const auto p = camera_geometry_project(camera, point);
    projected.emplace_back(p);
  }

  // Draw validation
  const cv::Scalar measured_color{0, 0, 255};  // Red
  const cv::Scalar projected_color{0, 255, 0};  // Green
  const auto validation =  draw_calib_validation(image,
                                                 measured,
                                                 projected,
                                                 measured_color,
                                                 projected_color);

  cv::imshow("Validation:", validation);
  cv::waitKey(0);

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_preprocess_and_load_camera_data);
  MU_ADD_TEST(test_draw_calib_validation);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
