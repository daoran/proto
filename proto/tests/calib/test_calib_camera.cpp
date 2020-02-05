#include "proto/munit.hpp"
#include "proto/calib/calib.hpp"
#include "proto/calib/calib_camera.hpp"

namespace proto {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF "test_data/calib/aprilgrid/target2.yaml"
#define APRILGRID_DATA "/tmp/aprilgrid_test/mono/cam0"
#define APRILGRID_IMAGE "test_data/calib/aprilgrid/aprilgrid.png"

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
  std::vector<timestamp_t> timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
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
  std::vector<timestamp_t> timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
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
  mat4s_t poses;
  MU_CHECK(calib_camera_solve(aprilgrids, pinhole, radtan, poses) == 0);
  MU_CHECK(aprilgrids.size() == poses.size());

  // Show results
  std::cout << "Optimized intrinsics and distortions:" << std::endl;
  std::cout << pinhole << std::endl;
  std::cout << radtan << std::endl;

  return 0;
}

int test_calib_camera_stats() {
  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  std::vector<timestamp_t> timestamps;
  int retval = load_camera_calib_data(APRILGRID_DATA, aprilgrids, timestamps);
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
  mat4s_t T_CF;
  MU_CHECK(calib_camera_solve(aprilgrids, pinhole, radtan, T_CF) == 0);
  MU_CHECK(aprilgrids.size() == T_CF.size());

  calib_camera_stats<pinhole_radtan4_residual_t>(aprilgrids,
                                                 *pinhole.data,
                                                 *radtan.data,
                                                 T_CF,
                                                 "");

  return 0;
}

int test_calib_generate_poses() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", APRILGRID_CONF);
    return -1;
  }

  // Generate nbv poses
  mat4s_t nbv_poses = calib_generate_poses(target);

  // Setup camera
  cv::VideoCapture camera(0);
  if (camera.isOpened() == false) {
    return -1;
  }
  sleep(2);

  // Guess the camera intrinsics and distortion
  cv::Mat frame;
  camera.read(frame);
  const auto detector = aprilgrid_detector_t();
  const double fx = pinhole_focal_length(frame.cols, 120.0);
  const double fy = pinhole_focal_length(frame.rows, 120.0);
  const double cx = frame.cols / 2.0;
  const double cy = frame.rows / 2.0;
  const mat3_t K = pinhole_K(fx, fy, cx, cy);
  const vec4_t D = zeros(4, 1);

  // Loop camera feed
  int pose_idx = randi(0, nbv_poses.size());
  while (true) {
    // Get image
    cv::Mat frame;
    camera.read(frame);

    // Detect AprilGrid
    aprilgrid_t grid;
    aprilgrid_set_properties(grid,
                             target.tag_rows,
                             target.tag_cols,
                             target.tag_size,
                             target.tag_spacing);
    aprilgrid_detect(grid, detector, frame, K, D);

    // Calculate calibration target from camera view
    vec3s_t object_points;
    aprilgrid_object_points(grid, object_points);
    const size_t nb_pts = object_points.size();
    const matx_t hp_T = vecs2mat(object_points);
    const mat4_t T_TC = nbv_poses[pose_idx];
    const mat4_t T_CT = T_TC.inverse();
    const matx_t hp_C = T_CT * hp_T;
    const matx_t p_C = hp_C.block(0, 0, 3, nb_pts);

    // Project target corners to camera frame
    for (size_t i = 0; i < nb_pts; i++) {
      const vec3_t p = p_C.block(0, i, 3, 1);
      const vec3_t pt{p(0) / p(2), p(1) / p(2), 1.0};
      const vec2_t pixel = (K * pt).head(2);
      cv::Point2f cv_pixel(pixel(0), pixel(1));
      if (i < 4) {
        cv::circle(frame, cv_pixel, 3, cv::Scalar(0, 255, 0), -1);
      } else {
        cv::circle(frame, cv_pixel, 3, cv::Scalar(0, 0, 255), -1);
      }
    }

    const vec3_t pos_desired = tf_trans(T_CT);
    const vec3_t pos_actual = tf_trans(grid.T_CF);
    const double pos_diff = (pos_desired - pos_actual).norm();
    bool pos_ok = (pos_diff < 0.25) ? true : false;

    const vec3_t rpy_desired = quat2euler(tf_quat(T_CT));
    const vec3_t rpy_actual = quat2euler(tf_quat(grid.T_CF));
    const double rpy_diff = (rpy_desired - rpy_actual).norm();
    bool rpy_ok = (rad2deg(rpy_diff) < 10.0) ? true : false;

    printf("pos diff [%.2f]\t rpy_diff [%.2f]\n", pos_diff, rpy_diff);
    if (pos_ok && rpy_ok) {
      pose_idx = randf(0, nbv_poses.size());
    }

    // const std::string title = "AprilGrid";
    // aprilgrid_imshow(grid, title, frame);

    // Show image and get user input
    cv::Mat frame_flip;
    cv::flip(frame, frame_flip, 1);
    cv::imshow("Image", frame_flip);
    char key = (char) cv::waitKey(1);
    if (key == 'q') {
      break;
    }
  }

  return 0;
}

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
  camera_geometry_t<pinhole_t, radtan4_t> camera{pinhole, radtan};
  vec2_t resolution{640, 480};
  mat4_t T_CF;
  aprilgrid_t aprilgrid =
      nbv_create_aprilgrid(target, camera, resolution, T_CF);

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
  test_setup();
  MU_ADD_TEST(test_pinhole_radtan4_residual);
  MU_ADD_TEST(test_calib_generate_poses);
  MU_ADD_TEST(test_calib_camera_stats);
  MU_ADD_TEST(test_calib_camera_solve);

  MU_ADD_TEST(test_nbv_create_aprilgrid);
  MU_ADD_TEST(test_nbv_draw_aprilgrid);
  // MU_ADD_TEST(test_nbv_find);
  MU_ADD_TEST(test_calib_camera_nbv);
  // MU_ADD_TEST(test_calib_camera_batch);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
