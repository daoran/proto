#include <limits>

#include "proto/munit.hpp"
#include "proto/calib/aprilgrid.hpp"
#include "proto/vision/vision.hpp"

// #include <apriltag/apriltag.h>
// #include <apriltag/tag36h11.h>
// #include <apriltag/apriltag_math.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac/Lmeds.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

namespace proto {

#define TEST_OUTPUT "/tmp/aprilgrid.csv"
#define TEST_IMAGE "test_data/calib/aprilgrid/aprilgrid.png"
#define TEST_CONF "test_data/calib/aprilgrid/target.yaml"

static void visualize_grid(const cv::Mat &image,
                           const mat3_t &K,
                           const vec4_t &D,
                           const vec3s_t &landmarks,
                           const vec2s_t &keypoints,
                           const bool show) {
  // Undistort image
  cv::Mat image_undistorted;
  cv::Mat intrinsics = convert(K);
  cv::Mat distortions = convert(D);
  cv::undistort(image, image_undistorted, intrinsics, distortions);

  // Project landmarks in 3D to image plane
  for (size_t i = 0; i < landmarks.size(); i++) {
    // Project then scale to image plane
    vec3_t x = landmarks[i];
    x(0) = x(0) / x(2);
    x(1) = x(1) / x(2);
    x(2) = x(2) / x(2);
    x = K * x;

    // Draw corner, Set color to yellow on first corner (origin), else blue
    auto color = (i == 0) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
    cv::circle(image_undistorted, cv::Point(x(0), x(1)), 1.0, color, 2, 8);

    // Label corner
    cv::Point2f cxy(keypoints[i](0), keypoints[i](1));
    cv::Scalar text_color(0, 255, 0);
    std::string text = std::to_string(i);
    int font = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1.0;
    int thickness = 2;
    cv::putText(image_undistorted,
                text,
                cxy,
                font,
                font_scale,
                text_color,
                thickness);
  }

  if (show) {
    cv::imshow("Visualize grid", image_undistorted);
    cv::waitKey(0);
  }
}

int test_aprilgrid_constructor() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  MU_CHECK(grid.detected == false);
  MU_CHECK(grid.ids.size() == 0);
  MU_CHECK(grid.keypoints.size() == 0);

  MU_CHECK(grid.estimated == false);
  MU_CHECK(grid.points_CF.size() == 0);
  MU_CHECK((I(4) - grid.T_CF).norm() < 1e-5);

  return 0;
}

int test_aprilgrid_add() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Keypoints
  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  std::vector<cv::Point2f> keypoints;
  keypoints.emplace_back(kp1(0), kp1(1));
  keypoints.emplace_back(kp2(0), kp2(1));
  keypoints.emplace_back(kp3(0), kp3(1));
  keypoints.emplace_back(kp4(0), kp4(1));

  // Test add
  const int tag_id = 1;
  aprilgrid_add(grid, tag_id, keypoints);
  MU_CHECK_FLOAT(0.0, (grid.keypoints[0] - kp1).norm());
  MU_CHECK_FLOAT(0.0, (grid.keypoints[1] - kp2).norm());
  MU_CHECK_FLOAT(0.0, (grid.keypoints[2] - kp3).norm());
  MU_CHECK_FLOAT(0.0, (grid.keypoints[3] - kp4).norm());
  MU_CHECK(grid.points_CF.size() == 0);

  return 0;
}

int test_aprilgrid_remove() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  // Add keypoints and points_CF
  for (int i = 0; i < 5; i++) {
    std::vector<cv::Point2f> keypoints;
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);
    keypoints.emplace_back(i, i);

    vec3s_t points_CF;
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);
    points_CF.emplace_back(i, i, i);

    aprilgrid_add(grid, i, keypoints);
    extend(grid.points_CF, points_CF);
  }

  // Test remove
  aprilgrid_remove(grid, 2);
  aprilgrid_remove(grid, 4);
  aprilgrid_remove(grid, 5); // Remove non-existant id

  for (size_t i = 0; i < grid.ids.size(); i++) {
    const int id = grid.ids[i];

    MU_CHECK((vec2_t(id, id) - grid.keypoints[i * 4]).norm() < 1e-8);
    MU_CHECK((vec2_t(id, id) - grid.keypoints[i * 4 + 1]).norm() < 1e-8);
    MU_CHECK((vec2_t(id, id) - grid.keypoints[i * 4 + 2]).norm() < 1e-8);
    MU_CHECK((vec2_t(id, id) - grid.keypoints[i * 4 + 3]).norm() < 1e-8);

    MU_CHECK((vec3_t(id, id, id) - grid.points_CF[i * 4]).norm() < 1e-8);
    MU_CHECK((vec3_t(id, id, id) - grid.points_CF[i * 4 + 1]).norm() < 1e-8);
    MU_CHECK((vec3_t(id, id, id) - grid.points_CF[i * 4 + 2]).norm() < 1e-8);
    MU_CHECK((vec3_t(id, id, id) - grid.points_CF[i * 4 + 3]).norm() < 1e-8);
  }

  // Test remove everything
  for (int i = 0; i < 5; i++) {
    aprilgrid_remove(grid, i);
  }
  MU_CHECK(grid.nb_detections == 0);
  MU_CHECK(grid.ids.size() == 0);
  MU_CHECK(grid.keypoints.size() == 0);
  MU_CHECK(grid.points_CF.size() == 0);
  std::cout << grid << std::endl;

  return 0;
}

int test_aprilgrid_get() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  for (int i = 0; i < 10; i++) {
    // Keypoints
    const vec2_t kp1(i, i);
    const vec2_t kp2(i, i);
    const vec2_t kp3(i, i);
    const vec2_t kp4(i, i);
    std::vector<cv::Point2f> keypoints;
    keypoints.emplace_back(kp1(0), kp1(1));
    keypoints.emplace_back(kp2(0), kp2(1));
    keypoints.emplace_back(kp3(0), kp3(1));
    keypoints.emplace_back(kp4(0), kp4(1));

    // Positions
    const vec3_t pos1(i, i, i);
    const vec3_t pos2(i, i, i);
    const vec3_t pos3(i, i, i);
    const vec3_t pos4(i, i, i);

    // Add measurment
    aprilgrid_add(grid, i, keypoints);
    grid.estimated = true;
    grid.points_CF.emplace_back(pos1);
    grid.points_CF.emplace_back(pos2);
    grid.points_CF.emplace_back(pos3);
    grid.points_CF.emplace_back(pos4);

    // Test get tag
    vec2s_t keypoints_result;
    vec3s_t positions_result;
    aprilgrid_get(grid, i, keypoints_result, positions_result);

    MU_CHECK((vec2_t(i, i) - keypoints_result[0]).norm() < 1e-4);
    MU_CHECK((vec2_t(i, i) - keypoints_result[1]).norm() < 1e-4);
    MU_CHECK((vec2_t(i, i) - keypoints_result[2]).norm() < 1e-4);
    MU_CHECK((vec2_t(i, i) - keypoints_result[3]).norm() < 1e-4);

    MU_CHECK((vec3_t(i, i, i) - positions_result[0]).norm() < 1e-4);
    MU_CHECK((vec3_t(i, i, i) - positions_result[1]).norm() < 1e-4);
    MU_CHECK((vec3_t(i, i, i) - positions_result[2]).norm() < 1e-4);
    MU_CHECK((vec3_t(i, i, i) - positions_result[3]).norm() < 1e-4);
  }

  return 0;
}

int test_aprilgrid_grid_index() {
  int i = 0;
  int j = 0;
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  MU_CHECK(aprilgrid_grid_index(grid, 0, i, j) == 0);
  MU_CHECK(i == 0);
  MU_CHECK(j == 0);

  MU_CHECK(aprilgrid_grid_index(grid, 1, i, j) == 0);
  MU_CHECK(i == 0);
  MU_CHECK(j == 1);

  MU_CHECK(aprilgrid_grid_index(grid, 5, i, j) == 0);
  MU_CHECK(i == 0);
  MU_CHECK(j == 5);

  MU_CHECK(aprilgrid_grid_index(grid, 7, i, j) == 0);
  MU_CHECK(i == 1);
  MU_CHECK(j == 1);

  MU_CHECK(aprilgrid_grid_index(grid, 17, i, j) == 0);
  MU_CHECK(i == 2);
  MU_CHECK(j == 5);

  return 0;
}

int test_aprilgrid_calc_relative_pose() {
  // Detect tags
  const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  const auto detector = aprilgrid_detector_t();
  auto tags = detector.det.extractTags(image);

  // Extract relative pose
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  const int tag_rows = 6;
  const int tag_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  aprilgrid_t grid(0, tag_rows, tag_cols, tag_size, tag_spacing);

  for (const auto &tag : tags) {
    // Image points (counter-clockwise, from bottom left)
    std::vector<cv::Point2f> img_pts;
    img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
    img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Top left

    aprilgrid_add(grid, tag.id, img_pts);
  }

  {
    auto t = proto::tic();
    aprilgrid_calc_relative_pose(grid, K, D);
    printf("OpenCV solvePnP time elasped: %fs\n", proto::toc(&t));
    print_matrix("T_CF", grid.T_CF);
  }

  return 0;
}

int test_aprilgrid_calc_relative_pose2() {
  // Detect tags
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const cv::Mat image_gray = rgb2gray(image);
  const auto detector = aprilgrid_detector_t();
  auto tags = detector.det.extractTags(image_gray);

  // Detect tags in image
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  const int tag_rows = 6;
  const int tag_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  aprilgrid_t grid(0, tag_rows, tag_cols, tag_size, tag_spacing);

  for (const auto &tag : tags) {
    // Image points (counter-clockwise, from bottom left)
    std::vector<cv::Point2f> img_pts;
    img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
    img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Top left
    aprilgrid_add(grid, tag.id, img_pts);
  }

  // Get object points
  vec3s_t object_points;
  aprilgrid_object_points(grid, object_points);

  // OpenGV specifics
  opengv::points_t points;
  opengv::bearingVectors_t bearing_vectors;

  // -- Convert object points from Eigen to the OpenGV type
  for (const auto &obj_pt : object_points) {
    opengv::point_t p;
    p.x() = obj_pt(0);
    p.y() = obj_pt(1);
    p.z() = obj_pt(2);
    points.push_back(p);
  }

  // -- Calculate bearing vectors
  const mat3_t K_inv = K.inverse();
  for (const auto &kp : grid.keypoints) {
    vec3_t bvec = K_inv * kp.homogeneous();
    bvec.normalize();
    bearing_vectors.emplace_back(bvec(0), bvec(1), bvec(2));
  }

  // -- Solve the central absolute problem
  opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vectors,
                                                        points);

  // Solve via RANSAC
  {
    opengv::sac::Ransac<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        ransac;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        absposeproblem_ptr(
            new opengv::sac_problems::absolute_pose::
                AbsolutePoseSacProblem(adapter,
                                       opengv::sac_problems::absolute_pose::
                                           AbsolutePoseSacProblem::KNEIP));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0));
    ransac.max_iterations_ = 50;

    // Run the RANSAC experiment
    auto t = proto::tic();
    ransac.computeModel();
    mat4_t T_FC = I(4);
    T_FC.block(0, 0, 3, 3) = ransac.model_coefficients_.block(0, 0, 3, 3);
    T_FC.block(0, 3, 3, 1) = ransac.model_coefficients_.block(0, 3, 3, 1);
    printf("OpenGV RANSAC time elasped: %fs\n", proto::toc(&t));
    print_matrix("T_CF", T_FC.inverse());
  }

  // Solve via LMedS
  {
    opengv::sac::Lmeds<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        lmeds;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        absposeproblem_ptr(
            new opengv::sac_problems::absolute_pose::
                AbsolutePoseSacProblem(adapter,
                                       opengv::sac_problems::absolute_pose::
                                           AbsolutePoseSacProblem::KNEIP));
    lmeds.sac_model_ = absposeproblem_ptr;
    lmeds.threshold_ = 1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0));
    lmeds.max_iterations_ = 50;

    // Run the LMedS experiment
    auto t = proto::tic();
    lmeds.computeModel();
    mat4_t T_FC = I(4);
    T_FC.block(0, 0, 3, 3) = lmeds.model_coefficients_.block(0, 0, 3, 3);
    T_FC.block(0, 3, 3, 1) = lmeds.model_coefficients_.block(0, 3, 3, 1);
    printf("OpenGV LMeds time elasped: %fs\n", proto::toc(&t));
    print_matrix("T_CF", T_FC.inverse());
  }

  return 0;
}

int test_aprilgrid_save_and_load() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  const vec3_t pos1{1.0, 2.0, 3.0};
  const vec3_t pos2{4.0, 5.0, 6.0};
  const vec3_t pos3{7.0, 8.0, 9.0};
  const vec3_t pos4{10.0, 11.0, 12.0};

  // Test save
  grid.configured = true;
  grid.tag_rows = 6;
  grid.tag_cols = 6;
  grid.tag_size = 0.088;
  grid.tag_spacing = 0.3;

  grid.detected = true;
  grid.timestamp = 1544020482626424074;
  grid.ids.push_back(1);
  grid.keypoints = {kp1, kp2, kp3, kp4};

  grid.estimated = true;
  grid.points_CF = {pos1, pos2, pos3, pos4};
  grid.T_CF = I(4);
  grid.T_CF.block(0, 3, 3, 1) = vec3_t{1.0, 2.0, 3.0};
  MU_CHECK(aprilgrid_save(grid, TEST_OUTPUT) == 0);

  // Test load
  aprilgrid_t grid2(0, 6, 6, 0.088, 0.3);
  MU_CHECK(aprilgrid_load(grid2, TEST_OUTPUT) == 0);
  std::cout << grid2 << std::endl;

  MU_CHECK(grid2.ids.size() == 1);
  MU_CHECK(grid2.keypoints.size() == 4);
  MU_CHECK(grid2.points_CF.size() == 4);
  MU_CHECK(grid.timestamp == grid2.timestamp);
  MU_CHECK_FLOAT(0.0, (grid2.keypoints[0] - kp1).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints[1] - kp2).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints[2] - kp3).norm());
  MU_CHECK_FLOAT(0.0, (grid2.keypoints[3] - kp4).norm());
  MU_CHECK_FLOAT(0.0, (grid2.points_CF[0] - pos1).norm());
  MU_CHECK_FLOAT(0.0, (grid2.points_CF[1] - pos2).norm());
  MU_CHECK_FLOAT(0.0, (grid2.points_CF[2] - pos3).norm());
  MU_CHECK_FLOAT(0.0, (grid2.points_CF[3] - pos4).norm());

  const auto diff = grid2.T_CF.block(0, 3, 3, 1) - vec3_t{1.0, 2.0, 3.0};
  MU_CHECK((diff).norm() < 1e-5);

  return 0;
}

int test_aprilgrid_print() {
  aprilgrid_t grid(0, 6, 6, 0.088, 0.3);

  const vec2_t kp1{1.0, 2.0};
  const vec2_t kp2{3.0, 4.0};
  const vec2_t kp3{5.0, 6.0};
  const vec2_t kp4{7.0, 8.0};
  const vec3_t pos1{1.0, 2.0, 3.0};
  const vec3_t pos2{4.0, 5.0, 6.0};
  const vec3_t pos3{7.0, 8.0, 9.0};
  const vec3_t pos4{10.0, 11.0, 12.0};

  grid.ids.push_back(1);
  grid.keypoints.push_back(kp1);
  grid.keypoints.push_back(kp2);
  grid.keypoints.push_back(kp3);
  grid.keypoints.push_back(kp4);
  grid.points_CF.push_back(pos1);
  grid.points_CF.push_back(pos2);
  grid.points_CF.push_back(pos3);
  grid.points_CF.push_back(pos4);

  std::cout << grid << std::endl;

  return 0;
}

int test_aprilgrid_detect() {
  aprilgrid_t grid;
  MU_CHECK(aprilgrid_configure(grid, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t();
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid, detector, image, K, D);

  for (const auto corner : grid.points_CF) {
    MU_CHECK(corner(0) < 1.0);
    MU_CHECK(corner(0) > -1.0);
    MU_CHECK(corner(1) < 1.0);
    MU_CHECK(corner(1) > -1.0);
    MU_CHECK(corner(2) < 2.0);
    MU_CHECK(corner(2) > 0.5);
  }

  return 0;
}

// int test_aprilgrid_detect2() {
//   // aprilgrid_t grid;
//   // MU_CHECK(aprilgrid_configure(grid, TEST_CONF) == 0);
//
//   const auto detector = aprilgrid_detector_t();
//   const cv::Mat image = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
//   // const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
//   // const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
//
//   apriltag_detector_t *td = apriltag_detector_create();
// 	td->debug = 1;
//   apriltag_family_t *tf = tag36h11_create();
// 	// tf->width_at_border = 4;
// 	tf->total_width = 6;
// 	// tf->reversed_border = true;
//   apriltag_detector_add_family(td, tf);
//
// 	image_u8_t im = {
// 		.width = image.cols,
// 		.height = image.rows,
// 		.stride = image.cols,
// 		.buf = image.data
// 	};
// 	// zarray_t *detections = apriltag_detector_detect(td, &im);
// 	// for (int i = 0; i < zarray_size(detections); i++) {
// 	// 	apriltag_detection_t *det;
// 	// 	zarray_get(detections, i, &det);
// 	// 	// Do something with det here
// 	// }
// 	// printf("detected: %d\n", zarray_size(detections));
// 	zarray_t *quads = apriltag_quad_thresh(td, &im);
// 	printf("quads: %d\n", zarray_size(quads));
// 	apriltag_detections_destroy(quads);
//
// 	// apriltag_detections_destroy(detections);
// 	tag36h11_destroy(tf);
// 	apriltag_detector_destroy(td);
//
// 	// cv::imshow("image", image);
// 	// cv::waitKey(0);
//
//
//   return 0;
// }

int test_aprilgrid_intersection() {
  aprilgrid_t grid0;
  aprilgrid_t grid1;
  MU_CHECK(aprilgrid_configure(grid0, TEST_CONF) == 0);
  MU_CHECK(aprilgrid_configure(grid1, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t();
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid0, detector, image, K, D);
  aprilgrid_detect(grid1, detector, image, K, D);

  // Remove tag id 2 from grid1
  aprilgrid_remove(grid1, 2);
  // std::cout << grid0 << std::endl;
  // std::cout << grid1 << std::endl;
  MU_CHECK(grid0.nb_detections == 36);
  MU_CHECK(grid1.nb_detections == 35);
  MU_CHECK(grid0.ids.size() == 36);
  MU_CHECK(grid1.ids.size() == 35);

  // Test intersection
  aprilgrid_intersection(grid0, grid1);
  MU_CHECK(grid0.nb_detections == 35);
  MU_CHECK(grid1.nb_detections == 35);
  MU_CHECK(grid0.ids.size() == 35);
  MU_CHECK(grid1.ids.size() == 35);

  return 0;
}

int test_aprilgrid_intersection2() {
  aprilgrid_t grid0;
  aprilgrid_t grid1;
  aprilgrid_t grid2;
  MU_CHECK(aprilgrid_configure(grid0, TEST_CONF) == 0);
  MU_CHECK(aprilgrid_configure(grid1, TEST_CONF) == 0);
  MU_CHECK(aprilgrid_configure(grid2, TEST_CONF) == 0);

  const auto detector = aprilgrid_detector_t();
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const mat3_t K = pinhole_K(458.654, 457.296, 367.215, 248.375);
  const vec4_t D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  aprilgrid_detect(grid0, detector, image, K, D);
  aprilgrid_detect(grid1, detector, image, K, D);
  aprilgrid_detect(grid2, detector, image, K, D);

  // Randomly remove between 2 to 10 tags from grid0, grid1 and grid2
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid0, randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid1, randi(0, 35));
  }
  for (int i = 0; i < randi(2, 10); i++) {
    aprilgrid_remove(grid2, randi(0, 35));
  }
  MU_CHECK(grid0.nb_detections < 36);
  MU_CHECK(grid1.nb_detections < 36);
  MU_CHECK(grid2.nb_detections < 36);

  // Test intersection
  std::vector<aprilgrid_t *> data = {&grid0, &grid1, &grid2};
  aprilgrid_intersection(data);
  const int nb_detections = grid0.nb_detections;
  MU_CHECK(grid0.nb_detections == nb_detections);
  MU_CHECK(grid1.nb_detections == nb_detections);
  MU_CHECK(grid2.nb_detections == nb_detections);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_aprilgrid_constructor);
  MU_ADD_TEST(test_aprilgrid_add);
  MU_ADD_TEST(test_aprilgrid_remove);
  MU_ADD_TEST(test_aprilgrid_get);
  MU_ADD_TEST(test_aprilgrid_grid_index);
  MU_ADD_TEST(test_aprilgrid_calc_relative_pose);
  MU_ADD_TEST(test_aprilgrid_calc_relative_pose2);
  MU_ADD_TEST(test_aprilgrid_save_and_load);
  MU_ADD_TEST(test_aprilgrid_print);
  MU_ADD_TEST(test_aprilgrid_detect);
  // MU_ADD_TEST(test_aprilgrid_detect2);
  MU_ADD_TEST(test_aprilgrid_intersection);
  MU_ADD_TEST(test_aprilgrid_intersection2);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
