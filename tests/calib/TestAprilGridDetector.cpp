#include <gtest/gtest.h>
#include "srl/calib/AprilGridDetector.hpp"

namespace srl {
namespace calib {

#define TEST_CONF "test_data/calib/aprilgrid/target.yaml"
#define TEST_IMAGE "test_data/calib/aprilgrid/aprilgrid.png"

static void visualize_detection(const cv::Mat &image,
                                const Mat3 &K,
                                const Vec4 &D,
                                const std::vector<Vec3> &landmarks,
                                const std::vector<Vec2> &keypoints,
                                const bool show) {
  // Undistort image
  cv::Mat image_undistorted;
  cv::Mat intrinsics = convert(K);
  cv::Mat distortions = convert(D);
  cv::undistort(image, image_undistorted, intrinsics, distortions);

  // Project landmarks in 3D to image plane
  for (size_t i = 0; i < landmarks.size(); i++) {
    // Project then scale to image plane
    Vec3 x = landmarks[i];
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
    cv::imshow("Visualize detection", image_undistorted);
    cv::waitKey(0);
  }
}

TEST(AprilGridDetector, constructor) {
  AprilGridDetector detector;

  EXPECT_FALSE(detector.configured);
  EXPECT_EQ(0, detector.tag_rows);
  EXPECT_EQ(0, detector.tag_cols);
  EXPECT_FLOAT_EQ(0.0, detector.tag_size);
  EXPECT_FLOAT_EQ(0.0, detector.tag_spacing);
  EXPECT_FALSE(detector.imshow);
}

TEST(AprilGridDetector, configure) {
  AprilGridDetector detector;

  ASSERT_EQ(0, detector.configure(TEST_CONF));
  EXPECT_TRUE(detector.configured);
  EXPECT_EQ(6, detector.tag_rows);
  EXPECT_EQ(6, detector.tag_cols);
  EXPECT_FLOAT_EQ(0.088, detector.tag_size);
  EXPECT_FLOAT_EQ(0.3, detector.tag_spacing);
}

// TEST(AprilGridDetector, calcCornerPositions) {
//   AprilGridDetector detector;
//   ASSERT_EQ(0, detector.configure(TEST_CONF));
//
//   // Detect tags
//   const cv::Mat image = cv::imread(TEST_IMAGE);
//   const cv::Mat image_gray = rgb2gray(image);
//   std::vector<AprilTags::TagDetection> tags =
//       detector.detector.extractTags(image_gray);
//
//   // Extract relative pose
//   const Mat3 K = camera_K(458.654, 457.296, 367.215, 248.375);
//   const Vec4 D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
//   const auto aprilgrid = detector.calcCornerPositions(0, tags, K, D);
//
//   // Assert
//   for (const auto corner : aprilgrid.positions_CF) {
//     EXPECT_TRUE(corner(0) < 1.0);
//     EXPECT_TRUE(corner(0) > -1.0);
//     EXPECT_TRUE(corner(1) < 1.0);
//     EXPECT_TRUE(corner(1) > -1.0);
//     EXPECT_TRUE(corner(2) < 2.0);
//     EXPECT_TRUE(corner(2) > 0.5);
//   }
//
//   // Visually inspect calculated corners by projecting the position back to the
//   // image plane
//   bool debug = false;
//   // bool debug = true;
//   // -- Get measurement
//   for (int i = 0; i < 36; i++) {
//     std::vector<Vec2> keypoints;
//     std::vector<Vec3> positions_CF;
//     Mat4 T_CF;
//     aprilgrid.getMeasurements(i, keypoints, positions_CF, T_CF);
//     visualize_detection(image, K, D, positions_CF, keypoints, debug);
//   }
// }

TEST(AprilGridDetector, detect) {
  AprilGridDetector detector;
  // detector.imshow = true;
  ASSERT_EQ(0, detector.configure(TEST_CONF));

  const cv::Mat image = cv::imread(TEST_IMAGE);
  const Mat3 K = camera_K(458.654, 457.296, 367.215, 248.375);
  const Vec4 D{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  const auto aprilgrid = detector.detect(0, image, K, D);
  if (detector.imshow) {
    cv::waitKey(0);
  }

  for (const auto corner : aprilgrid.positions_CF) {
    EXPECT_TRUE(corner(0) < 1.0);
    EXPECT_TRUE(corner(0) > -1.0);
    EXPECT_TRUE(corner(1) < 1.0);
    EXPECT_TRUE(corner(1) > -1.0);
    EXPECT_TRUE(corner(2) < 2.0);
    EXPECT_TRUE(corner(2) > 0.5);
  }
}

} // namespace calib
} // namespace srl
