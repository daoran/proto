#include "prototype/munit.hpp"
#include "prototype/vision/vision_common.hpp"
#include "prototype/vision/feature2d/grid_fast.hpp"

namespace proto {

#define TEST_IMAGE "test_data/vision/test_image.jpg"

int test_feature_mask() {
  const auto image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Cannot load image [%s]!", TEST_IMAGE);
    return -1;
  }

  const int image_width = image.cols;
  const int image_height = image.rows;
  auto keypoints = grid_fast(image, 100, 5, 5, 30);
  std::vector<cv::Point2f> points;
  for (auto kp : keypoints) {
    points.emplace_back(kp.pt);
  }
  points.emplace_back(0, 0);
  points.emplace_back(image_height, image_width);
  points.emplace_back(0, image_width);
  points.emplace_back(image_height, 0);

  auto mask = feature_mask(image_width, image_height, points, 4);
  const bool debug = false;
  if (debug) {
    cv::imshow("Mask", convert(mask));
    cv::waitKey(0);
  }

  return 0;
}

// int test_laplacian_blurr_measure() {
//   const auto image = cv::imread(TEST_IMAGE);
//   if (image.empty()) {
//     LOG_ERROR("Cannot load image [%s]!", TEST_IMAGE);
//     return -1;
//   }
//
//   double score = laplacian_blur_measure(image);
//   std::cout << score << std::endl;
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_feature_mask);
  // MU_ADD_TEST(test_laplacian_blurr_measure);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
