#include "feature2d/grid_fast.hpp"
#include "feature2d/util.hpp"
#include "prototype/munit.hpp"

namespace prototype {

#define TEST_DATA "test_data/kitti/raw/2011_09_26/2011_09_26_drive_0001_sync"

int test_feature_mask() {
  auto image = cv::imread(TEST_DATA "/image_00/data/0000000000.png");

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
  cv::imshow("Mask", convert(mask));
  cv::waitKey(0);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_feature_mask); }

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
