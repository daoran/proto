#include "prototype/munit.hpp"
#include "prototype/vision/feature2d/draw.hpp"
#include "prototype/vision/feature2d/grid_good.hpp"

namespace prototype {

#define TEST_IMAGE_CENTER "test_data/apriltag/center.png"
#define TEST_IMAGE "test_data/euroc/cam0/data/1403715273262142976.png"

int test_grid_good() {
  const cv::Mat image = cv::imread(TEST_IMAGE_CENTER);

  auto features = grid_good(image, // Input image
                            1000,  // Max number of corners
                            5,     // Grid rows
                            5);    // Grid columns

  bool debug = false;
  if (debug) {
    auto out_image = draw_grid_features(image, 5, 5, features);
    cv::imshow("Grid Features", out_image);
    cv::waitKey(0);
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_grid_good); }

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
