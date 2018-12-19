#include "prototype/munit.hpp"
#include "prototype/vision/feature2d/draw.hpp"
#include "prototype/vision/feature2d/grid_good.hpp"

namespace proto {

#define TEST_IMAGE "test_data/vision/test_image.jpg"

int test_grid_good() {
  const cv::Mat image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Cannot load image [%s]!", TEST_IMAGE);
    return -1;
  }

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

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
