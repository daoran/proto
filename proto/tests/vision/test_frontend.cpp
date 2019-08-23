#include "proto/munit.hpp"
#include "proto/dataset/kitti.hpp"
#include "proto/vision/frontend.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "/data/kitti"

int test_frontend_update() {
  // Load kitti dataset
  kitti_raw_t dataset(TEST_DATA_BASEPATH, "2011_09_26", "0005");
  if (kitti_raw_load(dataset) != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset!");
    return -1;
  }

  // Loop through frontend
  frontend_t frontend;
  for (const auto cam0_image_path : dataset.cam0) {
    const cv::Mat image = cv::imread(cam0_image_path, cv::IMREAD_COLOR);
    frontend_update(frontend, image, true);
    if (cv::waitKey(0) == 'q') {
      return 0;
    }
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_frontend_update); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
