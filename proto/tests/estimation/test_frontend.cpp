#include "proto/munit.hpp"
#include "proto/core/core.hpp"
#include "proto/dataset/kitti.hpp"
#include "proto/estimation/frontend.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "/data/kitti"

int test_frontend_detect() {

  return 0;
}

int test_frontend_track() {

  return 0;
}

int test_frontend_update() {
  // Load kitti dataset
  // kitti_raw_t dataset(TEST_DATA_BASEPATH, "2011_09_26", "0005");
  // if (kitti_raw_load(dataset) != 0) {
  //   LOG_ERROR("Failed to load KITTI raw dataset!");
  //   return -1;
  // }

  std::vector<std::string> results;
  list_dir("/home/chutsu/Downloads/mav0/cam0/data", results);
  std::sort(results.begin(), results.end());

  // Loop through frontend
  frontend_t frontend;
  profiler_t profiler;
  profiler.start("tracking");
  size_t nb_images = 0;
  bool debug = true;

  // for (const auto cam0_image_path : dataset.cam0) {
  for (auto cam0_image_path : results) {
    cam0_image_path = "/home/chutsu/Downloads/mav0/cam0/data/" + cam0_image_path;
    const auto image = cv::imread(cam0_image_path, cv::IMREAD_COLOR);
    const auto image_gray = rgb2gray(image);

    frontend.update(image_gray, debug);
    if (debug) {
      if (cv::waitKey(50) == 'q') {
        return 0;
      }
    }
    printf("tracked: %zu  ", frontend.features.tracking.size());
    printf("total: %zu\n", frontend.features.tracking.size() + frontend.features.lost.size());
    nb_images++;
  }
  printf("seconds per image: %f\n", profiler.stop("tracking") / nb_images);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_frontend_detect);
  MU_ADD_TEST(test_frontend_track);
  MU_ADD_TEST(test_frontend_update);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
