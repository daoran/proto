#include "prototype/munit.hpp"
#include "prototype/mav/lz.hpp"

namespace proto {

#define TEST_CONFIG "test_data/mav/lz.yaml"

int test_lz_detector_detector_contrustor() {
  {
    const std::vector<int> tag_ids{1, 2};
    const std::vector<double> tag_sizes{0.1, 0.2};
    lz_detector_t lz_detector{tag_ids, tag_sizes};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }
  {
    lz_detector_t lz_detector{TEST_CONFIG};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }

  return 0;
}

int test_lz_detector_detector_configure() {
  {
    const std::vector<int> tag_ids{1, 2};
    const std::vector<double> tag_sizes{0.1, 0.2};
    lz_detector_t lz_detector{tag_ids, tag_sizes};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }
  {
    lz_detector_t lz_detector{TEST_CONFIG};
    MU_CHECK(fltcmp(lz_detector.targets[1], 0.1) == 0);
    MU_CHECK(fltcmp(lz_detector.targets[2], 0.2) == 0);
  }

  return 0;
}

int test_lz_detector_detect() {
  const std::vector<int> tag_ids{1, 2};
  const std::vector<double> tag_sizes{0.1, 0.2};
  const lz_detector_t lz_detector{tag_ids, tag_sizes};

  cv::Mat image;
  const double fx = pinhole_focal_length(image.cols, 90.0);
  const double fy = pinhole_focal_length(image.rows, 90.0);
  const double cx = image.cols / 2.0;
  const double cy = image.rows / 2.0;
  const pinhole_t pinhole{fx, fy, cx, cy};
  proto::mat4_t T_CZ;
  // lz_detector_detect(lz, image, pinhole, T_CZ);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_lz_detector_detector_contrustor);
  MU_ADD_TEST(test_lz_detector_detector_configure);
  MU_ADD_TEST(test_lz_detector_detect);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
