#include "prototype/munit.hpp"
#include "calibration/stereo_calib.hpp"

// #define TEST_DATA_PATH "test_data/chessboard"
#define TEST_DATA_PATH "/home/chutsu/Dropbox/calibration_data/stereo_data2"

namespace prototype {

int test_StereoCalib_constructor() {
  StereoCalib calib;

  return 0;
}

int test_StereoCalib_preprocessData() {
  StereoCalib calib;

  calib.preprocessData(TEST_DATA_PATH);
  calib.calibrateExtrinsics();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_StereoCalib_constructor);
  MU_ADD_TEST(test_StereoCalib_preprocessData);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
