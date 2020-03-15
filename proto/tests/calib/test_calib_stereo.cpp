#include "proto/munit.hpp"
#include "proto/calib/calib.hpp"
#include "proto/calib/calib_stereo.hpp"

namespace proto {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define APRILGRID_CONF "test_data/calib/aprilgrid/target.yaml"

void test_setup() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, APRILGRID_CONF) != 0) {
    FATAL("Failed to load calib target [%s]!", APRILGRID_CONF);
  }
}

void test_suite() {
  test_setup();
  MU_ADD_TEST(test_stereo_residual);
  MU_ADD_TEST(test_calib_stereo_solve);
  // MU_ADD_TEST(test_calib_stereo_stats);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
