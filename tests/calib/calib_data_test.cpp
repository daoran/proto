#include "prototype/munit.hpp"
#include "prototype/calib/calib_data.hpp"

namespace prototype {

#define IMAGE_DIR "/data/euroc_mav/cam_april/mav0/cam0/data"
#define TARGET_FILE "test_data/calib/aprilgrid/target.yaml"

int test_process_mono_data() {
  // Setup calibration target
  calib_target_t target;
  if (calib_target_load(target, TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", TARGET_FILE);
    return -1;
  }

  // Test preprocess data
  const std::string image_dir = IMAGE_DIR;
  const vec2_t image_size{752, 480};
  const double lens_hfov = 98.0;
  const double lens_vfov = 73.0;
  const std::string output_dir = "/tmp/aprilgrid_test";
  preprocess_mono_data(target,
                       image_dir,
                       image_size,
                       lens_hfov,
                       lens_vfov,
                       output_dir);

  // Test load
  std::vector<aprilgrid_t> aprilgrids;
  int retval = load_mono_calib_data(output_dir, aprilgrids);
  MU_CHECK(retval == 0);
  MU_CHECK(aprilgrids.size() > 0);
  MU_CHECK(aprilgrids[0].ids.size() > 0);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_process_mono_data);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
