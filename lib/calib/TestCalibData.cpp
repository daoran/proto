#include <gtest/gtest.h>

#include "calib/CalibData.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace xyz {

TEST(CalibData, load) {
  CalibData calib_data{TEST_CONFIG};
  ASSERT_EQ(calib_data.getNumCameras(), 2);
}

} // namespace xyz
