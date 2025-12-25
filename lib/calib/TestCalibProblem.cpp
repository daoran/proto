#include <gtest/gtest.h>

#include "calib/CalibProblem.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace xyz {

TEST(CalibProblem, load) {
  CalibProblem calib{TEST_CONFIG};
  ASSERT_EQ(calib.getNumCameras(), 2);
}

} // namespace xyz
