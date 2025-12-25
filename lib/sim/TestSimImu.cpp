#include <gtest/gtest.h>

#include "sim/SimImu.hpp"

namespace xyz {

TEST(SimImu, construct) {
  SimImu sim;
  sim.save("/tmp/sim.csv");
}

} // namespace xyz
