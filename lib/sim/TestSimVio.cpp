#include <gtest/gtest.h>
#include "sim/SimImu.hpp"

namespace xyz {

TEST(Sim, construct) {
  SimImu sim;
  sim.save("/tmp/sim.csv");
}

} // namespace xyz
