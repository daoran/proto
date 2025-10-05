#include <gtest/gtest.h>
#include "sim/Sim.hpp"

namespace xyz {

TEST(Sim, construct) {
  Sim sim;
  sim.save("/tmp/sim.csv");
}

} // namespace xyz
