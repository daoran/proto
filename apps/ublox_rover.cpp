#include <prototype/prototype.hpp>

using namespace proto;

int main() {
  ublox_t rover;
  ublox_rover_run(rover, "192.168.1.100", 8080);
  return 0;
}
