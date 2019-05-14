#include <prototype/prototype.hpp>

using namespace proto;

int main() {
  ublox_t base;
  ublox_rover_run(base, "192.168.1.100", 8080);
  return 0;
}
