#include <prototype/prototype.hpp>

using namespace proto;

int main() {
  ublox_t base;
  if (ublox_connect(base) != 0) {
    FATAL("Failed to connect to ublox!");
  }
  ublox_base_station_run(base);
  return 0;
}
