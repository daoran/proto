#include <prototype/prototype.hpp>

using namespace proto;

int main() {
  ublox_t base;
  if (ublox_connect(base) != 0) {
    FATAL("Failed to connect to ublox!");
  }

  base.nav_pvt_cb = [](ublox_t &base) {
    const ubx_nav_pvt_t msg{base.ubx_parser.msg};
    print_ubx_nav_pvt(msg);
  };

  LOG_INFO("Ublox base station is running!");
  ublox_base_station_run(base);
  return 0;
}
