#include <proto/proto.hpp>
#include <map>

using namespace proto;

static FILE *setup_nav_hpposllh_output_file() {
  FILE *fp = fopen("./ublox_nav_hpposllh.csv", "w");
  fprintf(fp, "itow,");
  fprintf(fp, "lon,");
  fprintf(fp, "lat,");
  fprintf(fp, "height,");
  fprintf(fp, "hmsl,");
  fprintf(fp, "lon_hp,");
  fprintf(fp, "lat_hp,");
  fprintf(fp, "height_hp,");
  fprintf(fp, "hmsl_hp,");
  fprintf(fp, "hacc,");
  fprintf(fp, "vacc\n");
  return fp;
}

static void record_ubx_nav_hpposllh_msg(FILE *fp, const ubx_msg_t &msg_data) {
  if (fp) {
    const ubx_nav_hpposllh_t msg{msg_data};
    fprintf(fp, "%d,", msg.itow);
    fprintf(fp, "%d,", msg.lon);
    fprintf(fp, "%d,", msg.lat);
    fprintf(fp, "%d,", msg.height);
    fprintf(fp, "%d,", msg.hmsl);
    fprintf(fp, "%d,", msg.lon_hp);
    fprintf(fp, "%d,", msg.lat_hp);
    fprintf(fp, "%d,", msg.height_hp);
    fprintf(fp, "%d,", msg.hmsl_hp);
    fprintf(fp, "%d,", msg.hacc);
    fprintf(fp, "%d\n", msg.vacc);
    fflush(fp);
  }
}

static void setup_rover_callbacks(
    ublox_t &rover,
    FILE *hpposllh_fp,
    std::map<std::string, ubx_msg_t> msg_registry) {

  rover.nav_svin_cb = [&msg_registry](ublox_t &rover) {
    msg_registry["UBX_NAV_SVIN"] = rover.ubx_parser.msg;
  };

  rover.nav_status_cb = [&msg_registry](ublox_t &rover) {
    msg_registry["UBX_NAV_STATUS"] = rover.ubx_parser.msg;
  };

  rover.nav_pvt_cb = [&msg_registry](ublox_t &rover) {
    msg_registry["UBX_NAV_PVT"] = rover.ubx_parser.msg;
  };

  rover.nav_hpposllh_cb = [hpposllh_fp, &msg_registry](ublox_t &rover) {
    const ubx_msg_t msg_data = rover.ubx_parser.msg;
    record_ubx_nav_hpposllh_msg(hpposllh_fp, msg_data);
    msg_registry["UBX_NAV_HPPOSLLH"] = msg_data;
  };

  rover.rxm_rtcm_cb = [&msg_registry](ublox_t &rover) {
    msg_registry["UBX_RXM_RTCM"] = rover.ubx_parser.msg;
  };
}

int main() {
  ublox_t rover;
  std::map<std::string, ubx_msg_t> msg_registry;

  // Connect to ublox
  if (ublox_connect(rover) != 0) {
    FATAL("Failed to connect to ublox!");
  }

  // Setup ublox rover callbacks
  FILE *hpposllh_fp = setup_nav_hpposllh_output_file();
  setup_rover_callbacks(rover, hpposllh_fp, msg_registry);

  // Run ublox in rover mode
  ublox_rover_run(rover, "192.168.1.100", 8080);

  // Clean up
  fclose(hpposllh_fp);
  return 0;
}
