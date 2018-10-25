#include "prototype/dataset/kitti/raw/oxts.hpp"

namespace prototype {

int oxts_entry_load(oxts_entry_t &entry, const std::string &file_path) {
  // Load file
  std::ifstream oxt_file(file_path.c_str());
  if (oxt_file.good() == false) {
    return -1;
  }

  // Load the data
  std::string line;
  std::getline(oxt_file, line);
  const std::vector<double> array = parse_array(line);

  // Store
  const double lat = array[0];
  const double lon = array[1];
  const double alt = array[2];
  const double roll = array[3];
  const double pitch = array[4];
  const double yaw = array[5];
  const double vn = array[6];
  const double ve = array[7];
  const double vf = array[8];
  const double vl = array[9];
  const double vu = array[10];
  const double ax = array[11];
  const double ay = array[12];
  const double az = array[13];
  const double af = array[14];
  const double al = array[15];
  const double au = array[16];
  const double wx = array[17];
  const double wy = array[18];
  const double wz = array[19];
  const double wf = array[20];
  const double wl = array[21];
  const double wu = array[22];
  const double pos_acc = array[23];
  const double vel_acc = array[24];

  entry.gps = vec3_t{lat, lon, alt};
  entry.rpy = vec3_t{roll, pitch, yaw};
  entry.v_G = vec3_t{ve, vn, vu};
  entry.v_B = vec3_t{vf, vl, vu};
  entry.a_G = vec3_t{ax, ay, az};
  entry.a_B = vec3_t{af, al, au};
  entry.w_G = vec3_t{wx, wy, wz};
  entry.w_B = vec3_t{wf, wl, wu};
  entry.pos_accuracy = pos_acc;
  entry.vel_accuracy = vel_acc;

  return 0;
}

int oxts_load_entries(oxts_t &oxts, const std::string &oxts_dir) {
  // Get list of oxts files
  const std::string oxts_data_dir = strip(oxts_dir) + "/data";
  std::vector<std::string> oxts_files;
  if (list_dir(oxts_data_dir, oxts_files) != 0) {
    return -1;
  }
  std::sort(oxts_files.begin(), oxts_files.end());

  // Get first GPS point
  oxts_entry_t first_entry;
  const std::string file_path = oxts_data_dir + "/" + oxts_files[0];
  if (oxts_entry_load(first_entry, file_path) != 0) {
    return -1;
  }

  // Store initial conditions
  const vec3_t gps_ref = first_entry.gps;
  oxts.gps.emplace_back(first_entry.gps);
  oxts.rpy.emplace_back(first_entry.rpy);
  oxts.p_G.emplace_back(0, 0, 0);
  oxts.v_G.emplace_back(first_entry.v_G);
  oxts.v_B.emplace_back(first_entry.v_B);
  oxts.a_G.emplace_back(first_entry.a_G);
  oxts.a_B.emplace_back(first_entry.a_B);
  oxts.w_G.emplace_back(first_entry.w_G);
  oxts.w_B.emplace_back(first_entry.w_B);
  oxts.pos_accuracy.push_back(first_entry.pos_accuracy);
  oxts.vel_accuracy.push_back(first_entry.vel_accuracy);

  // Parse oxts files
  for (size_t i = 1; i < oxts_files.size(); i++) {
    // Parse single oxts file
    oxts_entry_t entry;
    const std::string file_path = oxts_data_dir + "/" + oxts_files[i];
    if (oxts_entry_load(entry, file_path) != 0) {
      return -1;
    }

    // Calculate local position
    double dist_N = 0.0;
    double dist_E = 0.0;
    double alt = entry.gps(2) - gps_ref(2);
    latlon_diff(gps_ref(0),
                gps_ref(1),
                entry.gps(0),
                entry.gps(1),
                &dist_N,
                &dist_E);

    // Store data
    oxts.gps.emplace_back(entry.gps);
    oxts.rpy.emplace_back(entry.rpy);
    oxts.p_G.emplace_back(dist_E, dist_N, alt);
    oxts.v_G.emplace_back(entry.v_G);
    oxts.v_B.emplace_back(entry.v_B);
    oxts.a_G.emplace_back(entry.a_G);
    oxts.a_B.emplace_back(entry.a_B);
    oxts.w_G.emplace_back(entry.w_G);
    oxts.w_B.emplace_back(entry.w_B);
    oxts.pos_accuracy.push_back(entry.pos_accuracy);
    oxts.vel_accuracy.push_back(entry.vel_accuracy);
  }

  return 0;
}

int oxts_load_timestamps(oxts_t &oxts, const std::string &oxts_dir) {
  // Setup parse
  const std::string file_path = strip(oxts_dir) + "/timestamps.txt";
  std::string line;
  std::ifstream timestamps_file(file_path.c_str());

  // Get first timestamp
  long ts_first = 0;
  int retval = 0;
  std::getline(timestamps_file, line);
  retval = parse_timestamp(line, &ts_first);
  if (retval != 0) {
    LOG_ERROR("Failed to parse timestamp -> [%s]", line.c_str());
    return -1;
  }
  oxts.timestamps.push_back(ts_first);
  oxts.time.push_back(0.0);
  line = std::string();

  // Parse the rest of timestamps
  long ts = 0;
  while (std::getline(timestamps_file, line)) {
    // Parse timestamp
    retval = parse_timestamp(line, &ts);
    if (retval != 0) {
      LOG_ERROR("Failed to parse timestamp -> [%s]", line.c_str());
      return -1;
    }

    // Check to make sure current parsed timestamp is larger than first
    if (ts_first > ts) {
      LOG_ERROR("First timestamp: %ld", ts_first);
      LOG_ERROR("Current timestamp: %ld", ts);
      LOG_ERROR("Current timestamp line: %s", line.c_str());
      FATAL("Failed to parse timestamp properly!");
    }

    oxts.timestamps.push_back(ts);
    oxts.time.push_back((double) (ts - ts_first) * 1.0e-9);
    line = std::string();
  }

  return 0;
}

int oxts_load(oxts_t &oxts, const std::string &oxts_dir) {
  if (oxts_load_entries(oxts, oxts_dir) != 0) {
    return -1;
  }
  if (oxts_load_timestamps(oxts, oxts_dir) != 0) {
    return -1;
  }

  return 0;
}

} //  namespace prototype
