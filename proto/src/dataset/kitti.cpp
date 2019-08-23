#include "proto/dataset/kitti.hpp"

namespace proto {

std::string kitti_parse_string(const std::string &line) {
  return strip(line.substr(line.find(":") + 1, line.size() - 1));
}

double kitti_parse_double(const std::string &line) {
  const std::string str_val = kitti_parse_string(line);
  const double val = atof(str_val.c_str());
  return val;
}

std::vector<double> kitti_parse_array(const std::string &line) {
  // Setup
  std::string s = kitti_parse_string(line);
  std::string delimiter = " ";

  // Extract tokens
  size_t pos = 0;
  std::string token;
  std::vector<double> values;

  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    values.push_back(std::stod(token));
    s.erase(0, pos + delimiter.length());
  }

  // Extract last token
  token = s.substr(0, pos);
  values.push_back(std::stod(token));

  return values;
}

vec2_t kitti_parse_vec2(const std::string &line) {
  const std::vector<double> values = kitti_parse_array(line);
  return vec2_t(values[0], values[1]);
}

vec3_t kitti_parse_vec3(const std::string &line) {
  const std::vector<double> values = kitti_parse_array(line);
  return vec3_t(values[0], values[1], values[2]);
}

vecx_t kitti_parse_vecx(const std::string &line) {
  const std::vector<double> values = kitti_parse_array(line);

  // Form the vector
  vecx_t vec;
  vec.resize(values.size());
  for (size_t i = 0; i < values.size(); i++) {
    vec(i) = values[i];
  }

  return vec;
}

mat3_t kitti_parse_mat3(const std::string &line) {
  const std::vector<double> values = kitti_parse_array(line);

  // Form the matrix
  mat3_t mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

mat34_t kitti_parse_mat34(const std::string &line) {
  const std::vector<double> values = kitti_parse_array(line);

  // Form the matrix
  mat34_t mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 4; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

int kitti_parse_timestamp(const std::string &line, long *s) {
  // Parse datetime string
  unsigned int year = 0;
  unsigned int month = 0;
  unsigned int day = 0;
  unsigned int hour = 0;
  unsigned int minute = 0;
  unsigned int second = 0;
  unsigned int subsecond = 0;

  int scanned = std::sscanf(line.c_str(),
                            "%4u-%2u-%2u %2u:%2u:%2u.%u",
                            &year,
                            &month,
                            &day,
                            &hour,
                            &minute,
                            &second,
                            &subsecond);
  if (scanned != 7) {
    return -1;
  }

  // Convert datetime to milliseconds since epoch (1970)
  struct tm t = {}; // IMPORTANT: MUST INITIALIZE TO ZERO ELSE BUGS...
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  *s = ((double) mktime(&t) * 1.0e9) + subsecond;

  return 0;
}

/*****************************************************************************
 * calib_cam2cam_t
 *****************************************************************************/

int calib_cam2cam_load(calib_cam2cam_t &calib) {
  std::ifstream file(calib.file_path.c_str());
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      calib.calib_time = kitti_parse_string(line);
      continue;

    } else if (token == "corner_dist") {
      calib.corner_dist = kitti_parse_double(line);
      continue;
    }

    // Parse rectification variables
    const double cam_id = std::stod(token.substr(token.length() - 1));
    const bool rect_var = (token.find("rect") != std::string::npos);
    if (rect_var) {
      if (token[0] == 'S') {
        calib.S_rect[cam_id] = kitti_parse_vec2(line);
      } else if (token[0] == 'R') {
        calib.R_rect[cam_id] = kitti_parse_mat3(line);
      } else if (token[0] == 'P') {
        calib.P_rect[cam_id] = kitti_parse_mat34(line);
      }

      continue;
    }

    // Parse calibration variables
    if (token[0] == 'S') {
      calib.S[cam_id] = kitti_parse_vec2(line);
    } else if (token[0] == 'K') {
      calib.K[cam_id] = kitti_parse_mat3(line);
    } else if (token[0] == 'D') {
      calib.D[cam_id] = kitti_parse_vecx(line);
    } else if (token[0] == 'R') {
      calib.R[cam_id] = kitti_parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.T[cam_id] = kitti_parse_vec3(line);
    }
  }

  calib.ok = true;
  return 0;
}

/*****************************************************************************
 * calib_imu2velo_t
 *****************************************************************************/

int calib_imu2velo_load(calib_imu2velo_t &calib) {
  std::ifstream file(calib.file_path.c_str());

  // Check file
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      calib.calib_time = kitti_parse_string(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      calib.R = kitti_parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.t = kitti_parse_vec3(line);
    }
  }

  calib.T_velo_imu = tf(calib.R, calib.t);
  calib.ok = true;
  return 0;
}

/*****************************************************************************
 * calib_velo2cam_t
 *****************************************************************************/

int calib_velo2cam_load(calib_velo2cam_t &calib) {
  std::ifstream file(calib.file_path.c_str());

  // Check file
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      calib.calib_time = kitti_parse_string(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      calib.R = kitti_parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.t = kitti_parse_vec3(line);
    } else if (token == "delta_f") {
      calib.df = kitti_parse_vec2(line);
    } else if (token == "delta_c") {
      calib.dc = kitti_parse_vec2(line);
    }
  }

  calib.T_cam_velo = tf(calib.R, calib.t);
  calib.ok = true;
  return 0;
}

/*****************************************************************************
 * oxts_entry_t
 *****************************************************************************/

int oxts_entry_load(oxts_entry_t &entry) {
  // Load file
  std::ifstream oxt_file(entry.file_path.c_str());
  if (oxt_file.good() == false) {
    return -1;
  }

  // Load the data
  std::string line;
  std::getline(oxt_file, line);
  const std::vector<double> array = kitti_parse_array(line);

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

/*****************************************************************************
 * oxts_t
 *****************************************************************************/

int oxts_load_entries(oxts_t &oxts) {
  // Get list of oxts files
  const std::string oxts_data_dir = strip(oxts.oxts_dir) + "/data";
  std::vector<std::string> oxts_files;
  if (list_dir(oxts_data_dir, oxts_files) != 0) {
    return -1;
  }
  std::sort(oxts_files.begin(), oxts_files.end());

  // Get first GPS point
  oxts_entry_t first_entry{oxts_data_dir + "/" + oxts_files[0]};
  if (oxts_entry_load(first_entry) != 0) {
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
    oxts_entry_t entry{oxts_data_dir + "/" + oxts_files[i]};
    if (oxts_entry_load(entry) != 0) {
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

int oxts_load_timestamps(oxts_t &oxts) {
  // Setup parse
  const std::string file_path = strip(oxts.oxts_dir) + "/timestamps.txt";
  std::string line;
  std::ifstream timestamps_file(file_path.c_str());

  // Get first timestamp
  long ts_first = 0;
  int retval = 0;
  std::getline(timestamps_file, line);
  retval = kitti_parse_timestamp(line, &ts_first);
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
    retval = kitti_parse_timestamp(line, &ts);
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

int oxts_load(oxts_t &oxts) {
  if (oxts_load_entries(oxts) != 0) {
    return -1;
  }
  if (oxts_load_timestamps(oxts) != 0) {
    return -1;
  }

  return 0;
}

/*****************************************************************************
 * kitti_raw_t
 *****************************************************************************/

kitti_raw_t::kitti_raw_t() {}

kitti_raw_t::kitti_raw_t(const std::string &raw_dir,
                         const std::string &date,
                         const std::string &seq)
    : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
      date_dir{raw_dir + "/" + date}, drive_dir{date_dir + "/" + date +
                                                "_drive_" + seq + "_sync"} {}

kitti_raw_t::kitti_raw_t(const std::string &raw_dir,
                         const std::string &date,
                         const std::string &seq,
                         const std::string &type)
    : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
      date_dir{raw_dir + "/" + date}, drive_dir{date_dir + "/" + date +
                                                "_drive_" + seq + "_" + type} {}

kitti_raw_t::~kitti_raw_t() {}

int kitti_raw_load(kitti_raw_t &ds) {
  // Pre-check
  bool res = dir_exists(ds.date_dir);
  CHECK(res, "Dir not found! [%s]", ds.date_dir.c_str());

  // Load calibration files
  int retval;
  // -- Camera to camera calibration
  ds.cam2cam = calib_cam2cam_t{ds.date_dir + "/calib_cam_to_cam.txt"};
  retval = calib_cam2cam_load(ds.cam2cam);
  CHECK(retval == 0, "Failed to load camera to camera calibration!");
  // -- IMU to Velo calibration
  ds.imu2velo = calib_imu2velo_t{ds.date_dir + "/calib_imu_to_velo.txt"};
  retval = calib_imu2velo_load(ds.imu2velo);
  CHECK(retval == 0, "Failed to load imu to velocalibration!");
  // -- Velo to camera calibration
  ds.velo2cam = calib_velo2cam_t{ds.date_dir + "/calib_velo_to_cam.txt"};
  retval = calib_velo2cam_load(ds.velo2cam);
  CHECK(retval == 0, "Failed to load velo to camera calibration!");

  // Load image files
  // -- Get list of image paths
  retval = list_dir(ds.drive_dir + "/image_00/data", ds.cam0);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_01/data", ds.cam1);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_02/data", ds.cam2);
  CHECK(retval == 0, "Failed to traverse image files!");
  retval = list_dir(ds.drive_dir + "/image_03/data", ds.cam3);
  CHECK(retval == 0, "Failed to traverse image files!");
  // -- Sort image paths
  std::sort(ds.cam0.begin(), ds.cam0.end());
  std::sort(ds.cam1.begin(), ds.cam1.end());
  std::sort(ds.cam2.begin(), ds.cam2.end());
  std::sort(ds.cam3.begin(), ds.cam3.end());
  // -- Add full path to files
  for (size_t i = 0; i < ds.cam0.size(); i++) {
    ds.cam0[i] = ds.drive_dir + "/image_00/data/" + ds.cam0[i];
    ds.cam1[i] = ds.drive_dir + "/image_01/data/" + ds.cam1[i];
    ds.cam2[i] = ds.drive_dir + "/image_02/data/" + ds.cam2[i];
    ds.cam3[i] = ds.drive_dir + "/image_03/data/" + ds.cam3[i];
  }

  // Load OXTS data
  ds.oxts = oxts_t{ds.drive_dir + "/oxts"};
  retval = oxts_load(ds.oxts);
  CHECK(retval == 0, "Failed to load OXTS data!");

  ds.ok = true;
  return 0;
error:
  return -1;
}

} //  namespace proto
