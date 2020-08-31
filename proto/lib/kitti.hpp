#ifndef PROTO_DATASET_KITTI_HPP
#define PROTO_DATASET_KITTI_HPP

#include "core.hpp"

namespace proto {

/** KITTI Dataset parse functions **/
struct kitti {
  static std::string parse_string(const std::string &line) {
    return strip(line.substr(line.find(":") + 1, line.size() - 1));
  }

  static real_t parse_double(const std::string &line) {
    const std::string str_val = parse_string(line);
    const real_t val = atof(str_val.c_str());
    return val;
  }

  static std::vector<real_t> parse_array(const std::string &line) {
    // Setup
    std::string s = parse_string(line);
    std::string delimiter = " ";

    // Extract tokens
    size_t pos = 0;
    std::string token;
    std::vector<real_t> values;

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

  static vec2_t parse_vec2(const std::string &line) {
    const std::vector<real_t> values = parse_array(line);
    return vec2_t(values[0], values[1]);
  }

  static vec3_t parse_vec3(const std::string &line) {
    const std::vector<real_t> values = parse_array(line);
    return vec3_t(values[0], values[1], values[2]);
  }

  static vecx_t parse_vecx(const std::string &line) {
    const std::vector<real_t> values = parse_array(line);

    // Form the vector
    vecx_t vec;
    vec.resize(values.size());
    for (size_t i = 0; i < values.size(); i++) {
      vec(i) = values[i];
    }

    return vec;
  }

  static mat3_t parse_mat3(const std::string &line) {
    const std::vector<real_t> values = parse_array(line);

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

  static mat34_t parse_mat34(const std::string &line) {
    const std::vector<real_t> values = parse_array(line);

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

  static int parse_timestamp(const std::string &line, long *s) {
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
    *s = ((real_t) mktime(&t) * 1.0e9) + subsecond;

    return 0;
  }
};

/** Camera to Camera calibration **/
struct calib_cam2cam_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  real_t corner_dist = 0.0;
  std::array<vec2_t, 4> S;
  std::array<mat3_t, 4> K;
  std::array<vecx_t, 4> D;
  std::array<mat3_t, 4> R;
  std::array<vec3_t, 4> T;
  std::array<vec2_t, 4> S_rect;
  std::array<mat3_t, 4> R_rect;
  std::array<mat34_t, 4> P_rect;

  calib_cam2cam_t() {}

  calib_cam2cam_t(const std::string &file_path_) : file_path{file_path_} {
    std::ifstream file(file_path.c_str());
    if (file.good() != true) {
      FATAL("Failed to load file [%s]!", file_path.c_str());
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
        calib_time = kitti::parse_string(line);
        continue;

      } else if (token == "corner_dist") {
        corner_dist = kitti::parse_double(line);
        continue;
      }

      // Parse rectification variables
      const real_t cam_id = std::stod(token.substr(token.length() - 1));
      const bool rect_var = (token.find("rect") != std::string::npos);
      if (rect_var) {
        if (token[0] == 'S') {
          S_rect[cam_id] = kitti::parse_vec2(line);
        } else if (token[0] == 'R') {
          R_rect[cam_id] = kitti::parse_mat3(line);
        } else if (token[0] == 'P') {
          P_rect[cam_id] = kitti::parse_mat34(line);
        }

        continue;
      }

      // Parse calibration variables
      if (token[0] == 'S') {
        S[cam_id] = kitti::parse_vec2(line);
      } else if (token[0] == 'K') {
        K[cam_id] = kitti::parse_mat3(line);
      } else if (token[0] == 'D') {
        D[cam_id] = kitti::parse_vecx(line);
      } else if (token[0] == 'R') {
        R[cam_id] = kitti::parse_mat3(line);
      } else if (token[0] == 'T') {
        T[cam_id] = kitti::parse_vec3(line);
      }
    }

    ok = true;
  }
};

/** IMU to Velo calibration **/
struct calib_imu2velo_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  mat3_t R;
  vec3_t t;
  mat4_t T_velo_imu;

  calib_imu2velo_t() {}

  calib_imu2velo_t(const std::string &file_path_) : file_path{file_path_} {
    std::ifstream file(file_path.c_str());

    // Check file
    if (file.good() != true) {
      FATAL("Failed to load file [%s]!", file_path.c_str());
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
        calib_time = kitti::parse_string(line);
        continue;
      }

      // Parse calibration variables
      if (token[0] == 'R') {
        R = kitti::parse_mat3(line);
      } else if (token[0] == 'T') {
        t = kitti::parse_vec3(line);
      }
    }

    T_velo_imu = tf(R, t);
    ok = true;
  }
};

/** Velo to Camera calibration **/
struct calib_velo2cam_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool ok = false;
  std::string file_path;

  std::string calib_time;
  mat3_t R;
  vec3_t t;
  vec2_t df;
  vec2_t dc;
  mat4_t T_cam_velo;

  calib_velo2cam_t() {}

  calib_velo2cam_t(const std::string &file_path_) : file_path{file_path_} {
    // Check file
    std::ifstream file(file_path.c_str());
    if (file.good() != true) {
      FATAL("Failed to load file [%s]!", file_path.c_str());
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
        calib_time = kitti::parse_string(line);
        continue;
      }

      // Parse calibration variables
      if (token[0] == 'R') {
        R = kitti::parse_mat3(line);
      } else if (token[0] == 'T') {
        t = kitti::parse_vec3(line);
      } else if (token == "delta_f") {
        df = kitti::parse_vec2(line);
      } else if (token == "delta_c") {
        dc = kitti::parse_vec2(line);
      }
    }

    T_cam_velo = tf(R, t);
    ok = true;
  }
};

/** OXTS entry **/
struct oxts_entry_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  std::string file_path;

  vec3_t gps = zeros(3, 1);
  vec3_t rpy = zeros(3, 1);
  vec3_t v_G = zeros(3, 1);
  vec3_t v_B = zeros(3, 1);
  vec3_t a_G = zeros(3, 1);
  vec3_t a_B = zeros(3, 1);
  vec3_t w_G = zeros(3, 1);
  vec3_t w_B = zeros(3, 1);
  real_t pos_accuracy = 0.0;
  real_t vel_accuracy = 0.0;

  oxts_entry_t() {}
  oxts_entry_t(const std::string &file_path_) : file_path{file_path_} {
    // Load file
    std::ifstream oxt_file(file_path.c_str());
    if (oxt_file.good() == false) {
      FATAL("Failed to load file [%s]!", file_path.c_str());
    }

    // Load the data
    std::string line;
    std::getline(oxt_file, line);
    const std::vector<real_t> array = kitti::parse_array(line);

    // Store
    const real_t lat = array[0];
    const real_t lon = array[1];
    const real_t alt = array[2];
    const real_t roll = array[3];
    const real_t pitch = array[4];
    const real_t yaw = array[5];
    const real_t vn = array[6];
    const real_t ve = array[7];
    const real_t vf = array[8];
    const real_t vl = array[9];
    const real_t vu = array[10];
    const real_t ax = array[11];
    const real_t ay = array[12];
    const real_t az = array[13];
    const real_t af = array[14];
    const real_t al = array[15];
    const real_t au = array[16];
    const real_t wx = array[17];
    const real_t wy = array[18];
    const real_t wz = array[19];
    const real_t wf = array[20];
    const real_t wl = array[21];
    const real_t wu = array[22];
    const real_t pos_acc = array[23];
    const real_t vel_acc = array[24];

    gps = vec3_t{lat, lon, alt};
    rpy = vec3_t{roll, pitch, yaw};
    v_G = vec3_t{ve, vn, vu};
    v_B = vec3_t{vf, vl, vu};
    a_G = vec3_t{ax, ay, az};
    a_B = vec3_t{af, al, au};
    w_G = vec3_t{wx, wy, wz};
    w_B = vec3_t{wf, wl, wu};
    pos_accuracy = pos_acc;
    vel_accuracy = vel_acc;
  }
};

/** OXTS **/
struct oxts_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  std::string oxts_dir;

  std::vector<long> timestamps;
  std::vector<real_t> time;
  vec3s_t gps;
  vec3s_t rpy;
  vec3s_t p_G;
  vec3s_t v_G;
  vec3s_t v_B;
  vec3s_t a_G;
  vec3s_t a_B;
  vec3s_t w_G;
  vec3s_t w_B;
  std::vector<real_t> pos_accuracy;
  std::vector<real_t> vel_accuracy;

  oxts_t() {}
  oxts_t(const std::string &oxts_dir_) : oxts_dir{oxts_dir_} {
    // ----------------------- LOAD OXTS ENTRIES -----------------------------
    // Get list of oxts files
    const std::string oxts_data_dir = strip(oxts_dir) + "/data";
    std::vector<std::string> oxts_files;
    if (list_dir(oxts_data_dir, oxts_files) != 0) {
      FATAL("Failed to list directory [%s]!", oxts_data_dir.c_str());
    }
    std::sort(oxts_files.begin(), oxts_files.end());

    // Get first GPS point
    oxts_entry_t first_entry{oxts_data_dir + "/" + oxts_files[0]};

    // Store initial conditions
    const vec3_t gps_ref = first_entry.gps;
    gps.emplace_back(first_entry.gps);
    rpy.emplace_back(first_entry.rpy);
    p_G.emplace_back(0, 0, 0);
    v_G.emplace_back(first_entry.v_G);
    v_B.emplace_back(first_entry.v_B);
    a_G.emplace_back(first_entry.a_G);
    a_B.emplace_back(first_entry.a_B);
    w_G.emplace_back(first_entry.w_G);
    w_B.emplace_back(first_entry.w_B);
    pos_accuracy.push_back(first_entry.pos_accuracy);
    vel_accuracy.push_back(first_entry.vel_accuracy);

    // Parse oxts files
    for (size_t i = 1; i < oxts_files.size(); i++) {
      // Parse single oxts file
      oxts_entry_t entry{oxts_data_dir + "/" + oxts_files[i]};

      // Calculate local position
      real_t dist_N = 0.0;
      real_t dist_E = 0.0;
      real_t alt = entry.gps(2) - gps_ref(2);
      latlon_diff(gps_ref(0),
                  gps_ref(1),
                  entry.gps(0),
                  entry.gps(1),
                  &dist_N,
                  &dist_E);

      // Store data
      gps.emplace_back(entry.gps);
      rpy.emplace_back(entry.rpy);
      p_G.emplace_back(dist_E, dist_N, alt);
      v_G.emplace_back(entry.v_G);
      v_B.emplace_back(entry.v_B);
      a_G.emplace_back(entry.a_G);
      a_B.emplace_back(entry.a_B);
      w_G.emplace_back(entry.w_G);
      w_B.emplace_back(entry.w_B);
      pos_accuracy.push_back(entry.pos_accuracy);
      vel_accuracy.push_back(entry.vel_accuracy);
    }

    // ------------------------ LOAD TIMESTAMPS ------------------------------
    const std::string file_path = strip(oxts_dir) + "/timestamps.txt";
    std::string line;
    std::ifstream timestamps_file(file_path.c_str());

    // Get first timestamp
    long ts_first = 0;
    int retval = 0;
    std::getline(timestamps_file, line);
    retval = kitti::parse_timestamp(line, &ts_first);
    if (retval != 0) {
      FATAL("Failed to parse timestamp -> [%s]", line.c_str());
    }
    timestamps.push_back(ts_first);
    time.push_back(0.0);
    line = std::string();

    // Parse the rest of timestamps
    long ts = 0;
    while (std::getline(timestamps_file, line)) {
      // Parse timestamp
      retval = kitti::parse_timestamp(line, &ts);
      if (retval != 0) {
        FATAL("Failed to parse timestamp -> [%s]", line.c_str());
      }

      // Check to make sure current parsed timestamp is larger than first
      if (ts_first > ts) {
        LOG_ERROR("First timestamp: %ld", ts_first);
        LOG_ERROR("Current timestamp: %ld", ts);
        LOG_ERROR("Current timestamp line: %s", line.c_str());
        FATAL("Failed to parse timestamp properly!");
      }

      timestamps.push_back(ts);
      time.push_back((real_t) (ts - ts_first) * 1.0e-9);
      line = std::string();
    }
  }
};

/** KITTI Raw Dataset **/
struct kitti_raw_t {
  bool ok = false;

  std::string raw_dir;
  std::string date;
  std::string seq;
  std::string date_dir;
  std::string drive_dir;

  calib_cam2cam_t cam2cam;
  calib_imu2velo_t imu2velo;
  calib_velo2cam_t velo2cam;

  oxts_t oxts;
  std::vector<std::string> cam0;
  std::vector<std::string> cam1;
  std::vector<std::string> cam2;
  std::vector<std::string> cam3;

  kitti_raw_t(const std::string &raw_dir,
              const std::string &date,
              const std::string &seq,
              const std::string &type)
      : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
        date_dir{raw_dir + "/" + date},
        drive_dir{date_dir + "/" + date + "_drive_" + seq + "_" + type} {
    // Pre-check
    CHECK(dir_exists(date_dir), "Dir not found! [%s]", date_dir.c_str());
    CHECK(dir_exists(drive_dir), "Dir not found! [%s]", drive_dir.c_str());

    // Load calibration files
    int retval;
    // -- Camera to camera calibration
    cam2cam = calib_cam2cam_t{date_dir + "/calib_cam_to_cam.txt"};
    // -- IMU to Velo calibration
    imu2velo = calib_imu2velo_t{date_dir + "/calib_imu_to_velo.txt"};
    // -- Velo to camera calibration
    velo2cam = calib_velo2cam_t{date_dir + "/calib_velo_to_cam.txt"};

    // Load image files
    // -- Get list of image paths
    retval = list_dir(drive_dir + "/image_00/data", cam0);
    CHECK(retval == 0, "Failed to traverse image files!");
    retval = list_dir(drive_dir + "/image_01/data", cam1);
    CHECK(retval == 0, "Failed to traverse image files!");
    retval = list_dir(drive_dir + "/image_02/data", cam2);
    CHECK(retval == 0, "Failed to traverse image files!");
    retval = list_dir(drive_dir + "/image_03/data", cam3);
    CHECK(retval == 0, "Failed to traverse image files!");
    // -- Sort image paths
    std::sort(cam0.begin(), cam0.end());
    std::sort(cam1.begin(), cam1.end());
    std::sort(cam2.begin(), cam2.end());
    std::sort(cam3.begin(), cam3.end());
    // -- Add full path to files
    for (size_t i = 0; i < cam0.size(); i++) {
      cam0[i] = drive_dir + "/image_00/data/" + cam0[i];
      cam1[i] = drive_dir + "/image_01/data/" + cam1[i];
      cam2[i] = drive_dir + "/image_02/data/" + cam2[i];
      cam3[i] = drive_dir + "/image_03/data/" + cam3[i];
    }

    // Load OXTS data
    oxts = oxts_t{drive_dir + "/oxts"};

    ok = true;
error:
    ok = false;
  }

  ~kitti_raw_t() {}
};

} //  namespace proto
#endif // PROTO_DATASET_KITTI_HPP
