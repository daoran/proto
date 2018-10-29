#include "prototype/dataset/kitti/raw/calib.hpp"

namespace prototype {

int calib_cam2cam_load(calib_cam2cam_t &calib) {
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
      calib.calib_time = parse_string(line);
      continue;

    } else if (token == "corner_dist") {
      calib.corner_dist = parse_double(line);
      continue;
    }

    // Parse rectification variables
    const double cam_id = std::stod(token.substr(token.length() - 1));
    const bool rect_var = (token.find("rect") != std::string::npos);
    if (rect_var) {
      if (token[0] == 'S') {
        calib.S_rect[cam_id] = parse_vec2(line);
      } else if (token[0] == 'R') {
        calib.R_rect[cam_id] = parse_mat3(line);
      } else if (token[0] == 'P') {
        calib.P_rect[cam_id] = parse_mat34(line);
      }

      continue;
    }

    // Parse calibration variables
    if (token[0] == 'S') {
      calib.S[cam_id] = parse_vec2(line);
    } else if (token[0] == 'K') {
      calib.K[cam_id] = parse_mat3(line);
    } else if (token[0] == 'D') {
      calib.D[cam_id] = parse_vecx(line);
    } else if (token[0] == 'R') {
      calib.R[cam_id] = parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.T[cam_id] = parse_vec3(line);
    }
  }

  calib.ok = true;
  return 0;
}

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
      calib.calib_time = parse_string(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      calib.R = parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.t = parse_vec3(line);
    }
  }

  calib.T_velo_imu = transform(calib.R, calib.t);
  calib.ok = true;
  return 0;
}

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
      calib.calib_time = parse_string(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      calib.R = parse_mat3(line);
    } else if (token[0] == 'T') {
      calib.t = parse_vec3(line);
    } else if (token == "delta_f") {
      calib.df = parse_vec2(line);
    } else if (token == "delta_c") {
      calib.dc = parse_vec2(line);
    }
  }

  calib.T_cam_velo = transform(calib.R, calib.t);
  calib.ok = true;
  return 0;
}

} //  namespace prototype
