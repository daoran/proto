#include "prototype/dataset/euroc/camera_data.hpp"

namespace prototype {

int camera_data_load(camera_data_t &cd) {
  const std::string cam_data_path = cd.data_dir + "/data.csv";
  const std::string cam_calib_path = cd.data_dir + "/sensor.yaml";

  // Load camera data
  matx_t data;
  if (csv2mat(cam_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load camera data [%s]!", cam_data_path.c_str());
    return -1;
  }

  const long t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const std::string image_file = std::to_string((long) data(i, 0)) + ".png";
    const std::string image_path = cd.data_dir + "/data/" + image_file;
    const long ts = data(i, 0);

    if (file_exists(image_path) == false) {
      LOG_ERROR("File [%s] does not exist!", image_path.c_str());
      return -1;
    }

    cd.timestamps.emplace_back(ts);
    cd.time.emplace_back((ts - t0) * 1e-9);
    cd.image_paths.emplace_back(image_path);
  }

  // Load calibration data
  config_t config{cam_calib_path};
  if (config.ok == false) {
    LOG_ERROR("Failed to load senor file [%s]!", cam_calib_path.c_str());
    return -1;
  }
  parse(config, "sensor_type", cd.sensor_type);
  parse(config, "comment", cd.comment);
  parse(config, "T_BS", cd.T_BS);
  parse(config, "rate_hz", cd.rate_hz);
  parse(config, "resolution", cd.resolution);
  parse(config, "camera_model", cd.camera_model);
  parse(config, "intrinsics", cd.intrinsics);
  parse(config, "distortion_model", cd.distortion_model);
  parse(config, "distortion_coefficients", cd.distortion_coefficients);

  return 0;
}

std::ostream &operator<<(std::ostream &os, const camera_data_t &cd) {
  // clang-format off
  os << "sensor_type: " << cd.sensor_type << std::endl;
  os << "comment: " << cd.comment << std::endl;
  os << "T_BS:\n" << cd.T_BS << std::endl;
  os << "rate_hz: " << cd.rate_hz << std::endl;
  os << "resolution: " << cd.resolution.transpose() << std::endl;
  os << "camera_model: " << cd.camera_model << std::endl;
  os << "intrinsics: " << cd.intrinsics.transpose() << std::endl;
  os << "distortion_model: " << cd.distortion_model << std::endl;
  os << "distortion_coefficients: " << cd.distortion_coefficients.transpose() << std::endl;
  // clang-format on

  return os;
}

} //  namespace prototype
