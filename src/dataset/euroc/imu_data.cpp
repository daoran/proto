#include "prototype/dataset/euroc/imu_data.hpp"

namespace prototype {

int imu_data_load(imu_data_t &data) {
  const std::string imu_data_path = data.data_dir + "/data.csv";
  const std::string imu_calib_path = data.data_dir + "/sensor.yaml";

  // Load IMU data
  matx_t csv_data;
  if (csv2mat(imu_data_path, true, csv_data) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_path.c_str());
    return -1;
  }

  const long t0 = csv_data(0, 0);
  for (long i = 0; i < csv_data.rows(); i++) {
    const long ts = csv_data(i, 0);
    data.timestamps.push_back(ts);
    data.time.push_back((ts - t0) * 1e-9);
    data.w_B.emplace_back(csv_data(i, 1), csv_data(i, 2), csv_data(i, 3));
    data.a_B.emplace_back(csv_data(i, 4), csv_data(i, 5), csv_data(i, 6));
  }

  // Load calibration data
  config_parser_t parser;
  config_parser_add(parser, "sensor_type", &data.sensor_type);
  config_parser_add(parser, "comment", &data.comment);
  config_parser_add(parser, "T_BS", &data.T_BS);
  config_parser_add(parser, "rate_hz", &data.rate_hz);
  config_parser_add(parser, "gyroscope_noise_density", &data.gyro_noise_density);
  config_parser_add(parser, "gyroscope_random_walk", &data.gyro_random_walk);
  config_parser_add(parser, "accelerometer_noise_density", &data.accel_noise_density);
  config_parser_add(parser, "accelerometer_random_walk", &data.accel_random_walk);
  if (config_parser_load(parser, imu_calib_path) != 0) {
    LOG_ERROR("Failed to load sensor file [%s]!", imu_calib_path.c_str());
    return -1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const imu_data_t &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "gyroscope_noise_density: " << data.gyro_noise_density << std::endl;
  os << "gyroscope_random_walk: " << data.gyro_random_walk << std::endl;
  os << "accelerometer_noise_density: " << data.accel_noise_density << std::endl;
  os << "accelerometer_random_walk: " << data.accel_random_walk << std::endl;
  // clang-format on

  return os;
}

} //  namespace prototype
