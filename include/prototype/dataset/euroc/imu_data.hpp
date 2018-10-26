/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef PROTOTYPE_VISION_DATASET_EUROC_IMU_DATA_HPP
#define PROTOTYPE_VISION_DATASET_EUROC_IMU_DATA_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * IMU data
 */
struct imu_data_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> w_B;
  std::vector<vec3_t> a_B;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;

  imu_data_t() {}
  imu_data_t(const std::string &data_dir_) : data_dir{data_dir_} {}
};

/**
  * Load IMU data
  *
  * @param data_dir IMU data directory
  * @returns 0 for success, -1 for failure
  */
int imu_data_load(imu_data_t &data);

/**
 * IMUData to output stream
 */
std::ostream &operator<<(std::ostream &os, const imu_data_t &data);

/** @} group euroc */
} //  namespace prototype
#endif // PROTOTYPE_VISION_DATASET_EUROC_IMU_DATA_HPP
