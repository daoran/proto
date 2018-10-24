/**
 * @file
 * @defgroup imu imu
 */
#ifndef PROTOTYPE_IMU_IMU_BASE_HPP
#define PROTOTYPE_IMU_IMU_BASE_HPP

#include <stdio.h>
#include <stdint.h>
#include <string>

#include "prototype/core.hpp"
#include "prototype/driver/i2c.hpp"

namespace prototype {
/**
 * @addtogroup imu
 * @{
 */

struct GyroData {
  double sensitivity = 0.0;

  int16_t raw_x = 0.0f;
  int16_t raw_y = 0.0f;
  int16_t raw_z = 0.0f;

  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  GyroData() {}
};

struct AccelData {
  double sensitivity = 0.0;

  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  AccelData() {}
};

class IMUBase {
public:
  GyroData gyro;
  AccelData accel;
  I2C i2c;

  IMUBase() {}

  /**
   * Ping
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int ping() = 0;

  /**
   * Calibrate
   */
  void calibrate();

  /**
   * Get data
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int getData() = 0;

  /**
   * Record header
   */
  void recordHeader(FILE *output_file);

  /**
   * Record data
   */
  void recordData(FILE *output_file);

  /**
   * Record
   */
  int record(std::string output_path, int nb_samples);
};

/** @} group imu */
} //  namespace prototype
#endif // PROTOTYPE_IMU_IMU_BASE_HPP
