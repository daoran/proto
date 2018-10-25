/**
 * @file
 * @ingroup gimbal
 */
#ifndef PROTOTYPE_GIMBAL_BASE_HPP
#define PROTOTYPE_GIMBAL_BASE_HPP

#include <iostream>
#include <map>
#include <math.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "prototype/core.hpp"
#include "prototype/driver/gimbal/sbgc.hpp"

namespace prototype {
/**
 * @addtogroup gimbal
 * @{
 */

class GimbalBase {
public:
  bool configured = false;

  SBGC sbgc;
  double roll_limits[2] = {0.0, 0.0};
  double pitch_limits[2] = {0.0, 0.0};

  vec3_t setpoints = vec3_t{0.0, 0.0, 0.0};

  vec3_t imu_accel = vec3_t{0.0, 0.0, 0.0};
  vec3_t imu_gyro = vec3_t{0.0, 0.0, 0.0};
  vec3_t camera_angles = vec3_t{0.0, 0.0, 0.0};
  vec3_t frame_angles = vec3_t{0.0, 0.0, 0.0};
  vec3_t encoder_angles = vec3_t{0.0, 0.0, 0.0};

  GimbalBase();
  virtual ~GimbalBase();

  /**
   * Configure
   *
   * @param config_path Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_path);

  /**
   * Turn gimbal on
   * @returns 0 for success, -1 for failure
   */
  int on();

  /**
   * Turn gimbal off
   * @returns 0 for success, -1 for failure
   */
  int off();

  /**
   * Update gimbal states
   * @returns 0 for success, -1 for failure
   */
  int update();

  /**
   * Set gimbal angle
   *
   * @param roll Roll (radians)
   * @param pitch Pitch (radians)
   * @returns 0 for success, -1 for failure
   */
  int setAngle(const double roll, const double pitch);

  /**
   * Print setpoints
   */
  void printSetpoints();
};

} //  namespace prototype
/** @} group gimbal */
#endif // PROTOTYPE_GIMBAL_HPP
