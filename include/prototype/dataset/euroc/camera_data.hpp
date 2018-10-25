/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef PROTOTYPE_VISION_DATASET_EUROC_CAMERA_DATA_HPP
#define PROTOTYPE_VISION_DATASET_EUROC_CAMERA_DATA_HPP

#include <iostream>
#include <vector>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * Camera data
 */
struct CameraData {
  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<std::string> image_paths;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  mat4_t T_BS = I(4);
  double rate_hz = 0.0;
  vec2_t resolution;
  std::string camera_model;
  vec4_t intrinsics;
  std::string distortion_model;
  vec4_t distortion_coefficients;

  /**
   * Load Camera data
   *
   * @param data_dir Camera data directory
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/**
 * CameraData to output stream
 */
std::ostream &operator<<(std::ostream &os, const CameraData &data);

/** @} group euroc */
} //  namespace prototype
#endif // PROTOTYPE_VISION_DATASET_EUROC_CAMERA_DATA_HPP
