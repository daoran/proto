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
struct camera_data_t {
  std::string data_dir;

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

  camera_data_t() {}
  camera_data_t(const std::string &data_dir_) : data_dir{data_dir_} {}
};

/**
 * Load Camera data
 *
 * @param[in,out] data Camera data
 * @param[in] data_dir Camera data directory
 * @returns 0 for success, -1 for failure
 */
int camera_data_load(camera_data_t &cam_data);

/**
 * camera_data_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const camera_data_t &data);

/** @} group euroc */
} //  namespace prototype
#endif // PROTOTYPE_VISION_DATASET_EUROC_CAMERA_DATA_HPP
