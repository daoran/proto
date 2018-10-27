/**
 * @file
 * @defgroup camera camera
 */
#ifndef PROTOTYPE_DRIVER_CAMERA_BASE_HPP
#define PROTOTYPE_DRIVER_CAMERA_BASE_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Camera config file
 */
struct camera_config_t {
  bool ok = false;
  std::string file_path;

  int index = 0;
  int image_width = 0;
  int image_height = 0;
  std::string image_type = "bgr8";

  camera_config_t() {}
  camera_config_t(const std::string &file_path_) : file_path{file_path_} {}
  virtual ~camera_config_t() {}
};

/**
 * Camera config to output stream
 */
std::ostream &operator<<(std::ostream &os, const camera_config_t &config);

/**
 * Generic camera
 */
struct camera_t {
  bool connected = false;
  int camera_index = 0;
  cv::VideoCapture capture;


  double last_tic = 0.0;
  cv::Mat image = cv::Mat(0, 0, CV_64F);

  camera_t() {}
  camera_t(const int camera_index_) : camera_index{camera_index_} {}
  virtual ~camera_t() {}
};

/**
 * Load camera config
 *
 * @param[in,out] config Camera config
 * @returns 0 for success, -1 for failure
 */
int camera_config_load(camera_config_t &config);

/**
 * Configure camera
 *
 * @param[in,out] cam Camera
 * @param[in] config Camera config
 * @returns 0 for success, -1 for failure
 */
int camera_configure(camera_t &cam, const camera_config_t &config);

/**
 * Connect camera
 *
 * @param[in] cam Camera
 * @returns 0 for success, -1 for failure
 */
int camera_connect(camera_t &cam);

/**
 * Disconnect camera
 *
 * @param[in] cam Camera
 * @returns 0 for success, -1 for failure
 */
int camera_disconnect(camera_t &cam);

/**
 * Get camera frame
 *
 * @param[in] cam Camera
 * @returns 0 for success, -1 for failure
 */
int camera_get_frame(camera_t &cam);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_DRIVER_CAMERA_BASE_HPP
