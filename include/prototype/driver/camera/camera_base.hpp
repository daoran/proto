/**
 * @file
 * @defgroup camera camera
 */
#ifndef PROTOTYPE_DRIVER_CAMERA_BASE_HPP
#define PROTOTYPE_DRIVER_CAMERA_BASE_HPP

#include "prototype/core.hpp"
#include "prototype/driver/camera/camera_config.hpp"
// #include "prototype/driver/camera/ids.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Generic Camera Base
 */
class CameraBase {
public:
  bool configured = false;
  bool connected = false;

  CameraConfig config;
  std::vector<std::string> modes;
  std::map<std::string, CameraConfig> configs;

  cv::Mat image = cv::Mat(0, 0, CV_64F);
  double last_tic = 0.0;

  cv::VideoCapture *capture = nullptr;

  CameraBase() {}
  virtual ~CameraBase();

  /**
   * Configure camera
   *
   * @param config_path Path to config file (YAML)
   * @returns 0 for success, -1 for failure
   */
  virtual int configure(const std::string &config_path);

  /**
   * Connect camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int connect();

  /**
   * Disconncet camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int disconnect();

  /**
   * Change camera mode
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int changeMode(const std::string &mode);

  /**
   * ROI Image

   * @returns 0 for success, -1 for failure
   */
  virtual int roiImage(cv::Mat &image);

  /**
   * Get camera frame
   *
   * @param image Camera frame image
   * @returns 0 for success, -1 for failure
   */
  virtual int getFrame(cv::Mat &image);

  /**
   * Run camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int run();

  /**
   * Show FPS

   * @param last_tic Last tic in seconds
   * @param frame Frame number
   * @returns 0 for success, -1 for failure
   */
  virtual int showFPS(double &last_tic, int &frame);

  /**
   * Show image

   * @param image Image
   * @returns 0 for success, -1 for failure
   */
  virtual int showImage(cv::Mat &image);
};

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_DRIVER_CAMERA_BASE_HPP
