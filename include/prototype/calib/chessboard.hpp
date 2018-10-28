/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
#define PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Chessboard
 */
struct chessboard_t {
  bool ok = false;

  int nb_rows = 10;
  int nb_cols = 10;
  double square_size = 0.1;
  std::vector<cv::Point3f> object_points;

  chessboard_t();
  virtual ~chessboard_t();
};

/**
 * Load config file
 *
 * @param config_file Path to config file
 * @returns 0 for success, -1 for failure
 */
int chessboard_load(chessboard_t &cb, const std::string &config_file);

/**
 * Create object points
 * @returns Vector of object points
 */
std::vector<cv::Point3f> chessboard_object_points(const chessboard_t &cb);

/**
 * Detect chessboard corners
 *
 * @param image Input image
 * @param corners Detected chessboard corners
 * @returns 0 for success, -1 for failure
 */
int chessboard_detect(const chessboard_t &cb,
                      const cv::Mat &image,
                      std::vector<cv::Point2f> &corners);

/**
 * Draw chessboard corners
 * @param image Image where chessboard corners are drawn
 * @returns 0 for success, -1 for failure
 */
int chessboard_draw_corners(const chessboard_t &cb, cv::Mat &image);

/**
 * Solve PnP between camera and chessboard
 *
 * @param corners Detected chessboard corners
 * @param K Camera intrinsics matrix K
 * @param T_c_t Transform from camera to calibration target
 *
 * @returns 0 for success, -1 for failure
 */
int chessboard_solvepnp(const chessboard_t &cb,
                        const std::vector<cv::Point2f> corners,
                        const mat3_t &K,
                        mat4_t &T_c_t);

/**
 * Project 3D points to image
 *
 * @param X Corner positions relative to camera in ideal coordinates
 * @param K Camera intrinsics matrix K
 * @param image Target image
 */
void chessboard_project_points(const chessboard_t &cb,
                               const matx_t &X,
                               const mat3_t &K,
                               cv::Mat &image);

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
