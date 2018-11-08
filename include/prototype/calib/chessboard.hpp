/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
#define PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"
#include "prototype/vision/util.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

/**
 * Chessboard
 */
struct chessboard_t {
  bool ok = false;
  std::string config_file;

  int nb_rows = 10;
  int nb_cols = 10;
  double square_size = 0.1;
  std::vector<cv::Point3f> object_points;

  chessboard_t();
  chessboard_t(const std::string &config_file_);
  ~chessboard_t();
};

/**
 * Load config file
 *
 * @param[in,out] cb Chessboard
 * @param[in] config_file Path to config file
 * @returns 0 for success, -1 for failure
 */
int chessboard_load(chessboard_t &cb, const std::string &config_file);

/**
 * Create object points
 *
 * @param[in] cb Chessboard
 * @returns Vector of object points
 */
std::vector<cv::Point3f> chessboard_object_points(const chessboard_t &cb);

/**
 * Detect chessboard corners
 *
 * @param[in] cb Chessboard
 * @param[in] image Input image
 * @param[out] corners Detected chessboard corners
 * @returns 0 for success, -1 for failure
 */
int chessboard_detect(const chessboard_t &cb,
                      const cv::Mat &image,
                      std::vector<cv::Point2f> &corners);

/**
 * Draw chessboard corners
 *
 * @param[in] cb Chessboard
 * @param[in,out] image Image where chessboard corners are drawn
 * @returns 0 for success, -1 for failure
 */
int chessboard_draw_corners(const chessboard_t &cb, cv::Mat &image);

/**
 * Solve PnP between camera and chessboard
 *
 * @param[in] cb Chessboard
 * @param[in] corners Detected chessboard corners
 * @param[in] K Camera intrinsics matrix K
 * @param[out] T_c_t Transform from camera to calibration target
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
 * @param[in] cb Chessboard
 * @param[in] X Corner positions relative to camera in ideal coordinates
 * @param[in] K Camera intrinsics matrix K
 * @param[in,out] image Target image
 */
void chessboard_project_points(const chessboard_t &cb,
                               const matx_t &X,
                               const mat3_t &K,
                               cv::Mat &image);

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
