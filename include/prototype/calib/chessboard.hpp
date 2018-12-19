#ifndef PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
#define PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core/core.hpp"
#include "prototype/vision/util.hpp"

namespace proto {

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
 * Configure chessboard.
 * @returns 0 for success, -1 for failure
 */
int chessboard_configure(chessboard_t &cb, const std::string &config_file);

/**
 * Create object points.
 * @returns Vector of object points
 */
std::vector<cv::Point3f> chessboard_object_points(const chessboard_t &cb);

/**
 * Detect chessboard corners.
 * @returns 0 for success, -1 for failure
 */
int chessboard_detect(const chessboard_t &cb,
                      const cv::Mat &image,
                      std::vector<cv::Point2f> &corners);

/**
 * Draw chessboard corners.
 * @returns 0 for success, -1 for failure
 */
int chessboard_draw(const chessboard_t &cb, cv::Mat &image);

/**
 * Solve PnP between camera and chessboard.
 * @returns 0 for success, -1 for failure
 */
int chessboard_solvepnp(const chessboard_t &cb,
                        const std::vector<cv::Point2f> corners,
                        const mat3_t &K,
                        mat4_t &T_CF,
                        matx_t &X);

/**
 * Project 3D points to image. Where the corner positoins relative to the
 * camera is in *ideal coordinates*, and `K` is the camera matrix.
 */
void chessboard_project_points(const chessboard_t &cb,
                               const mat3_t &K,
                               const matx_t &X,
                               cv::Mat &image);

} //  namespace proto
#endif // PROTOTYPE_VISION_CALIBRATION_CHESSBOARD_HPP
