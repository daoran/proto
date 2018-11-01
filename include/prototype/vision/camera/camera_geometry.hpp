/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
#define PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_HPP

#include <iostream>

#include "prototype/core.hpp"
#include "prototype/vision/camera/pinhole.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Undistort points
 */
void undistort_points(
    const std::vector<cv::Point2f> &pts_in,
    const cv::Vec4d &intrinsics,
    const std::string &distortion_model,
    const cv::Vec4d &distortion_coeffs,
    std::vector<cv::Point2f> &pts_out,
    const cv::Matx33d &rectification_matrix = cv::Matx33d::eye(),
    const cv::Vec4d &new_intrinsics = cv::Vec4d(1, 1, 0, 0));

/**
 * Distort points
 */
std::vector<cv::Point2f> distortPoints(const std::vector<cv::Point2f> &pts_in,
                                       const cv::Vec4d &intrinsics,
                                       const std::string &distortion_model,
                                       const cv::Vec4d &distortion_coeffs);

/**
 * Camera geometry
 */
template <typename CM, typename DM>
struct camera_geometry_t {
  int camera_index = 0;
  CM camera_model;
  DM distortion_model;

  camera_geometry_t(const CM &camera_model_,
                    const DM &distortion_model_);
  ~camera_geometry_t();
};

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::camera_geometry_t(
    const CM &camera_model_,
    const DM &distortion_model_)
    : camera_model{camera_model_}, distortion_model{distortion_model_} {}

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::~camera_geometry_t() {}

/**
 * Project point to image plane in pixels
 *
 * @param[in] cam Camera geometry
 * @param[in] point Point
 * @returns Point to image plane projection in pixel coordinates
 */
template <typename CM, typename DM>
vec2_t camera_geometry_project(
    const camera_geometry_t<CM, DM> &cam,
    const vec3_t &point) {
  const vec3_t point_distorted = distort(cam.distortion_model, point);
  return project(cam.camera_model, point_distorted);
}

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
