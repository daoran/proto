/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_CAMERA_PROPERTY_HPP
#define PROTOTYPE_CALIB_CAMERA_PROPERTY_HPP

#include <iostream>

#include "prototype/core.hpp"
#include "prototype/vision/camera/distortion.hpp"
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

struct CameraProperty {
  int camera_index;
  std::string camera_model;
  std::string distortion_model;
  vecx_t distortion_coeffs;
  vecx_t intrinsics;
  vec2_t resolution;

  CameraProperty();

  // Pinhole model w/o distortion constructor
  CameraProperty(const int camera_index,
                 const double fx,
                 const double fy,
                 const double cx,
                 const double cy,
                 const int image_width,
                 const int image_height);

  // Pinhole model w/o distortion constructor
  CameraProperty(const int camera_index, const mat3_t &K, const vec2_t &resolution);

  // Camera model and distortion model constructor
  CameraProperty(const int camera_index,
                 const std::string &camera_model,
                 const mat3_t &K,
                 const std::string &distortion_model,
                 const vecx_t &D,
                 const vec2_t &resolution);

  /**
   * Camera intrinsics matrix K
   * @returns Camera intrinsics matrix K
   */
  mat3_t K();

  /**
   * Distortion coefficients D
   * @returns Distortion coefficients D
   */
  vecx_t D();

  /**
   * Undistort points
   *
   * @param image_points Image points [px]
   * @param rect_mat Rectification matrix
   * @returns Undistorted image points [ideal]
   */
  std::vector<cv::Point2f>
  undistortPoints(const std::vector<cv::Point2f> &image_points,
                  const mat3_t &rect_mat = I(3));

  /**
   * Undistort point
   *
   * @param image_point Image point [px]
   * @param rect_mat Rectification matrix
   * @returns Undistorted image point [ideal]
   */
  cv::Point2f undistortPoint(const cv::Point2f &image_point,
                             const mat3_t &rect_mat = I(3));

  /**
   * Distort points
   *
   * @param image_points Image points [ideal]
   * @returns Distorted image points [px]
   */
  std::vector<cv::Point2f>
  distortPoints(const std::vector<cv::Point2f> &points);

  /**
   * Distort point
   *
   * @param image_points Image points [ideal]
   * @returns Distorted image points [px]
   */
  cv::Point2f distortPoint(const cv::Point2f &points);

  /**
   * Undistort image
   *
   * @param image Image
   * @param balance Balance (between 0.0 and 1.0)
   * @param K_ud Camera intrinsic matrix for undistorted image
   *
   * @returns Undistorted image
   */
  cv::Mat undistortImage(const cv::Mat &image,
                         const double balance,
                         cv::Mat &K_ud);

  /**
   * Undistort image
   *
   * @param image Image
   * @param balance Balance (between 0.0 and 1.0)
   * @param image_ud Undistorted image
   *
   * @return 0 for success, -1 for failure
   */
  cv::Mat undistortImage(const cv::Mat &image, const double balance);

  /**
   * Project 3D points to image plane
   *
   * @param X 3D points
   * @returns pixels Pixel point in image plane [px]
   */
  matx_t project(const matx_t &X);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point [ideal]
   * @param pixel Point in image plane [px]
   * @return 0 for success, -1 for failure
   */
  vec2_t project(const vec3_t &X);
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CAMERA_PROPERTY_HPP
