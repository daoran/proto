/**
 * @file
 * @defgroup vision vision
 * @ingroup util
 */
#ifndef PROTOTYPE_CORE_VISION_HPP
#define PROTOTYPE_CORE_VISION_HPP

#include <numeric>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "prototype/core/math.hpp"

namespace prototype {
/**
 * @addtogroup vision
 * @{
 */

/**
 * Compare `cv::Mat` whether they are equal
 *
 * @param m1 First matrix
 * @param m2 Second matrix
 * @returns true or false
 */
bool is_equal(const cv::Mat &m1, const cv::Mat &m2);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const cv::Mat &x, matx_t &y);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const matx_t &x, cv::Mat &y);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @returns Matrix as Eigen::Matrix
 */
matx_t convert(const cv::Mat &x);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @returns Matrix as cv::Mat
 */
cv::Mat convert(const matx_t &x);

/**
 * Convert x to homogenous coordinates
 *
 * @param x Input vector
 * @return Output vector in homogeneous coordinates
 */
vec3_t homogeneous(const vec2_t &x);

/**
 * Convert x to homogenous coordinates
 *
 * @param x Input vector
 * @returns Output vector in homogeneous coordinates
 */
vec4_t homogeneous(const vec3_t &x);

/**
 * Normalize vector of
 */
vec2_t normalize(const vec2_t &x);

/**
 * Convert rvec, tvec into transform matrix
 *
 * @param rvec Rodrigues rotation vector
 * @param tvec Translation vector
 * @returns Transform matrix
 */
mat4_t rvectvec2transform(const cv::Mat &rvec, const cv::Mat &tvec);

/**
 * Skew symmetric matrix
 *
 * @param v Vector
 * @returns Skew symmetric matrix
 */
cv::Matx33d skew(const cv::Vec3d &v);

/**
 * Sort Keypoints
 *
 * @param keypoints
 * @param limit
 * @returns Sorted keypoints by response
 */
std::vector<cv::KeyPoint> sort_keypoints(
    const std::vector<cv::KeyPoint> keypoints, const size_t limit = 0);

/**
 * Convert gray-scale image to rgb image
 *
 * @param image
 *
 * @returns RGB image
 */
cv::Mat gray2rgb(const cv::Mat &image);

/**
 * Convert rgb image to gray-scale image
 *
 * @param image
 *
 * @returns Gray-scale image
 */
cv::Mat rgb2gray(const cv::Mat &image);

/** @} group vision */
} //  namespace prototype
#endif // PROTOTYPE_CORE_VISION_HPP
