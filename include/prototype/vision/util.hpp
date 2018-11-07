#ifndef PROTOTYPE_VISION_UTIL_HPP
#define PROTOTYPE_VISION_UTIL_HPP

#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "prototype/core.hpp"
#include "prototype/vision/camera/camera_geometry.hpp"

namespace prototype {

/**
 * Create ROI from an image
 *
 * @param[in] image Input image
 * @param[in] width ROI width
 * @param[in] height ROI height
 * @param[in] cx ROI center x-axis
 * @param[in] cy ROI center y-axis
 *
 * @returns ROI
 */
cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const double cx,
            const double cy);

/**
 * Compare two keypoints based on the response.
 *
 * @param[in] kp1 First keypoint
 * @param[in] kp2 Second keypoint
 * @returns Boolean to denote if first keypoint repose is larger than second
 */
static bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                         const cv::KeyPoint &kp2) {
  // Keypoint with higher response will be at the beginning of the vector
  return kp1.response > kp2.response;
}

/**
 * Check to see if rotation matrix is valid (not singular)
 *
 * @param[in] R Rotation matrix
 * @returns Boolean to denote whether rotation matrix is valid
 */
bool is_rot_mat(const cv::Mat &R);

/**
 * Convert rotation matrix to euler angles
 *
 * @param[in] R Rotation matrix
 * @returns Euler angles
 */
cv::Vec3f rot2euler(const cv::Mat &R);

/**
 * Outlier rejection using essential matrix
 *
 * @param[in] cam0 Camera 0 geometry
 * @param[in] cam1 Camera 1 geometry
 * @param[in] T_cam1_cam0 Transform between cam1 and cam0
 * @param[in] cam0_points Points observed from camera 0
 * @param[in] cam1_points Points observed from camera 1
 * @param[in] threshold Threshold
 * @param[out] inlier_markers
 */
template <typename CAMERA_MODEL, typename DISTORTION_MODEL>
void essential_matrix_outlier_rejection(
    const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam0,
    const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam1,
    const mat4_t &T_cam1_cam0,
    const std::vector<cv::Point2f> &cam0_points,
    const std::vector<cv::Point2f> &cam1_points,
    const double threshold,
    std::vector<uchar> &inlier_markers);

/**
 * Rescale points
 *
 * @param[in] pts1 Points 1
 * @param[in] pts2 Points 2
 * @returns scaling_factor Scaling factor
 */
float rescale_points(std::vector<vec2_t> &pts1,
                     std::vector<vec2_t> &pts2);

/**
  * Calculate reprojection error
  *
  * @param[in] measured Measured image pixels
  * @param[in] projected Projected image pixels
  * @returns Reprojection error
  */
double reprojection_error(const std::vector<vec2_t> &measured,
                          const std::vector<vec2_t> &projected);

/**
  * Calculate reprojection error
  *
  * @param[in] measured Measured image pixels
  * @param[in] projected Projected image pixels
  * @returns Reprojection error
  */
double reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] points Points
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width);

/**
 * Create feature mask
 *
 * @param[in] image_width Image width
 * @param[in] image_height Image height
 * @param[in] keypoints Keypoints
 * @param[in] patch_width Patch width
 *
 * @returns Feature mask
 */
cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 *
 * @returns Undistorted image using radial-tangential distortion
 */
cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image);

/**
 * Equi undistort image
 *
 * @param[in] K Camera matrix K
 * @param[in] D Distortion vector D
 * @param[in] image Input image
 * @param[in,out] Knew New camera matrix K
 * @param[in] balance Balance
 *
 * @returns Undistorted image using equidistant distortion
 */
cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             cv::Mat &Knew,
                             const double balance);

} //  namespace prototype
#endif // PROTOTYPE_VISION_UTIL_HPP
