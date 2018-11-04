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
 */
static bool keypoint_compare_by_response(const cv::KeyPoint &pt1,
                                         const cv::KeyPoint &pt2) {
  // Keypoint with higher response will be at the beginning of the vector
  return pt1.response > pt2.response;
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
 * @param pts1 Points 1
 * @param pts2 Points 2
 * @returns scaling_factor Scaling factor
 */
float rescale_points(std::vector<cv::Point2f> &pts1,
                     std::vector<cv::Point2f> &pts2);

/**
 * Two point ransac algorithm to mark the inliers in the input set.
 *
 * @param pts1 First set of points.
 * @param pts2 Second set of points.
 * @param R_p_c: Rotation matrix from previous to current camera frame
 * @param intrinsics: Camera intrinsics
 * @param distortion_model: Distortion model of the camera.
 * @param distortion_coeffs: Distortion coefficients.
 * @param inlier_error: Acceptable error to be considered as an inlier
 * @param success_probability Required probability of success
 *
 * @returns Vector of ints, 1 for inliers and 0 for outliers
 */
// void two_point_ransac(const std::vector<cv::Point2f> &pts1,
//                       const std::vector<cv::Point2f> &pts2,
//                       const cv::Matx33f &R_p_c,
//                       camera_geometry_t &cam,
//                       const double &inlier_error,
//                       const double &success_probability,
//                       std::vector<int> &inlier_markers);

/**
 * Create feature mask
 *
 * @param image_width Image width
 * @param image_height Image height
 * @param points Points
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
 * @param image_width Image width
 * @param image_height Image height
 * @param keypoints Keypoints
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
 * @param image_width Image width
 * @param image_height Image height
 * @param points Points
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
 * @param image_width Image width
 * @param image_height Image height
 * @param keypoints Keypoints
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
