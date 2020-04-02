#ifndef PROTO_VISION_HPP
#define PROTO_VISION_HPP

#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "proto/core/core.hpp"
#include "proto/vision/vision.hpp"

namespace proto {

/*****************************************************************************
 *                            VISION COMMON
 ****************************************************************************/

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
            const real_t cx,
            const real_t cy);

/**
 * Compare two keypoints based on the response.
 *
 * @param[in] kp1 First keypoint
 * @param[in] kp2 Second keypoint
 * @returns Boolean to denote if first keypoint repose is larger than second
 */
bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2);

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
 * Rescale points
 *
 * @param[in] pts1 Points 1
 * @param[in] pts2 Points 2
 * @returns scaling_factor Scaling factor
 */
float rescale_points(vec2s_t &pts1, std::vector<vec2_t> &pts2);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected);

/**
 * Calculate reprojection error
 *
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @returns Reprojection error
 */
real_t reprojection_error(const std::vector<cv::Point2f> &measured,
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
 * @param[in] balance Balance
 * @param[in,out] Knew New camera matrix K
 *
 * @returns Undistorted image using equidistant distortion
 */
cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew);
/**
 * Illumination invariant transform.
 *
 * @param[in] image Image
 * @param[in] lambda_1 Lambad 1
 * @param[in] lambda_2 Lambad 2
 * @param[in] lambda_3 Lambad 3
 */
void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3);

// /**
//  * Outlier rejection using essential matrix
//  *
//  * @param[in] cam0 Camera 0 geometry
//  * @param[in] cam1 Camera 1 geometry
//  * @param[in] T_cam1_cam0 Transform between cam1 and cam0
//  * @param[in] cam0_points Points observed from camera 0
//  * @param[in] cam1_points Points observed from camera 1
//  * @param[in] threshold Threshold
//  * @param[out] inlier_markers
//  */
// template <typename CAMERA_MODEL, typename DISTORTION_MODEL>
// void essential_matrix_outlier_rejection(
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam0,
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam1,
//     const mat4_t &T_cam1_cam0,
//     const std::vector<cv::Point2f> &cam0_points,
//     const std::vector<cv::Point2f> &cam1_points,
//     const real_t threshold,
//     std::vector<uchar> &inlier_markers);

// template <typename CAMERA_MODEL, typename DISTORTION_MODEL>
// void essential_matrix_outlier_rejection(
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam0,
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam1,
//     const mat4_t &T_cam1_cam0,
//     const std::vector<cv::Point2f> &cam0_points,
//     const std::vector<cv::Point2f> &cam1_points,
//     const real_t threshold,
//     std::vector<uchar> &inlier_markers) {
//   // Remove outliers using essential matrix
//   // -- Compute the relative rotation between the cam0 frame and cam1 frame
//   const cv::Matx33d R_cam1_cam0 = convert(T_cam1_cam0.block(0, 0, 3, 3));
//   const cv::Vec3d t_cam0_cam1 = convert(T_cam1_cam0.block(0, 3, 3, 1));
//   // -- Compute the essential matrix
//   const cv::Matx33d E = skew(t_cam0_cam1) * R_cam1_cam0;
//   // -- Calculate norm pixel unit
//   const real_t cam0_fx = cam0.camera_model.fx;
//   const real_t cam0_fy = cam0.camera_model.fy;
//   const real_t cam1_fx = cam1.camera_model.fx;
//   const real_t cam1_fy = cam1.camera_model.fy;
//   const real_t norm_pixel_unit = 4.0 / (cam0_fx + cam0_fy + cam1_fx +
//   cam1_fy);
//   // -- Further remove outliers based on essential matrix
//   // std::vector<cv::Point2f> cam0_points_ud = undistort(cam0_points);
//   // std::vector<cv::Point2f> cam1_points_ud = undistort(cam1_points);
//   // std::vector<cv::Point2f> cam0_points_ud =
//   // cam0.undistortPoints(cam0_points); std::vector<cv::Point2f>
//   cam1_points_ud
//   // = cam1.undistortPoints(cam1_points); for (size_t i = 0; i <
//   // cam0_points_ud.size(); i++) {
//   //   if (inlier_markers[i] == 0) {
//   //     continue;
//   //   }
//   //
//   //   const cv::Vec3d pt0(cam0_points_ud[i].x, cam0_points_ud[i].y, 1.0);
//   //   const cv::Vec3d pt1(cam1_points_ud[i].x, cam1_points_ud[i].y, 1.0);
//   //   const cv::Vec3d el = E * pt0;
//   //   real_t err = fabs((pt1.t() * el)[0]) / sqrt(el[0] * el[0] + el[1] *
//   //   el[1]); if (err > threshold * norm_pixel_unit) {
//   //     inlier_markers[i] = 0;
//   //   }
//   // }
// }

/*****************************************************************************
 *                                DRAW
 ****************************************************************************/

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw tracks
 *
 * @param[in] img_cur Current image frame
 * @param[in] p0 Previous corners
 * @param[in] p1 Current corners
 * @param[in] status Corners status
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Image frame 0
 * @param[in] img1 Image frame 1
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] status Inlier vector
 *
 * @returns Image with feature matches between frame 0 and 1
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status);

/**
 * Draw matches
 *
 * @param[in] img0 Previous image frame
 * @param[in] img1 Current image frame
 * @param[in] k0 Previous keypoints
 * @param[in] k1 Current keypoints
 * @param[in] matches Feature matches
 *
 * @returns Image with feature matches between previous and current frame
 */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features);

/**
 * Draw grid features
 *
 * @param[in] image Image frame
 * @param[in] grid_rows Grid rows
 * @param[in] grid_cols Grid cols
 * @param[in] features List of features
 *
 * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features);

/*****************************************************************************
 *                              FEATURES2D
 ****************************************************************************/

/**
 * Grid fast
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] threshold Fast threshold
 * @param[in] nonmax_suppression Nonmax Suppression
 *
 * @returns List of keypoints
 */
std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int max_corners = 100,
                                    const int grid_rows = 5,
                                    const int grid_cols = 5,
                                    const real_t threshold = 10.0,
                                    const bool nonmax_suppression = true);

/**
 * Grid good
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] quality_level Quality level
 * @param[in] min_distance Min distance
 * @param[in] mask Mask
 * @param[in] block_size Block size
 * @param[in] use_harris_detector Use Harris detector
 * @param[in] k Free parameter for Harris detector
 *
 * @returns List of points
 */
std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const real_t quality_level = 0.01,
                                   const real_t min_distance = 10,
                                   const cv::Mat mask = cv::Mat(),
                                   const int block_size = 3,
                                   const bool use_harris_detector = false,
                                   const real_t k = 0.04);

/****************************************************************************
 *                            RADIAL-TANGENTIAL
 ***************************************************************************/

/**
 * Radial-tangential distortion
 */
struct radtan4_t {
  real_t k1 = 0.0;
  real_t k2 = 0.0;
  real_t p1 = 0.0;
  real_t p2 = 0.0;
  real_t *data[4] = {&k1, &k2, &p1, &p2};

  radtan4_t();
  radtan4_t(const real_t *distortion_);
  radtan4_t(const vec4_t &distortion_);
  radtan4_t(const real_t k1_,
            const real_t k2_,
            const real_t p1_,
            const real_t p2_);
  radtan4_t(radtan4_t &radtan);
  radtan4_t(const radtan4_t &radtan);
  ~radtan4_t();

  vec2_t distort(const vec2_t &p);
  vec2_t distort(const vec2_t &p) const;

  mat2_t J_point(const vec2_t &p);
  mat2_t J_point(const vec2_t &p) const;

  mat_t<2, 4> J_param(const vec2_t &p);
  mat_t<2, 4> J_param(const vec2_t &p) const;

  void operator=(const radtan4_t &src) throw();
};

/**
 * Create Radial-tangential distortion vector
 */
template <typename T>
Eigen::Matrix<T, 4, 1> radtan4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T p1 = distortion[2];
  const T p2 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, p1, p2};
  return D;
}

/**
 * Type to output stream.
 */
std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4);

/**
 * Return distortion coefficients of a Radial-Tangential distortion
 */
vec4_t distortion_coeffs(const radtan4_t &radtan);

/**
 * Distort points with the radial-tangential distortion model.
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] point Point
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point);

/**
 * Distort 3D points with the radial-tangential distortion model.
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] point Point
 * @param[out] J_point Jacobian of distorted point w.r.t. projection point
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point, mat2_t &J_point);

/**
 * Distort 3D points with the radial-tangential distortion model.
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] point Point
 * @param[out] J_point Jacobian of distorted point w.r.t. projection point
 * @param[out] J_radtan Jacobian of distorted point w.r.t. radtan params
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan,
               const vec2_t &point,
               mat2_t &J_point,
               mat_t<2, 4> &J_params);

/**
 * Distort 3D points with the radial-tangential distortion model.
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] points Points
 * @returns Distorted points
 */
matx_t distort(const radtan4_t &radtan, const matx_t &points);

/**
 * Undistort point.
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] p0 Distorted point
 * @param[in] max_iter Max iteration
 * @returns Undistorted point
 */
vec2_t undistort(const radtan4_t &radtan,
                 const vec2_t &p0,
                 const int max_iter = 5);

/****************************************************************************
 *                              EQUI-DISTANCE
 ***************************************************************************/

/**
 * Equi-distant distortion
 */
struct equi4_t {
  real_t k1 = 0.0;
  real_t k2 = 0.0;
  real_t k3 = 0.0;
  real_t k4 = 0.0;
  real_t *data[4] = {&k1, &k2, &k3, &k4};

  equi4_t(const real_t k1_,
          const real_t k2_,
          const real_t k3_,
          const real_t k4_);
  ~equi4_t();
};

/**
 * Create Equidistant distortion vector
 */
template <typename T>
Eigen::Matrix<T, 4, 1> equi4_D(const T *distortion) {
  const T k1 = distortion[0];
  const T k2 = distortion[1];
  const T k3 = distortion[2];
  const T k4 = distortion[3];
  Eigen::Matrix<T, 4, 1> D{k1, k2, k3, k4};
  return D;
}

/**
 * Type to output stream.
 */
std::ostream &operator<<(std::ostream &os, const equi4_t &equi4);

/**
 * Distort point with equi-distant distortion model.
 *
 * @param[in] equi Equi-distance parameters
 * @param[in] point Point
 * @returns Distorted point
 */
vec2_t distort(const equi4_t &equi, const vec2_t &point);

/**
 * Distort point with equi-distant distortion model.
 *
 * @param[in] equi Equi-distance parameters
 * @param[in] point Point
 * @param[out] J_point Jacobian of equi w.r.t. point
 * @returns Distorted point
 */
vec2_t distort(const equi4_t &equi, const vec2_t &point, mat2_t &J_point);

/**
 * Distort point with equi-distant distortion model.
 *
 * @param[in] equi Equi-distance parameters
 * @param[in] points Points
 * @returns Distorted points
 */
matx_t distort(const equi4_t &equi, const matx_t &points);

/**
 * Un-distort a 2D point with the equi-distant distortion model.
 */
vec2_t undistort(const equi4_t &equi, const vec2_t &p);

/****************************************************************************
 *                               PINHOLE
 ***************************************************************************/

/**
 * Pinhole camera model
 */
struct pinhole_t {
  real_t fx = 0.0;
  real_t fy = 0.0;
  real_t cx = 0.0;
  real_t cy = 0.0;
  real_t *data[4] = {&fx, &fy, &cx, &cy};

  pinhole_t();
  pinhole_t(const real_t *intrinsics);
  pinhole_t(const vec4_t &intrinsics);
  pinhole_t(const mat3_t &K);
  pinhole_t(const real_t fx_,
            const real_t fy_,
            const real_t cx_,
            const real_t cy_);
  pinhole_t(pinhole_t &pinhole);
  pinhole_t(const pinhole_t &pinhole);
  ~pinhole_t();

  vec2_t project(const vec2_t &p);
  vec2_t project(const vec2_t &p) const;

  mat2_t J_point();
  mat2_t J_point() const;

  mat_t<2, 4> J_param(const vec2_t &p);
  mat_t<2, 4> J_param(const vec2_t &p) const;

  void operator=(const pinhole_t &src) throw();
};

/**
 * `pinhole_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const pinhole_t &pinhole);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] fx Focal length in x-axis
 * @param[in] fy Focal length in y-axis
 * @param[in] cx Principal center in x-axis
 * @param[in] cy Principal center in y-axis
 *
 * @returns Camera matrix K
 */
mat3_t
pinhole_K(const real_t fx, const real_t fy, const real_t cx, const real_t cy);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] pinhole Pinhole camera
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const pinhole_t &pinhole);

/**
 * Pinhole camera matrix K
 */
template <typename T>
static Eigen::Matrix<T, 3, 3> pinhole_K(const T *intrinsics) {
  const T fx = intrinsics[0];
  const T fy = intrinsics[1];
  const T cx = intrinsics[2];
  const T cy = intrinsics[3];

  // clang-format off
  Eigen::Matrix<T, 3, 3> K;
  K << fx, T(0.0), cx,
       T(0.0), fy, cy,
       T(0.0), T(0.0), T(1.0);
  // clang-format on
  return K;
}

/**
 * Form **theoretical** pinhole camera matrix K
 *
 * @param[in] image_size Image width and height [px]
 * @param[in] lens_hfov Lens horizontal field of view [deg]
 * @param[in] lens_vfov Lens vertical field of view [deg]
 *
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const vec2_t &image_size,
                 const real_t lens_hfov,
                 const real_t lens_vfov);

/**
 * Form pinhole projection matrix P
 *
 * @param[in] K Camera matrix K
 * @param[in] C_WC Camera rotation matrix in world frame
 * @param[in] r_WC Camera translation vector in world frame
 * @returns Camera projection matrix P
 */
mat34_t pinhole_P(const mat3_t &K, const mat3_t &C_WC, const vec3_t &r_WC);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param[in] image_width Image width [px]
 * @param[in] fov Field of view [deg]
 * @returns Focal length in pixels
 */
real_t pinhole_focal_length(const int image_width, const real_t fov);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param[in] image_size Image width and height [px]
 * @param[in] hfov Horizontal field of view [deg]
 * @param[in] vfov Vertical field of view [deg]
 * @returns Focal length in pixels
 */
vec2_t pinhole_focal_length(const vec2_t &image_size,
                            const real_t hfov,
                            const real_t vfov);

/**
 * Project 3D point to image plane (not in pixels)
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point in 3D
 * @returns Point in image plane (not in pixels)
 */
vec2_t project(const vec3_t &p);

/**
 * Project 3D point to image plane (not in pixels)
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point in 3D
 * @param[out] J_P Project Jacobian.
 * @returns Point in image plane (not in pixels)
 */
vec2_t project(const vec3_t &p, mat_t<2, 3> &J_P);

/**
 * Scale and center projected point to pixel coordinates
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &pinhole, const vec2_t &p);

/**
 * Project 3D point, scale and center to pixel coordinates
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point in 3D
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &pinhole, const vec3_t &p);

/**
 * Project 3D point, scale and center to pixel coordinates
 *
 * @param[in] pinhole Pinhole camera model.
 * @param[in] p Point in 3D.
 * @param[out] J_h Measurement Jacobian.
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &model, const vec3_t &p, mat_t<2, 3> &J_h);

template <typename CM, typename DM>
int project(const int img_w,
            const int img_h,
            const CM &cam_model,
            const DM &dist_model,
            const vec3_t &p_C,
            vec2_t &z_hat) {
  // Check validity of the point, simple depth test.
  const real_t x = p_C(0);
  const real_t y = p_C(1);
  const real_t z = p_C(2);
  if (fabs(z) < 0.05) {
    return -1;
  }

  // Project, distort and then scale and center
  const vec2_t p{x / z, y / z};
  const vec2_t p_dist = dist_model.distort(p);
  z_hat = cam_model.project(p_dist);

  // Check projection
  const bool x_ok = (z_hat(0) >= 0 && z_hat(0) <= img_w);
  const bool y_ok = (z_hat(1) >= 0 && z_hat(1) <= img_h);
  if (x_ok == false || y_ok == false) {
    return -2;
  }

  return 0;
}

template <typename CM, typename DM>
int project(const int img_w,
            const int img_h,
            const CM &cam_model,
            const DM &dist_model,
            const vec3_t &p_C,
            vec2_t &z_hat,
            mat_t<2, 3> &J_h) {
  int retval = project(img_w, img_h, cam_model, dist_model, p_C, z_hat);
  if (retval != 0) {
    return retval;
  }

  // Projection Jacobian
  const real_t x = p_C(0);
  const real_t y = p_C(1);
  const real_t z = p_C(2);
  mat_t<2, 3> J_proj = zeros(2, 3);
  J_proj(0, 0) = 1.0 / z;
  J_proj(1, 1) = 1.0 / z;
  J_proj(0, 2) = -x / (z * z);
  J_proj(1, 2) = -y / (z * z);

  // Measurement Jacobian
  const vec2_t p{x / z, y / z};
  J_h = cam_model.J_point() * dist_model.J_point(p) * J_proj;

  return 0;
}

/****************************************************************************
 *                            CAMERA GEOMETRY
 ***************************************************************************/

/**
 * Camera geometry
 */
template <typename CM, typename DM>
struct camera_geometry_t {
  int camera_index = 0;
  CM camera_model;
  DM distortion_model;

  camera_geometry_t();
  camera_geometry_t(const CM &camera_model_, const DM &distortion_model_);
  ~camera_geometry_t();
};

/**
 * Pinhole Radial-Tangential Camera Geometry
 */
typedef camera_geometry_t<pinhole_t, radtan4_t> pinhole_radtan4_t;

/**
 * Pinhole Equi Camera Geometry
 */
typedef camera_geometry_t<pinhole_t, equi4_t> pinhole_equi4_t;

/**
 * Project point to image plane in pixels
 *
 * @param[in] cam Camera geometry
 * @param[in] point Point
 * @returns Point to image plane projection in pixel coordinates
 */
template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &point);

/**
 * Project point to image plane in pixels
 *
 * @param[in] cam Camera geometry
 * @param[in] p_C 3D Point observed from camera frame
 * @param[out] J_h Measurement model jacobian
 * @returns Point to image plane projection in pixel coordinates
 */
template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &p_C,
                               matx_t &J_h);

/**
 * Project point to image plane in pixels
 *
 * @param[in] cam Camera geometry
 * @param[in] p_C 3D Point observed from camera frame
 * @param[out] J_h Measurement model jacobian
 * @param[out] J_params jacobian
 * @returns Point to image plane projection in pixel coordinates
 */
template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &p_C,
                               matx_t &J_h,
                               matx_t &J_params);

/**
 * Project point using pinhole radial-tangential
 */
template <typename T>
int pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                            const Eigen::Matrix<T, 4, 1> &D,
                            const Eigen::Matrix<T, 3, 1> &point,
                            Eigen::Matrix<T, 2, 1> &image_point);

/**
 * Project point using pinhole equidistant
 */
template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_equi4_project(const Eigen::Matrix<T, 3, 3> &K,
                      const Eigen::Matrix<T, 4, 1> &D,
                      const Eigen::Matrix<T, 3, 1> &point);

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::camera_geometry_t() {}

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::camera_geometry_t(const CM &camera_model_,
                                             const DM &distortion_model_)
    : camera_model{camera_model_}, distortion_model{distortion_model_} {}

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::~camera_geometry_t() {}

template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &point) {
  const vec2_t p{point(0) / point(2), point(1) / point(2)};
  const vec2_t point_distorted = distort(cam.distortion_model, p);
  return project(cam.camera_model, point_distorted);
}

template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &p_C,
                               matx_t &J_h) {
  mat_t<2, 3> J_P;
  mat2_t J_K;
  mat2_t J_D;

  const vec2_t p = project(p_C, J_P);
  const vec2_t p_distorted = distort(cam.distortion_model, p, J_D);
  const vec2_t pixel = project(cam.camera_model, p_distorted);

  J_h = J_K * J_D * J_P;

  return pixel;
}

template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &p_C,
                               matx_t &J_h,
                               matx_t &J_params) {
  mat_t<2, 3> J_P;
  mat2_t J_K;
  mat2_t J_D;

  const vec2_t p = project(p_C, J_P);
  const vec2_t p_distorted = distort(cam.distortion_model, p, J_D, J_params);
  const vec2_t pixel = project(cam.camera_model, p_distorted);

  J_h = J_K * J_D * J_P;

  return pixel;
}

template <typename T>
int pinhole_radtan4_project(const Eigen::Matrix<T, 8, 1> &params,
                            const Eigen::Matrix<T, 3, 1> &point,
                            Eigen::Matrix<T, 2, 1> &image_point) {
  // Check for singularity
  const T z_norm = sqrt(point(2) * point(2)); // std::abs doesn't work for all T
  if ((T) z_norm < (T) 1.0e-12) {
    return -1;
  }

  // Extract intrinsics params
  const T fx = params(0);
  const T fy = params(1);
  const T cx = params(2);
  const T cy = params(3);
  const T k1 = params(4);
  const T k2 = params(5);
  const T p1 = params(6);
  const T p2 = params(7);

  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);
  // const T fx = K(0, 0);
  // const T fy = K(1, 1);
  // const T cx = K(0, 2);
  // const T cy = K(1, 2);
  // const T x = fx * (point(0) / point(2)) + cx;
  // const T y = fy * (point(1) / point(2)) + cy;

  // // Scale and center
  // const T fx = K(0, 0);
  // const T fy = K(1, 1);
  // const T cx = K(0, 2);
  // const T cy = K(1, 2);
  // x = fx * x + cx;
  // y = fy * y + cy;

  // Apply Radial distortion factor
  const T x2 = x * x;
  const T y2 = y * y;
  const T r2 = x2 + y2;
  const T r4 = r2 * r2;
  const T radial_factor = T(1) + (k1 * r2) + (k2 * r4);
  const T x_dash = x * radial_factor;
  const T y_dash = y * radial_factor;

  // Apply Tangential distortion factor
  const T xy = x * y;
  const T x_ddash = x_dash + (T(2) * p1 * xy + p2 * (r2 + T(2) * x2));
  const T y_ddash = y_dash + (p1 * (r2 + T(2) * y2) + T(2) * p2 * xy);

  // Scale and center
  image_point(0) = fx * x_ddash + cx;
  image_point(1) = fy * y_ddash + cy;

  // // Set result
  // image_point(0) = x_ddash;
  // image_point(1) = y_ddash;

  if (point(2) > T(0.0)) {
    return 0; // Point is infront of camera
  } else {
    return 1; // Point is behind camera
  }
}

template <typename T>
int pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                            const Eigen::Matrix<T, 4, 1> &D,
                            const Eigen::Matrix<T, 3, 1> &point,
                            Eigen::Matrix<T, 2, 1> &image_point) {
  Eigen::Matrix<T, 8, 1> params;
  params << K(0, 0);  // fx
  params << K(1, 1);  // fy
  params << K(0, 2);  // cx
  params << K(1, 2);  // cy
  params << D(0);     // k1
  params << D(1);     // k2
  params << D(2);     // p1
  params << D(3);     // p2

  return pinhole_radtan4_project(params, point, image_point);
}

template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_equi4_project(const Eigen::Matrix<T, 8, 1> &params,
                      const Eigen::Matrix<T, 3, 1> &point) {
  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

  // Pinhole params
  const T fx = params(0);
  const T fy = params(1);
  const T cx = params(2);
  const T cy = params(3);

  // Radial distortion params
  const T k1 = params(4);
  const T k2 = params(5);
  const T k3 = params(6);
  const T k4 = params(7);
  const T r = sqrt(pow(x, 2) + pow(y, 2));
  // if (r < 1e-8) {
  //   return point;
  // }

  // Apply equi distortion
  const T th = atan(r);
  const T th2 = th * th;
  const T th4 = th2 * th2;
  const T th6 = th4 * th2;
  const T th8 = th4 * th4;
  const T th_d = th * (T(1) + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const T x_dash = (th_d / r) * x;
  const T y_dash = (th_d / r) * y;

  // Scale distorted point
  const Eigen::Matrix<T, 2, 1> pixel{fx * x_dash + cx, fy * y_dash + cy};

  return pixel;
}

template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_equi4_project(const Eigen::Matrix<T, 3, 3> &K,
                      const Eigen::Matrix<T, 4, 1> &D,
                      const Eigen::Matrix<T, 3, 1> &point) {
  Eigen::Matrix<T, 8, 1> params;
  params << K(0, 0);  // fx
  params << K(1, 1);  // fy
  params << K(0, 2);  // cx
  params << K(1, 2);  // cy
  params << D(0);     // k1
  params << D(1);     // k2
  params << D(2);     // p1
  params << D(3);     // p2

  return pinhole_equi4_project(params, point, point);
}

} //  namespace proto
#endif // PROTO_VISION_HPP
