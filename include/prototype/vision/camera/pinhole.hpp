/**
 * @file
 * @ingroup camera
 */
#ifndef PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
#define PROTOTYPE_VISION_CAMERA_PINHOLE_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Pinhole camera model
 */
struct pinhole_t {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  mat3_t K = zeros(3, 3);

  pinhole_t();
  pinhole_t(const vec4_t &intrinsics);
  pinhole_t(const double fx_,
            const double fy_,
            const double cx_,
            const double cy_);
  ~pinhole_t();
};

/**
 * Project point to pixel coordinates
 *
 * @param[in] model Camera model
 * @param[in] X Point in 3D
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &model, const vec3_t &X);

/**
 * Form projection matrix
 *
 * @param[in] model Camera model
 * @param[in] R Rotation matrix
 * @param[in] t Translation vector
 * @returns 3x4 Projection matrix
 */
mat34_t projection_matrix(const pinhole_t &model,
                          const mat3_t &R,
                          const vec3_t &t);

/**
 * Convert pixel to point
 *
 * @param[in] model Camera model
 * @param[in] pixel Pixel measurement
 * @returns Point in image plane
 */
vec2_t pixel2point(const pinhole_t &model, const vec2_t &pixel);

/**
 * Pinhole camera model intrinsics matrix
 *
 * @param[in] intrinsics Intrinsics vector (fx, fy, cx, cy)
 * @returns Intrinsics matrix K
 */
mat3_t pinhole_K(const vec4_t &intrinsics);

/**
 * Pinhole camera model intrinsics matrix
 *
 * @param[in] fx Focal-length in x-axis
 * @param[in] fy Focal-length in y-axis
 * @param[in] cx Principal center in x-axis
 * @param[in] cy Principal center in y-axis
 * @returns Intrinsics matrix K
 */
mat3_t
pinhole_K(const double fx, const double fy, const double cx, const double cy);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param[in] image_width Image width [px]
 * @param[in] fov Field of view [deg]
 * @returns Focal length in pixels
 */
double pinhole_focal_length(const int image_width, const double fov);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param[in] image_size Image width and height [px]
 * @param[in] hfov Horizontal field of view [deg]
 * @param[in] vfov Vertical field of view [deg]
 * @returns Focal length in pixels
 */
vec2_t pinhole_focal_length(const vec2_t &image_size,
                            const double hfov,
                            const double vfov);

/**
 * Pinhole projection matrix
 *
 * @param[in] K camera intrinsics matrix K
 * @param[in] R Rotation matrix
 * @param[in] t translation vector
 * @returns Projection matrix
 */
mat34_t pinhole_projection_matrix(const mat3_t &K,
                                  const mat3_t &R,
                                  const vec3_t &t);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param[in] K camera intrinsics matrix K
 * @param[in] p 3D point
 * @returns Projected point in image plane
 */
vec2_t pinhole_project(const mat3_t &K, const vec3_t &p);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param[in] K camera intrinsics matrix K
 * @param[in] R Rotation matrix
 * @param[in] t translation vector
 * @param[in] X 3D point
 *
 * @returns Projected point in image plane
 */
vec3_t pinhole_project(const mat3_t &K,
                       const mat3_t &R,
                       const vec3_t &t,
                       const vec4_t &X);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param[in] K camera intrinsics matrix K
 * @param[in] R Rotation matrix
 * @param[in] t translation vector
 * @param[in] X 3D point
 *
 * @returns Projected point in image plane
 */
vec2_t pinhole_project(const mat3_t &K,
                       const mat3_t &R,
                       const vec3_t &t,
                       const vec3_t &X);

/**
 * Convert pixel to point
 *
 * @param[in] fx Focal length in x-axis
 * @param[in] fy Focal length in y-axis
 * @param[in] cx Principle center in x-axis
 * @param[in] cy Principle center in y-axis
 * @param[in] pixel Pixel measurement
 *
 * @returns Point in image plane
 */
vec2_t pinhole_pixel2point(const double fx,
                           const double fy,
                           const double cx,
                           const double cy,
                           const vec2_t &pixel);

/**
 * Convert pixel to point
 *
 * @param[in] K camera intrinsics matrix K
 * @param[in] pixel Pixel measurement
 * @returns Point in image plane
 */
vec2_t pinhole_pixel2point(const mat3_t &K, const vec2_t &pixel);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
