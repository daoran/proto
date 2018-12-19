#ifndef PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
#define PROTOTYPE_VISION_CAMERA_PINHOLE_HPP

#include "prototype/core/core.hpp"

namespace proto {

/**
 * Pinhole camera model
 */
struct pinhole_t {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double *data[4] = {&fx, &fy, &cx, &cy};

  pinhole_t();
  pinhole_t(const vec4_t &intrinsics);
  pinhole_t(const double fx_,
            const double fy_,
            const double cx_,
            const double cy_);
  ~pinhole_t();
};

/**
 * Type to output stream
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
pinhole_K(const double fx, const double fy, const double cx, const double cy);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] pinhole Pinhole camera
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const pinhole_t &pinhole);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] intrinsics Intrinsics (fx, fy, cx, cy)
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const double *intrinsics);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] intrinsics Intrinsics (fx, fy, cx, cy)
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const vec4_t &intrinsics);

/**
 * Form pinhole camera matrix K
 *
 * @param[in] image_size Image width and height [px]
 * @param[in] lens_hfov Lens horizontal field of view [deg]
 * @param[in] lens_vfov Lens vertical field of view [deg]
 *
 * @returns Camera matrix K
 */
mat3_t pinhole_K(const vec2_t &image_size,
                 const double lens_hfov,
                 const double lens_vfov);

/**
 * Form pinhole projection matrix P
 *
 * @param[in] K Camera matrix K
 * @param[in] R Camera rotation matrix
 * @param[in] t Camera translation vector
 * @returns Camera projection matrix P
 */
mat34_t pinhole_P(const mat3_t &K, const mat3_t &R, const vec3_t &t);

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
 * Project point to pixel coordinates
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point in 3D
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &pinhole, const vec3_t &p);

/**
 * Project point to pixel coordinates
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] p Point
 * @returns Point in pixel coordinates
 */
vec2_t project(const pinhole_t &pinhole, const vec2_t &p);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] R Rotation matrix
 * @param[in] t translation vector
 * @param[in] hp 3D point in homogeneous coordinates
 *
 * @returns Projected point in image plane
 */
vec2_t project(const pinhole_t &pinhole,
               const mat3_t &R,
               const vec3_t &t,
               const vec4_t &hp);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] R Rotation matrix
 * @param[in] t translation vector
 * @param[in] p 3D point
 *
 * @returns Projected point in image plane
 */
vec2_t project(const pinhole_t &pinhole,
               const mat3_t &R,
               const vec3_t &t,
               const vec3_t &p);

/**
 * Convert pixel to ideal
 *
 * @param[in] pinhole Pinhole camera model
 * @param[in] pixel Pixel measurement
 * @returns Point in image plane
 */
vec2_t pixel2ideal(const pinhole_t &pinhole, const vec2_t &pixel);

} //  namespace proto
#endif // PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
