#ifndef PROTO_VISION_CAMERA_PINHOLE_HPP
#define PROTO_VISION_CAMERA_PINHOLE_HPP

#include "proto/core/core.hpp"
#include "proto/vision/camera/radtan.hpp"
#include "proto/vision/camera/equi.hpp"

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
  pinhole_t(const mat3_t &K);
  pinhole_t(const double fx_,
            const double fy_,
            const double cx_,
            const double cy_);
  pinhole_t(pinhole_t &pinhole);
  pinhole_t(const pinhole_t &pinhole);
  ~pinhole_t();

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
pinhole_K(const double fx, const double fy, const double cx, const double cy);

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
                 const double lens_hfov,
                 const double lens_vfov);

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
vec2_t project(const pinhole_t &model,
               const vec3_t &p,
               mat_t<2, 3> &J_h);

} //  namespace proto
#endif // PROTO_VISION_CAMERA_PINHOLE_HPP
