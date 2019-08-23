#ifndef PROTO_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
#define PROTO_CALIB_CAMERA_CAMERA_GEOMETRY_HPP

#include <iostream>

#include "proto/core/core.hpp"
#include "proto/vision/camera/pinhole.hpp"
#include "proto/vision/camera/radtan.hpp"
#include "proto/vision/camera/equi.hpp"

namespace proto {

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
 * Project point using pinhole radial-tangential
 */
template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                        const Eigen::Matrix<T, 4, 1> &D,
                        const Eigen::Matrix<T, 3, 1> &point);

/******************************************************************************
 * IMPLEMENTATION
 *****************************************************************************/

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

template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                        const Eigen::Matrix<T, 4, 1> &D,
                        const Eigen::Matrix<T, 3, 1> &point) {
  const T k1 = D(0);
  const T k2 = D(1);
  const T p1 = D(2);
  const T p2 = D(3);

  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

  // Radial distortion factor
  const T x2 = x * x;
  const T y2 = y * y;
  const T r2 = x2 + y2;
  const T r4 = r2 * r2;
  const T radial_factor = T(1) + (k1 * r2) + (k2 * r4);
  const T x_dash = x * radial_factor;
  const T y_dash = y * radial_factor;

  // Tangential distortion factor
  const T xy = x * y;
  const T x_ddash = x_dash + (T(2) * p1 * xy + p2 * (r2 + T(2) * x2));
  const T y_ddash = y_dash + (p1 * (r2 + T(2) * y2) + T(2) * p2 * xy);

  // Scale distorted point
  Eigen::Matrix<T, 2, 1> x_distorted{x_ddash, y_ddash};
  const Eigen::Matrix<T, 2, 1> pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

} //  namespace proto
#endif // PROTO_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
