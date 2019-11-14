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
int pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                            const Eigen::Matrix<T, 4, 1> &D,
                            const Eigen::Matrix<T, 3, 1> &point,
                            Eigen::Matrix<T, 2, 1> &image_point) {
  // Check for singularity
  const T z_norm = sqrt(point(2) * point(2));  // std::abs doesn't work for all T
  if (z_norm < 1.0e-12) {
    return -1;
  }

  // Extract distortion params
  const T k1 = D(0);
  const T k2 = D(1);
  const T p1 = D(2);
  const T p2 = D(3);

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
  const T fx = K(0, 0);
  const T fy = K(1, 1);
  const T cx = K(0, 2);
  const T cy = K(1, 2);
  image_point(0) = fx * x_ddash + cx;
  image_point(1) = fy * y_ddash + cy;

  // // Set result
  // image_point(0) = x_ddash;
  // image_point(1) = y_ddash;

  if (point(2) > T(0.0)) {
    return 0;  // Point is infront of camera
  } else {
    return 1;  // Point is behind camera
  }
}

template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_equi4_project(const Eigen::Matrix<T, 3, 3> &K,
                      const Eigen::Matrix<T, 4, 1> &D,
                      const Eigen::Matrix<T, 3, 1> &point) {
  // Project
  const T x = point(0) / point(2);
  const T y = point(1) / point(2);

  // Radial distortion factor
  const T k1 = D(0);
  const T k2 = D(1);
  const T k3 = D(2);
  const T k4 = D(3);
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
  Eigen::Matrix<T, 2, 1> x_distorted{x_dash, y_dash};
  const Eigen::Matrix<T, 2, 1> pixel = (K * x_distorted.homogeneous()).head(2);

  return pixel;
}

} //  namespace proto
#endif // PROTO_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
