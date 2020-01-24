#ifndef PROTO_CALIB_CAMERA_CAMERA_HPP
#define PROTO_CALIB_CAMERA_CAMERA_HPP

#include <iostream>

#include "proto/core/core.hpp"

namespace proto {

/****************************************************************************
 *                            RADIAL-TANGENTIAL
 ***************************************************************************/

/**
 * Radial-tangential distortion
 */
struct radtan4_t {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double *data[4] = {&k1, &k2, &p1, &p2};

  radtan4_t();
  radtan4_t(const vec4_t &distortion_);
  radtan4_t(const double k1_,
            const double k2_,
            const double p1_,
            const double p2_);
  radtan4_t(radtan4_t &radtan);
  radtan4_t(const radtan4_t &radtan);
  ~radtan4_t();

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
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;

  equi4_t(const double k1_,
          const double k2_,
          const double k3_,
          const double k4_);
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
vec2_t project(const pinhole_t &model, const vec3_t &p, mat_t<2, 3> &J_h);

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
  const T z_norm = sqrt(point(2) * point(2)); // std::abs doesn't work for all T
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
    return 0; // Point is infront of camera
  } else {
    return 1; // Point is behind camera
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
#endif // PROTO_CALIB_CAMERA_CAMERA_HPP
