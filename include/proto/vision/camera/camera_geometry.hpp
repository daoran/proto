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

} //  namespace proto
#include "camera_geometry_impl.hpp"
#endif // PROTO_CALIB_CAMERA_CAMERA_GEOMETRY_HPP
