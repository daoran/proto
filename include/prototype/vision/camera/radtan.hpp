/**
 * @file
 * @ingroup camera
 */
#ifndef PROTOTYPE_VISION_CAMERA_RADTAN_HPP
#define PROTOTYPE_VISION_CAMERA_RADTAN_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Radial-tangential distortion
 */
struct radtan4_t {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;

  radtan4_t(const double k1_,
            const double k2_,
            const double p1_,
            const double p2_);
  ~radtan4_t();
};

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] point Point
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point);

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] points Points
 * @returns Distorted points
 */
matx_t distort(const radtan4_t &radtan, const matx_t &points);

/**
 * Distort Jacobian
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] point Point
 * @returns Jacobian of distorted point p' w.r.t. point p
 */
mat2_t distort_jacobian(const radtan4_t &radtan, const vec2_t &point);

/**
 * Undistort point
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] p0 Distorted point
 * @returns Undistorted point
 */
vec2_t undistort(const radtan4_t &radtan, const vec2_t &p0, const int max_iter=5);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_RADTAN_HPP
