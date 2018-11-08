/**
 * @file
 * @defgroup radtan radtan
 * @ingroup vision
 */
#ifndef PROTOTYPE_VISION_CAMERA_RADTAN_HPP
#define PROTOTYPE_VISION_CAMERA_RADTAN_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup radtan
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
 * Type to output stream
 */
std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4);

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
 * @param[in] point Point
 * @param[out] J Jacobian of radtan w.r.t. point
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point, mat2_t &J);

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] points Points
 * @returns Distorted points
 */
matx_t distort(const radtan4_t &radtan, const matx_t &points);

/**
 * Undistort point
 *
 * @param[in] radtan Radial tangential parameters
 * @param[in] p0 Distorted point
 * @param[in] max_iter Max iteration
 * @returns Undistorted point
 */
vec2_t undistort(const radtan4_t &radtan, const vec2_t &p0, const int max_iter=5);

/** @} group radtan */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_RADTAN_HPP
