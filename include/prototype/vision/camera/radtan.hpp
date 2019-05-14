#ifndef PROTOTYPE_VISION_CAMERA_RADTAN_HPP
#define PROTOTYPE_VISION_CAMERA_RADTAN_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core/core.hpp"

namespace proto {

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
 * @param[out] J Jacobian of radtan w.r.t. point
 * @returns Distorted point
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point, mat2_t &J_point);

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

} //  namespace proto
#endif // PROTOTYPE_VISION_CAMERA_RADTAN_HPP
