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
            const double p2_)
      : k1{k1_}, k2{k2_}, p1{p1_}, p2{p2_} {}
  ~radtan4_t() {}
};

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * Source:
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
matx_t distort(const radtan4_t &radtan, const matx_t &points);

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * Source:
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
vec2_t distort(const radtan4_t &radtan, const vec2_t &point);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_RADTAN_HPP
