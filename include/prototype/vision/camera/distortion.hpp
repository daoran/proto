/**
 * @file
 * @ingroup camera
 */
#ifndef PROTOTYPE_VISION_CAMERA_DISTORTION_HPP
#define PROTOTYPE_VISION_CAMERA_DISTORTION_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * Source:
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
matx_t radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const matx_t &points);

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * Source:
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
vec2_t radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const vec3_t &point);

/**
 * Distort 3D points with the equi-distant distortion model
 */
matx_t equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const matx_t &points);

/**
 * Distort a single 3D point with the equi-distant distortion model
 */
vec2_t equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const vec3_t &point);

/**
 * Un-distort a 2D point with the equi-distant distortion model
 */
void equi_undistort(const double k1,
                    const double k2,
                    const double k3,
                    const double k4,
                    vec2_t &p);

/**
 * Pinhole Equidistant undistort image
 */
cv::Mat pinhole_equi_undistort_image(const mat3_t &K,
                                     const vecx_t &D,
                                     const cv::Mat &image,
                                     const double balance,
                                     cv::Mat &K_new);

/**
 * Pinhole Equidistant undistort image
 */
cv::Mat pinhole_equi_undistort_image(const mat3_t &K,
                                     const vecx_t &D,
                                     const cv::Mat &image,
                                     cv::Mat &K_new);

/**
 * Project pinhole radial-tangential
 */
vec2_t project_pinhole_radtan(const mat3_t &K, const vecx_t &D, const vec3_t &X);

/**
 * Project pinhole equidistant
 */
vec2_t project_pinhole_equi(const mat3_t &K, const vec4_t &D, const vec3_t &X);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_DISTORTION_HPP
