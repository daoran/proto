/**
 * @file
 * @ingroup camera
 */
#ifndef PROTOTYPE_VISION_CAMERA_EQUI_HPP
#define PROTOTYPE_VISION_CAMERA_EQUI_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Equi-distant distortion
 */
struct equi_t {
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;

  equi_t(const double k1_, const double k2_, const double k3_, const double k4_)
      : k1{k1_}, k2{k2_}, k3{k3_}, k4{k4_} {}
  ~equi_t() {}
};

/**
 * Distort 3D points with the equi-distant distortion model
 */
matx_t distort(const equi_t &equi, const matx_t &point);

/**
 * Distort a single 3D point with the equi-distant distortion model
 */
vec2_t distort(const equi_t &equi, const vec3_t &point);

/**
 * Un-distort a 2D point with the equi-distant distortion model
 */
void undistort(const equi_t &equi, vec2_t &p);

/**
 * Equidistant undistort image
 */
cv::Mat undistort_image(const mat3_t &K,
                        const vecx_t &D,
                        const cv::Mat &image,
                        cv::Mat &K_new,
                        const double balance = 0);

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_EQUI_HPP
