/**
 * @file
 * @ingroup camera
 */
#ifndef PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
#define PROTOTYPE_VISION_CAMERA_PINHOLE_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Pinhole camera model intrinsics matrix
 *
 * @param intrinsics Intrinsics vector (fx, fy, cx, cy)
 * @returns Intrinsics matrix K
 */
mat3_t pinhole_K(const vec4_t &intrinsics);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param image_width Image width [px]
 * @param fov Field of view [deg]
 * @returns Focal length in pixels
 */
double pinhole_focal_length(const int image_width, const double fov);

/**
 * Pinhole camera model theoretical focal length
 *
 * @param image_size Image width and height [px]
 * @param hfov Horizontal field of view [deg]
 * @param vfov Vertical field of view [deg]
 * @returns Focal length in pixels
 */
vec2_t pinhole_focal_length(const vec2_t &image_size,
                            const double hfov,
                            const double vfov);

/**
 * Pinhole projection matrix
 *
 * @param K camera intrinsics matrix K
 * @param R Rotation matrix
 * @param t translation vector
 *
 * @returns Projection matrix
 */
mat34_t pinhole_projection_matrix(const mat3_t &K, const mat3_t &R, const vec3_t &t);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param K camera intrinsics matrix K
 * @param p 3D point
 * @returns Projected point in image plane
 */
vec2_t pinhole_project(const mat3_t &K, const vec3_t &p);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param K camera intrinsics matrix K
 * @param R Rotation matrix
 * @param t translation vector
 * @param X 3D point
 * @returns Projected point in image plane
 */
vec3_t pinhole_project(const mat3_t &K,
                     const mat3_t &R,
                     const vec3_t &t,
                     const vec4_t &X);

/**
 * Project 3D point to image plane using pinhole model
 *
 * @param K camera intrinsics matrix K
 * @param R Rotation matrix
 * @param t translation vector
 * @param X 3D point
 * @returns Projected point in image plane
 */
vec2_t pinhole_project(const mat3_t &K,
                     const mat3_t &R,
                     const vec3_t &t,
                     const vec3_t &X);

/**
 * Convert pixel to ideal coordinates
 *
 * @param fx Focal length in x-axis
 * @param fy Focal length in y-axis
 * @param cx Principle center in x-axis
 * @param cy Principle center in y-axis
 * @param pixel Pixel measurement
 *
 * @returns Pixel in ideal coordinates
 */
vec2_t pinhole_pixel2ideal(const double fx,
                         const double fy,
                         const double cx,
                         const double cy,
                         const vec2_t &pixel);

/**
 * Convert pixel to ideal coordinates
 *
 * @param K camera intrinsics matrix K
 * @param pixel Pixel measurement
 * @returns Pixel in ideal coordinates
 */
vec2_t pinhole_pixel2ideal(const mat3_t &K, const vec2_t &pixel);

/**
 * Pinhole camera model
 */
class PinholeModel {
public:
  int image_width = 0;
  int image_height = 0;

  mat3_t K = zeros(3, 3); ///< Camera intrinsics
  double cx = 0.0;      ///< Principle center in x-axis
  double cy = 0.0;      ///< Principle center in y-axis
  double fx = 0.0;      ///< Focal length in x-axis
  double fy = 0.0;      ///< Focal length in y-axis

  PinholeModel() {}
  virtual ~PinholeModel() {}

  PinholeModel(const int image_width, const int image_height, const matx_t &K)
      : image_width{image_width}, image_height{image_height}, K{K}, cx{K(0, 2)},
        cy{K(1, 2)}, fx{K(0, 0)}, fy{K(1, 1)} {}

  PinholeModel(const int image_width,
               const int image_height,
               const double fx,
               const double fy,
               const double cx,
               const double cy)
      : image_width{image_width}, image_height{image_height}, cx{cx}, cy{cy},
        fx{fx}, fy{fy} {
    this->K = mat3_t::Zero();
    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;
    K(2, 2) = 1.0;
  }

  /**
   * Configure
   *
   * @param config_file Path to configuration file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Return projection matrix
   *
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns Projection matrix
   */
  mat34_t P(const mat3_t &R, const vec3_t &t);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns 3D point in image plane (homogenous)
   */
  vec2_t project(const vec3_t &X, const mat3_t &R, const vec3_t &t);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point in homogeneous coordinates
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns 3D point in image plane (homogenous)
   */
  vec3_t project(const vec4_t &X, const mat3_t &R, const vec3_t &t);

  /**
   * Convert pixel measurement to ideal coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to ideal coordinates
   */
  vec2_t pixel2ideal(const vec2_t &pixel);

  /**
   * Convert pixel measurement to ideal coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to ideal coordinates
   */
  vec2_t pixel2ideal(const cv::Point2f &pixel);

  /**
   * Convert pixel measurement to ideal coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to ideal coordinates
   */
  vec2_t pixel2ideal(const cv::KeyPoint &pixel);
};

/** @} group camera */
} //  namespace prototype
#endif // PROTOTYPE_VISION_CAMERA_PINHOLE_HPP
