/**
 * @file
 * @ingroup sim
 */
#ifndef PROTOTYPE_SIM_CAMERA_HPP
#define PROTOTYPE_SIM_CAMERA_HPP

#include "prototype/core.hpp"
#include "prototype/model/gimbal.hpp"
#include "prototype/vision/camera/pinhole_model.hpp"

namespace prototype {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Virtual camera
 */
class VirtualCamera {
public:
  PinholeModel camera_model;

  VirtualCamera();
  VirtualCamera(const int image_width,
                const int image_height,
                const double fx,
                const double fy,
                const double cx,
                const double cy);
  virtual ~VirtualCamera();

  /**
   * Return features are observed by camera
   *
   * **IMPORTANT**: This function assumes the inputs uses a coordinate sytem
   * where x-forard, y-left, z-up. This is in contrast to common camera
   * coordinate system where z is forward.
   *
   * @param features Feature matrix witn N features per row (i.e. Nx3 matrix)
   * @param rpy_G Orientation as roll, pitch  yaw. Note: x is forward
   * @param t_G Translation. Note: x is forward
   * @param feature_ids Features observed
   *
   * @returns Observed features in the image plane
   */
  matx_t observedFeatures(const matx_t &features,
                        const vec3_t &rpy_G,
                        const vec3_t &t_G,
                        std::vector<int> &feature_ids);
};

/**
 * Virtual stereo camera
 */
class VirtualStereoCamera {
public:
  std::string type;
  PinholeModel camera_model;
  GimbalModel gimbal_model;
  mat4_t T_cam1_cam0;

  VirtualStereoCamera();

  VirtualStereoCamera(const int image_width,
                      const int image_height,
                      const double fx,
                      const double fy,
                      const double cx,
                      const double cy,
                      const mat4_t &T_cam1_cam0);

  VirtualStereoCamera(const int image_width,
                      const int image_height,
                      const double fx,
                      const double fy,
                      const double cx,
                      const double cy,
                      const GimbalModel &gimal_model);

  virtual ~VirtualStereoCamera();

  /**
   * Return features are observed by camera
   *
   * **IMPORTANT**: This function assumes the inputs uses a coordinate sytem
   * where x-forard, y-left, z-up. This is in contrast to common camera
   * coordinate system where z is forward.
   *
   * @param features Feature matrix witn N features per row (i.e. Nx3 matrix)
   * @param rpy_G Orientation as roll, pitch  yaw. Note: x is forward
   * @param t_G Translation. Note: x is forward
   * @param feature_ids Features observed
   *
   * @returns Observed features in the image plane
   */
  matx_t observedFeatures(const matx_t &features,
                        const vec3_t &rpy_G,
                        const vec3_t &t_G,
                        std::vector<int> &feature_ids);

  /**
   * Set gimbal attitude
   *
   * @param roll Roll (radians)
   * @param pitch Pitch (radians)
   */
  void setGimbalAttitude(const double roll, const double pitch);
};

/** @} group sim */
} //  namespace prototype
#endif // PROTOTYPE_SIM_CAMERA_HPP
