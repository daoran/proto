/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_CAMCHAIN_HPP
#define PROTOTYPE_CALIB_CAMCHAIN_HPP

#include "prototype/core.hpp"
#include "camera/distortion.hpp"
#include "calibration/camera_property.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Camchain
 */
class Camchain {
public:
  std::vector<CameraProperty> cam;
  Mat4 T_C1_C0;

  VecX tau_s = zeros(6, 1);
  VecX tau_d = zeros(6, 1);
  VecX w1 = zeros(3, 1);
  VecX w2 = zeros(3, 1);
  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

  Camchain();
  virtual ~Camchain();

  /**
   * Load
   *
   * @param nb_cameras Number of cameras
   * @param camchain_file Path to camchain file
   *
   * @returns 0 for success, -1 for failure
   */
  int load(const int nb_cameras, const std::string &camchain_file);

  /**
   * Save
   *
   * @param output_path Output path to camchain file
   *
   * @returns 0 for success, -1 for failure
   */
  int save(const std::string &camchain_file);
};

/**
 * Camchain to string
 */
std::ostream &operator<<(std::ostream &os, const Camchain &camchain);

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CAMCHAIN_HPP
