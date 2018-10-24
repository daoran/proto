/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
#define PROTOTYPE_CALIB_GIMBAL_CALIB_HPP

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/calib/calib_data.hpp"
#include "prototype/calib/calib_params.hpp"
#include "prototype/calib/residual.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

class GimbalCalib {
public:
  std::string data_dir;
  CalibData data;
  CalibParams params;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  GimbalCalib();
  virtual ~GimbalCalib();

  /**
   * Load data
   *
   * @param data_dir Data directory
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);

  /**
   * Calculate reprojection errors
   * @return 0 for success, -1 for failure
   */
  int calculateReprojectionErrors();

  /**
   * Calibrate calibration
   */
  int calibrate();
};

/** @} group gimbal */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
