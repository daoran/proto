/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_SIMULATION_HPP
#define PROTOTYPE_CALIB_SIMULATION_HPP

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/calib/calib_data.hpp"
#include "prototype/calib/calib_params.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

class GimbalSimulation {
public:
  CalibData data;
  CalibParams params;

  GimbalSimulation();
  virtual ~GimbalSimulation();
};

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_SIMULATION_HPP
