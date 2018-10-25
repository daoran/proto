/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_CALIB_DATA_HPP
#define PROTOTYPE_CALIB_CALIB_DATA_HPP

#include <string>
#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

struct CalibData {
  int nb_measurements = 0;
  std::vector<matx_t> P_s;
  std::vector<matx_t> P_d;
  std::vector<matx_t> Q_s;
  std::vector<matx_t> Q_d;
  matx_t joint_data;

  /**
   * Load data
   *
   * @param data_dir Data directory
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/** @} group calibration */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_DATA_HPP
