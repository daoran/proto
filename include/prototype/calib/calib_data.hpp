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
  std::vector<MatX> P_s;
  std::vector<MatX> P_d;
  std::vector<MatX> Q_s;
  std::vector<MatX> Q_d;
  MatX joint_data;

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
