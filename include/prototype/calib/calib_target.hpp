/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_TARGET_HPP
#define PROTOTYPE_CALIB_CALIB_TARGET_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

/**
 * Calibration target
 */
struct calib_target_t {
  std::string type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  calib_target_t();
  ~calib_target_t();
};

/**
 * calib_target_t to string
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);

/**
 * Load calibration target
 *
 * @param[in,out] ct Calibration target
 * @param[in] target_file Target file
 * @returns 0 and -1 for success or failure
 */
int calib_target_load(calib_target_t &ct, const std::string &target_file);

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_TARGET_HPP
