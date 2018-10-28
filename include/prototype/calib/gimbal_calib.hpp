/**
 * @file
 * @ingroup calibration
 */
#ifndef PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
#define PROTOTYPE_CALIB_GIMBAL_CALIB_HPP

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/model/gimbal.hpp"
// #include "prototype/calib/calib_params.hpp"
// #include "prototype/calib/residual.hpp"

namespace prototype {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Gimbal calibration data
 */
struct gimbal_calib_data_t {
	bool ok = false;

  int nb_measurements = 0;
  std::vector<matx_t> P_s;
  std::vector<matx_t> P_d;
  std::vector<matx_t> Q_s;
  std::vector<matx_t> Q_d;
  matx_t joint_data;

	gimbal_calib_data_t();
	virtual ~gimbal_calib_data_t();
};

/**
 * Calibration parameters
 */
struct gimbal_calib_params_t {
  bool ok = false;
  // Camchain camchain;

  double *tau_s = nullptr;
  double *tau_d = nullptr;
  double *w1 = nullptr;
  double *w2 = nullptr;
  double *theta1_offset = nullptr;
  double *theta2_offset = nullptr;
  double *Lambda1 = nullptr;
  double *Lambda2 = nullptr;
  int nb_measurements = 0;

  gimbal_calib_params_t();
  virtual ~gimbal_calib_params_t();
};

/**
 * Gimbal calibrator
 */
struct gimbal_calibrator_t {
  std::string data_dir;
  gimbal_calib_data_t data;
  gimbal_calib_params_t params;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  gimbal_calibrator_t();
  virtual ~gimbal_calibrator_t();
};

/**
 * gimbal_calib_data_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_calib_data_t &m);

/**
 * gimbal_calib_params_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_calib_params_t &m);

/**
	* Load gimbal calibration data
	*
	* @param[in,out] data Gimbal calibration data
	* @param[in] data_dir Data directory
	* @return 0 for success, -1 for failure
	*/
int gimbal_calib_data_load(gimbal_calib_data_t &data,
													 const std::string &data_dir);

/**
	* Load initial optimization params
	*
	* @param[in,out] data Gimbal calibration data
	* @param[in] camchain_file Path to camchain file
	* @param[in] joint_file Path to joint angles file
	* @returns 0 for success, -1 for failure
	*/
int gimbal_calib_params_load(gimbal_calib_data_t &data,
														 const std::string &camchain_file,
														 const std::string &joint_file);

/**
 * Load gimbal calibrator and data
 *
	* @param[in,out] calib Gimbal calibrator
  * @param[in] data_dir Path to where camchain file and calib data are
	* @returns 0 for success, -1 for failure
 */
int gimbal_calibrator_load(gimbal_calibrator_t &calib,
                           const std::string &data_dir);

/**
  * Calculate reprojection errors
  *
	* @param[in,out] calib Gimbal calibrator
  * @return 0 for success, -1 for failure
  */
int gimbal_calibrator_calc_reprojection_errors(gimbal_calibrator_t &calib);

/**
  * Calibrate calibration
  *
	* @param[in,out] calib Gimbal calibrator
  * @return 0 for success, -1 for failure
  */
int gimbal_calibrator_solve(gimbal_calibrator_t &calib);


/** @} group gimbal */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_GIMBAL_CALIB_HPP
