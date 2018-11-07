/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_CAMERA_HPP
#define PROTOTYPE_CALIB_CALIB_CAMERA_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/calib/calib.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

/**
 * Pose parameter block
 */
struct pose_param_t {
  quat_t q;
  vec3_t t;

  pose_param_t(const mat4_t &T)
    : q{T.block<3, 3>(0, 0)}, t{T.block<3, 1>(0, 3)} {}

  ~pose_param_t() {}
};

/**
 * Pinhole Radial-tangential calibration residual
 */
struct pinhole_radtan4_residual_t {
  double p_F_[3] = {0.0, 0.0, 0.0};  ///< Object point
  double z_[2] = {0.0, 0.0};         ///< Measurement

  pinhole_radtan4_residual_t(const vec2_t &z, const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const intrinsics,
                  const T *const distortion,
                  const T *const q_CF,
                  const T *const t_CF,
                  T *residual) const;
};

/**
 * Setup camera calibration problem
 *
 * @param[in] aprilgrids AprilGrids
 * @param[in,out] pinhole Pinhole parameters
 * @param[in,out] radtan Radtan parameters
 * @param[out] poses Optimized poses
 *
 * @returns 0 or -1 for success or failure
 */
int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan,
                       std::vector<mat4_t> &poses);

/**
 * Perform stats analysis on calibration after performing intrinsics
 * calibration
 *
 * @param[in] aprilgrids AprilGrids
 * @param[in] intrinsics Intrinsics vector (fx, fy, cx, cy)
 * @param[in] distortion Distortion vector
 * @param[in] poses Optimized poses
 * @param[in] output_path Path to save output
 *
 * @returns 0 or -1 for success or failure
 */
template <typename RESIDUAL>
int calib_camera_stats(const std::vector<aprilgrid_t> &aprilgrids,
                       const double *intrinsics,
                       const double *distortion,
                       const std::vector<mat4_t> &poses,
                       const std::string &output_path);

/** @} group calib */
} //  namespace prototype
#include "calib_camera_impl.hpp"
#endif // PROTOTYPE_CALIB_CALIB_CAMERA_HPP
