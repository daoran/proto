#ifndef PROTOTYPE_CALIB_CALIB_VICON_MARKER_HPP
#define PROTOTYPE_CALIB_CALIB_VICON_MARKER_HPP

#include <ceres/ceres.h>

#include "prototype/core/core.hpp"
#include "prototype/calib/ceres.hpp"
#include "prototype/calib/calib_data.hpp"

namespace proto {

/**
 * Stereo camera calibration residual
 */
struct vicon_marker_residual_t {
  double z_[2] = {0.0, 0.0};         ///< Measurement from cam0
  double p_F_[3] = {0.0, 0.0, 0.0};  ///< Object point

  vicon_marker_residual_t(const vec2_t &z, const vec3_t &p_F);
  ~vicon_marker_residual_t();

  /**
   * Calculate residual
   */
  template <typename T>
  bool operator()(const T *const intrinsics_,
                  const T *const distortion_,
                  const T *const q_MC_,
                  const T *const t_MC_,
                  const T *const q_WM_,
                  const T *const t_WM_,
                  const T *const q_WF_,
                  const T *const t_WF_,
                  T *residual) const;
};

/**
 * Evaluate the vicon marker cost function
 */
double evaluate_vicon_marker_cost(const std::vector<aprilgrid_t> &aprilgrids,
                                  mat4s_t &T_WM,
                                  pinhole_t &pinhole,
                                  radtan4_t &radtan,
                                  mat4_t &T_MC);

/**
 * Calibrate vicon marker to camera extrinsics
 */
int calib_vicon_marker_solve(const std::vector<aprilgrid_t> &aprilgrids,
                             mat4s_t &T_WM,
                             pinhole_t &pinhole,
                             radtan4_t &radtan,
                             mat4_t &T_MC,
                             mat4_t &T_WF);

} //  namespace proto
#include "calib_vicon_marker_impl.hpp"
#endif // PROTOTYPE_CALIB_CALIB_VICON_MARKER_HPP
