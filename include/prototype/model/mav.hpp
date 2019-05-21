#ifndef PROTOTYPE_MODEL_MAV_HPP
#define PROTOTYPE_MODEL_MAV_HPP

#include <float.h>
#include <iostream>

#include "prototype/core/core.hpp"

namespace proto {

/**
 * MAV model
 */
struct mav_model_t {
  vec3_t attitude{0.0, 0.0, 0.0};         ///< Attitude in global frame
  vec3_t angular_velocity{0.0, 0.0, 0.0}; ///< Angular velocity in global frame
  vec3_t position{0.0, 0.0, 0.0};         ///< Position in global frame
  vec3_t linear_velocity{0.0, 0.0, 0.0};  ///< Linear velocity in global frame

  double Ix = 0.0963; ///< Moment of inertia in x-axis
  double Iy = 0.0963; ///< Moment of inertia in y-axis
  double Iz = 0.1927; ///< Moment of inertia in z-axis

  double kr = 0.1; ///< Rotation drag constant
  double kt = 0.2; ///< Translation drag constant

  double l = 0.9; ///< MAV arm length
  double d = 1.0; ///< drag constant

  double m = 1.0;  ///< Mass
  double g = 9.81; ///< Gravity
};

/**
 * Update
 *
 * @param[in,out] qm Model
 * @param[in] motor_inputs Motor inputs (m1, m2, m3, m4)
 * @param[in] dt Time difference (s)
 * @returns 0 for success, -1 for failure
 */
int mav_model_update(mav_model_t &qm,
                     const vec4_t &motor_inputs,
                     const double dt);

} //  namespace proto
#endif // PROTOTYPE_MODEL_MAV_HPP
