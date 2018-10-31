/**
 * @file
 * @ingroup mav
 */
#ifndef PROTOTYPE_MODEL_MAV_HPP
#define PROTOTYPE_MODEL_MAV_HPP

#include <float.h>
#include <iostream>

#include "prototype/control/att_ctrl.hpp"
#include "prototype/control/pos_ctrl.hpp"
#include "prototype/control/wp_ctrl.hpp"
#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup model
 * @{
 */

/**
 * MAV model
 */
struct mav_model_t {
  vec3_t rpy_G{0.0, 0.0, 0.0}; ///< Attitude in global frame
  vec3_t w_G{0.0, 0.0, 0.0};   ///< Angular velocity in global frame
  vec3_t p_G{0.0, 0.0, 0.0};   ///< Position in global frame
  vec3_t a_G{0.0, 0.0, 0.0};   ///< Acceleration in global frame
  vec3_t v_G{0.0, 0.0, 0.0};   ///< Linear velocity in global frame

  vec3_t w_B{0.0, 0.0, 0.0}; ///< Angular velocity in body frame
  vec3_t a_B{0.0, 0.0, 0.0}; ///< Acceleration in body frame

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
 * @param motor_inputs Motor inputs (m1, m2, m3, m4)
 * @param dt Time difference (s)
 * @returns 0 for success, -1 for failure
 */
int mav_model_update(mav_model_t &qm,
                     const vec4_t &motor_inputs,
                     const double dt);

// /**
//   * Update
//   *
//   * @param dt Time difference (s)
//   * @returns 0 for success, -1 for failure
//   */
// int mav_model_update(const double dt);

/**
 * Set mav attitude controller setpoints
 *
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @param z Thrust
 */
void mav_model_set_attitude(mav_model_t &qm,
                            const double roll,
                            const double pitch,
                            const double yaw,
                            const double z);

/**
 * Set mav position controller setpoints
 *
 * @param p_G Position in global frame
 */
void mav_model_set_position(mav_model_t &qm, const vec3_t &p_G);

/**
 * Print state
 */
void mav_model_print(mav_model_t &qm);

/** @} group model */
} //  namespace prototype
#endif // PROTOTYPE_MODEL_MAV_HPP
