#ifndef PROTO_MODEL_HPP
#define PROTO_MODEL_HPP

#include <float.h>

#include <string>
#include <iostream>

#include "proto/core/core.hpp"

namespace proto {

/*****************************************************************************
 *                                GIMBAL
 ****************************************************************************/

/**
 * Create DH transform from link n to link n-1 (end to front)
 *
 * @param[in] theta
 * @param[in] d
 * @param[in] a
 * @param[in] alpha
 *
 * @returns DH transform
 */
mat4_t dh_transform(const double theta,
                    const double d,
                    const double a,
                    const double alpha);

/**
 * 2-DOF Gimbal Model
 */
struct gimbal_model_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Parameter vector of transform from
  // static camera to base-mechanism
  vecx_t tau_s = zeros(6, 1);

  // Parameter vector of transform from
  // end-effector to dynamic camera
  vecx_t tau_d = zeros(6, 1);

  // First gibmal-joint
  double Lambda1 = 0.0;
  vec3_t w1 = zeros(3, 1);

  // Second gibmal-joint
  double Lambda2 = 0.0;
  vec3_t w2 = zeros(3, 1);

  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

  gimbal_model_t();
  gimbal_model_t(const vec6_t &tau_s,
                 const vec6_t &tau_d,
                 const double Lambda1,
                 const vec3_t w1,
                 const double Lambda2,
                 const vec3_t w2,
                 const double theta1_offset = 0.0,
                 const double theta2_offset = 0.0);
  virtual ~gimbal_model_t();
};

/**
 * Set gimbal attitude
 *
 * @param[in,out] model Model
 * @param[in] roll Roll [rads]
 * @param[in] pitch Pitch [rads]
 */
void gimbal_model_set_attitude(gimbal_model_t &model,
                               const double roll,
                               const double pitch);

/**
 * Get gimbal joint angle
 *
 * @param[in] model Model
 * @returns Gimbal joint angles
 */
vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model);

/**
 * Returns transform from static camera to base mechanism
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_BS(const gimbal_model_t &model);

/**
 * Returns transform from base mechanism to end-effector
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_EB(const gimbal_model_t &model);

/**
 * Returns transform from end-effector to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DE(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DS(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in,out] model Model
 * @param[in] theta Gimbal roll and pitch [radians]
 * @returns Transform from static to dynamic camera
 */
mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta);

/**
 * gimbal_model_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_model_t &gimbal);

/*****************************************************************************
 *                                TWO WHEEL
 ****************************************************************************/

/**
 * Calculate target angular velocity and time taken to traverse a desired
 * circle * trajectory of radius r and velocity v
 *
 * @param[in] r Desired circle radius
 * @param[in] v Desired trajectory velocity
 * @param[in] w Target angular velocity
 * @param[in] time Target time taken to complete circle trajectory
 **/
void circle_trajectory(const double r, const double v, double *w, double *time);

/**
 * Two wheel robot
 */
struct two_wheel_t {
  vec3_t p_G = vec3_t::Zero();
  vec3_t v_G = vec3_t::Zero();
  vec3_t a_G = vec3_t::Zero();
  vec3_t rpy_G = vec3_t::Zero();
  vec3_t w_G = vec3_t::Zero();

  double vx_desired = 0.0;
  double yaw_desired = 0.0;

  pid_t vx_controller{0.1, 0.0, 0.1};
  pid_t yaw_controller{0.1, 0.0, 0.1};

  vec3_t a_B = vec3_t::Zero();
  vec3_t v_B = vec3_t::Zero();
  vec3_t w_B = vec3_t::Zero();

  two_wheel_t() {}

  two_wheel_t(const vec3_t &p_G_, const vec3_t &v_G_, const vec3_t &rpy_G_)
      : p_G{p_G_}, v_G{v_G_}, rpy_G{rpy_G_} {}

  ~two_wheel_t() {}
};

/**
 * Update
 *
 * @param[in,out] tm Model
 * @param[in] dt Time difference (s)
 */
void two_wheel_update(two_wheel_t &tm, const double dt);

/*****************************************************************************
 *                                 MAV
 ****************************************************************************/

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
#endif // PROTO_MODEL_HPP
