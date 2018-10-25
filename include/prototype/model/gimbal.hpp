/**
 * @file
 * @ingroup gimbal
 */
#ifndef PROTOTYPE_MODEL_GIMBAL_HPP
#define PROTOTYPE_MODEL_GIMBAL_HPP

#include <string>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup model
 * @{
 */

/**
 * Create DH transform from link n to link n-1 (end to front)
 *
 * @param theta
 * @param d
 * @param a
 * @param alpha
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
  * @param roll Roll (radians)
  * @param pitch Pitch (radians)
  */
void gimbal_model_set_attitude(gimbal_model_t &model,
                               const double roll,
                               const double pitch);

/**
  * Get gimbal joint angle
  */
vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model);

/**
  * Returns transform from static camera to base mechanism
  */
mat4_t gimbal_model_T_bs(const gimbal_model_t &model);

/**
  * Returns transform from base mechanism to end-effector
  */
mat4_t gimbal_model_T_eb(const gimbal_model_t &model);

/**
  * Returns transform from end-effector to dynamic camera
  */
mat4_t gimbal_model_T_de(const gimbal_model_t &model);

/**
  * Returns transform from static to dynamic camera
  */
mat4_t gimbal_model_T_ds(const gimbal_model_t &model);

/**
  * Returns transform from static to dynamic camera
  *
  * @param theta Gimbal roll and pitch [radians]
  * @returns Transform from static to dynamic camera
  */
mat4_t gimbal_model_T_ds(gimbal_model_t &model, const vec2_t &theta);

std::ostream &operator<<(std::ostream &os, const gimbal_model_t &gimbal);

/** @} group gimbal */
} //  namespace prototype
#endif // PROTOTYPE_MODEL_GIMBAL_HPP
