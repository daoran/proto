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
Mat4 dh_transform(const double theta,
                  const double d,
                  const double a,
                  const double alpha);

/**
 * 2-DOF Gimbal Model
 */
class GimbalModel {
public:
  // Parameter vector of transform from
  // static camera to base-mechanism
  VecX tau_s = zeros(6, 1);

  // Parameter vector of transform from
  // end-effector to dynamic camera
  VecX tau_d = zeros(6, 1);

  // First gibmal-joint
  double Lambda1 = 0.0;
  Vec3 w1 = zeros(3, 1);

  // Second gibmal-joint
  double Lambda2 = 0.0;
  Vec3 w2 = zeros(3, 1);

  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

  GimbalModel();
  GimbalModel(const Vec6 &tau_s,
              const Vec6 &tau_d,
              const double Lambda1,
              const Vec3 w1,
              const double Lambda2,
              const Vec3 w2,
              const double theta1_offset = 0.0,
              const double theta2_offset = 0.0);
  virtual ~GimbalModel();

  /**
   * Set gimbal attitude
   *
   * @param roll Roll (radians)
   * @param pitch Pitch (radians)
   */
  void setAttitude(const double roll, const double pitch);

  /**
   * Get gimbal joint angle
   */
  Vec2 getJointAngles();

  /**
   * Returns transform from static camera to base mechanism
   */
  Mat4 T_bs();

  /**
   * Returns transform from base mechanism to end-effector
   */
  Mat4 T_eb();

  /**
   * Returns transform from end-effector to dynamic camera
   */
  Mat4 T_de();

  /**
   * Returns transform from static to dynamic camera
   */
  Mat4 T_ds();

  /**
   * Returns transform from static to dynamic camera
   *
   * @param theta Gimbal roll and pitch [radians]
   * @returns Transform from static to dynamic camera
   */
  Mat4 T_ds(const Vec2 &theta);
};

std::ostream &operator<<(std::ostream &os, const GimbalModel &gimbal);

/** @} group gimbal */
} //  namespace prototype
#endif // PROTOTYPE_MODEL_GIMBAL_HPP
