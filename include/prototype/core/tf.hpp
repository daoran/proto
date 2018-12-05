#ifndef PROTOTYPE_TF_HPP
#define PROTOTYPE_TF_HPP

#include "prototype/core/math.hpp"
#include "prototype/core/linalg.hpp"

namespace prototype {

/**
 * Extract rotation from transform
 */
inline mat3_t tf_rot(const mat4_t &tf) {
  return tf.block<3, 3>(0, 0);
}

/**
 * Extract translation from transform
 */
inline vec3_t tf_trans(const mat4_t &tf) {
  return tf.block<3, 1>(0, 3);
}

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
mat4_t tf(const mat3_t &C, const vec3_t &r);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * hamiltonian quaternion `q` and translation vector `r`.
 */
mat4_t tf(const quat_t &q, const vec3_t &r);

} //  namespace prototype
#endif // PROTOTYPE_TF_HPP
