/**
 * @file
 * @ingroup quaternion
 */
#ifndef PROTOTYPE_CORE_QUATERNION_JPL_HPP
#define PROTOTYPE_CORE_QUATERNION_JPL_HPP

#include "prototype/core.hpp"
#include "prototype/core/math.hpp"

namespace prototype {
/**
 * @addtogroup quaternion
 * @{
 */

/**
 * Skew
 *
 * @param vector Quaternion
 * @param matrix_const Skew matrix
 */
template <typename Derived, typename OtherDerived>
void skew(const Eigen::MatrixBase<Derived> &vector,
          Eigen::MatrixBase<OtherDerived> const &matrix_const);

/**
 * Signed quaternion product
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param product_const Quaternion product
 */
template <typename Derived1, typename Derived2, typename Derived3>
void signed_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                    const Eigen::MatrixBase<Derived2> &q2,
                    const Eigen::MatrixBase<Derived3> &product_const);

/**
 * Positive quaternion product
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param product_const Quaternion product
 */
template <typename Derived1, typename Derived2, typename Derived3>
void positive_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                      const Eigen::MatrixBase<Derived2> &q2,
                      const Eigen::MatrixBase<Derived3> &product_const);

/**
 * Quaternion norm
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Norm of quaternion
 */
double quatnorm(const vec4_t &q);

/**
 * Quaternion normalize
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Normalized quaternion
 */
vec4_t quatnormalize(const vec4_t &q);

/**
 * Quaternion conjugate
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Conjugate of quaternion
 */
vec4_t quatconj(const vec4_t &q);

/**
 * Quaternion multiply
 *
 * Page 3. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param p Quaternion in (x, y, z, w)
 * @param q Quaternion in (x, y, z, w)
 * @returns Product of quaternions p and q
 */
vec4_t quatmul(const vec4_t &p, const vec4_t &q);

/**
 * Quaternion to Rotation Matrix
 *
 * Page 9. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Rotation matrix
 */
mat3_t quat2rot(const vec4_t &q);

/**
 * Quaternion left-compound
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Quaternion left-compound
 */
mat4_t quatlcomp(const vec4_t &q);

/**
 * Quaternion right-compound
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Quaternion right-compound
 */
mat4_t quatrcomp(const vec4_t &q);

/**
 * Convert the vector part of a quaternion to a full quaternion.
 *
 * This function is useful to convert delta quaternion which is usually a 3x1
 * vector to a full quaternion.  For more details, check Section 3.2 Kalman
 * Filter Update of Equation (217) and (218) in "Indirect Kalman Filter for 3D
 * Attitude Estimation" by Trawny and Roumeliotis.
 *
 * @param dtheta Small angle vector to form full quaternion with
 * @returns Small angle Quaternion
 */
vec4_t quatsmallangle(const vec3_t &dtheta);

/**
 * Quaternion to Euler-angles
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Euler-angles
 */
vec3_t quat2euler(const vec4_t &q);

/**
 * Euler-angles to Quaternion
 *
 * @param rpy Euler-angles in (roll, pitch, yaw)
 * @returns Quaternion
 */
vec4_t euler2quat(const vec3_t &rpy);

/**
 * Rotation matrix to Quaternion
 */
vec4_t rot2quat(const mat3_t &rot);

/**
 * Quaternion to Rotation Matrix
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Rotation matrix
 */
mat3_t C(const vec4_t &q);

/**
 * Omega function
 *
 * @param w Angular velocity
 * @returns Differential form of an angular velocity
 */
mat4_t Omega(const vec3_t &w);

/**
 * Quaternion Zeroth order integration
 *
 * Page 12-13 of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman
 * filter for 3D attitude estimation." University of Minnesota, Dept. of Comp.
 * Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @param w Angular velocity
 * @param dt Time difference (s)
 *
 * @returns Zeroth order integrated quaternion
 */
vec4_t quatzoi(const vec4_t &q, const vec3_t &w, const double dt);

/** @} group quaternion */
} //  namespace prototype
#include "jpl_impl.hpp"
#endif // PROTOTYPE_CORE_QUATERNION_JPL_HPP
