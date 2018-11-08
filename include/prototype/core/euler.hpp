#ifndef PROTOTYPE_CORE_EULER_HPP
#define PROTOTYPE_CORE_EULER_HPP

#include "prototype/core/math.hpp"

namespace prototype {

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed)
 *
 * @param theta Angle in radians
 * @returns Rotation matrix
 */
mat3_t rotx(const double theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed)
 *
 * @param theta Angle in radians
 * @returns Rotation matrix
 */
mat3_t roty(const double theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed)
 *
 * @param theta Angle in radians
 * @returns Rotation matrix
 */
mat3_t rotz(const double theta);

/**
 * Convert euler sequence 123 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @param euler Euler angle (roll, pitch, yaw)
 * @returns Rotation matrix
 */
mat3_t euler123ToRot(const vec3_t &euler);

/**
 * Convert euler sequence 321 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @param euler Euler angle (roll, pitch, yaw)
 * @returns Rotation matrix
 */
mat3_t euler321ToRot(const vec3_t &euler);

} //  namespace prototype
#endif // PROTOTYPE_CORE_EULER_HPP
