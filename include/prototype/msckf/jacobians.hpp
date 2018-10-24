/**
 * @file
 * @ingroup msckf
 */
#ifndef PROTOTYPE_VISION_MSCKF_JACOBIANS_HPP
#define PROTOTYPE_VISION_MSCKF_JACOBIANS_HPP

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup msckf
 * @{
 */

Vec3 p_c2_f_jacobian_wrt_theta1(const Vec3 &p_c1_f,
                                const Vec6 &tau_s,
                                const Vec6 &tau_d,
																const double &Lambda1,
                                const Vec3 &link1,
																const double &Lambda2,
                                const Vec3 &link2,
																const double theta1_offset = 0.0,
																const double theta2_offset = 0.0);

Vec3 p_c2_f_jacobian_wrt_theta2(const Vec3 &p_c1_f,
                                const Vec6 &tau_s,
                                const Vec6 &tau_d,
																const double &Lambda1,
                                const Vec3 &link1,
																const double &Lambda2,
                                const Vec3 &link2,
																const double theta1_offset = 0.0,
																const double theta2_offset = 0.0);

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_JACOBIANS_HPP
