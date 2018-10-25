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

vec3_t p_c2_f_jacobian_wrt_theta1(const vec3_t &p_c1_f,
                                const vec6_t &tau_s,
                                const vec6_t &tau_d,
																const double &Lambda1,
                                const vec3_t &link1,
																const double &Lambda2,
                                const vec3_t &link2,
																const double theta1_offset = 0.0,
																const double theta2_offset = 0.0);

vec3_t p_c2_f_jacobian_wrt_theta2(const vec3_t &p_c1_f,
                                const vec6_t &tau_s,
                                const vec6_t &tau_d,
																const double &Lambda1,
                                const vec3_t &link1,
																const double &Lambda2,
                                const vec3_t &link2,
																const double theta1_offset = 0.0,
																const double theta2_offset = 0.0);

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_JACOBIANS_HPP
