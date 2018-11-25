#include "prototype/calib/calib_stereo.hpp"

namespace prototype {

stereo_residual_t::stereo_residual_t(const vec2_t &z_C0,
                                     const vec2_t &z_C1,
                                     const vec3_t &p_F)
    : z_C0_{z_C0(0), z_C0(1)},
      z_C1_{z_C1(0), z_C1(1)},
      p_F_{p_F(0), p_F(1), p_F(2)} {}

stereo_residual_t::~stereo_residual_t() {}

} //  namespace prototype
