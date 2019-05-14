#ifndef PROTOTYPE_ROVIO_FRONTEND_HPP
#define PROTOTYPE_ROVIO_FRONTEND_HPP

#include "prototype/core/math.hpp"

namespace proto {

struct feature_t {
  vec2f_t pixel;
  vec2f_t bearing;
};

struct multi_level_patch_t {
  int levels = 0;   // Number of pyramid levels.
  mat3f_t H;        // Hessian matrix.
  float e0 = 0.0f;  // Smaller eigenvalue of H.
  float e1 = 0.0f;  // Larger eigenvalue of H.
  float s = 0.0f;   // Shi-Tomasi score.
};

} //  namespace proto
#endif // PROTOTYPE_ROVIO_FRONTEND_HPP
