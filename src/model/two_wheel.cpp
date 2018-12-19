#include "prototype/model/two_wheel.hpp"

namespace proto {

void circle_trajectory(const double r,
                       const double v,
                       double *w,
                       double *time) {
  const double dist = 2 * M_PI * r;
  *time = dist / v;
  *w = (2 * M_PI) / *time;
}

void two_wheel_update(two_wheel_t &tm, const double dt) {
  const vec3_t p_G_prev = tm.p_G;
  const vec3_t v_G_prev = tm.v_G;
  const vec3_t rpy_G_prev = tm.rpy_G;

  tm.p_G += euler321ToRot(tm.rpy_G) * tm.v_B * dt;
  tm.v_G = (tm.p_G - p_G_prev) / dt;
  tm.a_G = (tm.v_G - v_G_prev) / dt;

  tm.rpy_G += euler321ToRot(tm.rpy_G) * tm.w_B * dt;
  tm.w_G = tm.rpy_G - rpy_G_prev;
  tm.a_B = euler123ToRot(tm.rpy_G) * tm.a_G;

  // Wrap angles to +/- pi
  for (int i = 0; i < 3; i++) {
    tm.rpy_G(i) = (tm.rpy_G(i) > M_PI) ? tm.rpy_G(i) - 2 * M_PI : tm.rpy_G(i);
    tm.rpy_G(i) = (tm.rpy_G(i) < -M_PI) ? tm.rpy_G(i) + 2 * M_PI : tm.rpy_G(i);
  }
}

} //  namespace proto
