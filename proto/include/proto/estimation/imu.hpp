#include "proto/core/core.hpp"

namespace proto {

struct imu_t {
  vec3_t p_WS{0.0, 0.0, 0.0};
  vec3_t v_WS{0.0, 0.0, 0.0};
  mat3_t C_WS = I(3);
  vec3_t b_a{0.0, 0.0, 0.0};
  vec3_t b_g{0.0, 0.0, 0.0};
  double g = -9.81;
};

void imu_update(imu_t &imu,
                const vec3_t &a_S,
                const vec3_t &w_S,
                const double dt,
                const vec3_t &up = vec3_t{0.0, 0.0, 1.0});

void imu_update(const mat3_t &C_WS_i,
                const vec3_t &v_WS_i,
                const vec3_t &p_WS_i,
                const vec3_t &b_g_i,
                const vec3_t &b_a_i,
                mat3_t &C_WS_j,
                vec3_t &v_WS_j,
                vec3_t &p_WS_j,
                vec3_t &b_g_j,
                vec3_t &b_a_j);

}  // namespace proto
