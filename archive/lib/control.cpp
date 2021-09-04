#include "control.hpp"

namespace proto {

pid_t::pid_t() {}

pid_t::pid_t(const real_t k_p_, const real_t k_i_, const real_t k_d_)
    : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}

pid_t::~pid_t() {}

std::ostream &operator<<(std::ostream &os, const pid_t &pid) {
  os << "error_prev:" << pid.error_prev << std::endl;
  os << "error_sum:" << pid.error_sum << std::endl;
  os << "error_p:" << pid.error_p << std::endl;
  os << "error_i:" << pid.error_i << std::endl;
  os << "error_d:" << pid.error_d << std::endl;
  os << "k_p:" << pid.k_p << std::endl;
  os << "k_i:" << pid.k_i << std::endl;
  os << "k_d:" << pid.k_d << std::endl;
  return os;
}

real_t pid_update(pid_t &p,
                  const real_t setpoint,
                  const real_t actual,
                  const real_t dt) {
  // Calculate errors
  const real_t error = setpoint - actual;
  p.error_sum += error * dt;

  // Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  const real_t output = p.error_p + p.error_i + p.error_d;

  p.error_prev = error;
  return output;
}

real_t pid_update(pid_t &p, const real_t error, const real_t dt) {
  return pid_update(p, error, 0.0, dt);
}

void pid_reset(pid_t &p) {
  p.error_prev = 0.0;
  p.error_sum = 0.0;

  // p.error_p = 0.0;
  // p.error_i = 0.0;
  // p.error_d = 0.0;
}

carrot_ctrl_t::carrot_ctrl_t() {}

carrot_ctrl_t::~carrot_ctrl_t() {}

int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const real_t look_ahead_dist) {
  if (waypoints.size() <= (size_t) 2) {
    LOG_ERROR("Too few waypoints!");
    return -1;
  }

  cc.waypoints = waypoints;
  cc.wp_start = cc.waypoints[0];
  cc.wp_end = cc.waypoints[1];
  cc.wp_index = 1;
  cc.look_ahead_dist = look_ahead_dist;

  return 0;
}

int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result) {
  // Calculate closest point
  const vec3_t v1 = pos - cc.wp_start;
  const vec3_t v2 = cc.wp_end - cc.wp_start;
  const real_t t = v1.dot(v2) / v2.squaredNorm();
  result = cc.wp_start + t * v2;

  return t;
}

int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result) {
  vec3_t closest_pt;
  int t = carrot_ctrl_closest_point(cc, pos, closest_pt);

  if (t == -1) {
    // Closest point is before wp_start
    result = cc.wp_start;

  } else if (t == 0) {
    // Closest point is between wp_start wp_end
    const vec3_t u = cc.wp_end - cc.wp_start;
    const vec3_t v = u / u.norm();
    result = closest_pt + cc.look_ahead_dist * v;

  } else if (t == 1) {
    // Closest point is after wp_end
    result = cc.wp_end;
  }

  return t;
}

int carrot_ctrl_update(carrot_ctrl_t &cc,
                       const vec3_t &pos,
                       vec3_t &carrot_pt) {
  // Calculate new carot point
  int status = carrot_ctrl_carrot_point(cc, pos, carrot_pt);

  // Check if there are more waypoints
  if ((cc.wp_index + 1) == cc.waypoints.size()) {
    return 1;
  }

  // Update waypoints
  if (status == 1) {
    cc.wp_index++;
    cc.wp_start = cc.wp_end;
    cc.wp_end = cc.waypoints[cc.wp_index];
  }

  return 0;
}

} // namespace proto
