#include "prototype/control/carrot_ctrl.hpp"

namespace proto {

carrot_ctrl_t::carrot_ctrl_t() {}

carrot_ctrl_t::~carrot_ctrl_t() {}

int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const double look_ahead_dist) {
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
  const double t = v1.dot(v2) / v2.squaredNorm();
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

} //  namespace proto
