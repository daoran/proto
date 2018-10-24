#include "prototype/control/carrot_ctrl.hpp"

namespace prototype {

int carrot_ctrl_configure(struct carrot_ctrl &cc,
                          const std::vector<Vec3> &waypoints,
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

int carrot_ctrl_closest_point(const struct carrot_ctrl &cc,
                              const Vec3 &pos,
                              Vec3 &result) {
  // Calculate closest point
  const Vec3 v1 = pos - cc.wp_start;
  const Vec3 v2 = cc.wp_end - cc.wp_start;
  const double t = v1.dot(v2) / v2.squaredNorm();
  result = cc.wp_start + t * v2;

  return t;
}

int carrot_ctrl_carrot_point(const struct carrot_ctrl &cc,
                             const Vec3 &pos,
                             Vec3 &result) {
  Vec3 closest_pt;
  int t = carrot_ctrl_closest_point(cc, pos, closest_pt);

  if (t == -1) {
    // Closest point is before wp_start
    result = cc.wp_start;

  } else if (t == 0) {
    // Closest point is between wp_start wp_end
    const Vec3 u = cc.wp_end - cc.wp_start;
    const Vec3 v = u / u.norm();
    result = closest_pt + cc.look_ahead_dist * v;

  } else if (t == 1) {
    // Closest point is after wp_end
    result = cc.wp_end;
  }

  return t;
}

int carrot_ctrl_update(struct carrot_ctrl &cc,
                       const Vec3 &pos,
                       Vec3 &carrot_pt) {
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

} //  namespace prototype
