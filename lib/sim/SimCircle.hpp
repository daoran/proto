#pragma once
#include "../core/Core.hpp"

namespace cartesian {

struct SimCircle {
  double circle_r = 5.0;
  double circle_v = 1.0;
  double circle_dist = 2.0 * M_PI * circle_r;
  double time_taken = circle_dist / circle_v;
  double w = -2.0 * M_PI * (1.0 / time_taken);
  double theta_init = M_PI;
  double yaw_init = M_PI / 2.0;

  Vec3 get_position(const double time_s) const;
  Mat3 get_rotation(const double time_s) const;
  Mat4 get_pose(const double time_s);
  Vec3 get_velocity(const double time_s) const;
  Vec3 get_acceleration(const double time_s) const;
};

} // namespace cartesian
