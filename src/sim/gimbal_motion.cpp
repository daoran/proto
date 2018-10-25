#include "sim/gimbal_motion.hpp"

namespace prototype {

GimbalMotion::GimbalMotion() {}

GimbalMotion::GimbalMotion(const std::vector<vec2_t> &joint_setpoints,
                           const double time_dt,
                           const double time_end)
    : joint_setpoints{joint_setpoints},
      time_dt{time_dt}, time_end{time_end},
      bezier_dt{time_dt / time_end},
      scale{bezier_dt / time_dt} {
  this->update();
}

GimbalMotion::~GimbalMotion() {}

int GimbalMotion::update() {
  // Calculate pos, vel and accel on the Bezier curve at t
  this->joint_angles = bezier(this->joint_setpoints, this->bezier_t);

  // Update time
  this->time += this->time_dt;
  this->bezier_t = this->time / this->time_end;
  if (this->bezier_t > 1.0) {
    return 1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const GimbalMotion &gimbal_motion) {
  os << "gimbal_motion:" << std::endl;
  os << "\t joint_angle: " << gimbal_motion.joint_angles.transpose()
     << std::endl;
  return os;
}

} //  namespace prototype
