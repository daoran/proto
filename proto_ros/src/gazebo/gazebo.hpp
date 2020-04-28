#ifndef PROTO_ROS_GAZEBO_HPP
#define PROTO_ROS_GAZEBO_HPP

#include <proto/proto.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo {

ignition::math::Vector3d convert(const proto::vec3_t &v) {
  return ignition::math::Vector3d{v(0), v(1), v(2)};
}

ignition::math::Quaterniond convert(const proto::quat_t &q) {
  return ignition::math::Quaterniond{q.w(), q.x(), q.y(), q.z()};
}

proto::vec3_t convert(const ignition::math::Vector3d &v) {
  return proto::vec3_t{v.X(), v.Y(), v.Z()};
}

proto::quat_t convert(const ignition::math::Quaterniond &q) {
  return proto::quat_t{q.W(), q.X(), q.Y(), q.Z()};
}

proto::mat4_t convert(const ignition::math::Pose3d &pose) {
  const proto::vec3_t r_WR = convert(pose.Pos());
  const proto::quat_t q_WR = convert(pose.Rot());
  return proto::tf(q_WR, r_WR);
}

} // namespace gazebo
#endif // PROTO_ROS_GAZEBO_HPP
