#include "CalibTargetGeometry.hpp"

namespace xyz {

CalibTargetGeometry::CalibTargetGeometry(const int target_id,
                                         const Vec7 &target_pose)
    : target_id_{target_id}, target_pose_{target_pose} {}

Vec7 CalibTargetGeometry::getPose() const { return target_pose_; }

double *CalibTargetGeometry::getPosePtr() { return target_pose_.data(); }

std::map<int, Vec3> &CalibTargetGeometry::getPoints() { return target_points_; }

Vec3 &CalibTargetGeometry::getPoint(const int point_id) {
  assert(target_points_.count(point_id));
  return target_points_.at(point_id);
}

double *CalibTargetGeometry::getPointPtr(const int point_id) {
  assert(target_points_.count(point_id));
  return target_points_.at(point_id).data();
}

void CalibTargetGeometry::addPoint(const int point_id, const Vec3 &point) {
  assert(target_points_.count(point_id) == 0);
  target_points_[point_id] = point;
}

} // namespace xyz
