#include "CalibTargetGeometry.hpp"

namespace xyz {

CalibTargetGeometry::CalibTargetGeometry(
    const int target_id,
    const Vec7 &extrinsic,
    const std::map<int, Vec3> &target_points)
    : target_id_{target_id}, extrinsic_{extrinsic}, target_points_{
                                                        target_points} {}

int CalibTargetGeometry::getTargetId() const { return target_id_; }

Vec7 CalibTargetGeometry::getExtrinsic() const { return extrinsic_; }

double *CalibTargetGeometry::getExtrinsicPtr() { return extrinsic_.data(); }

Mat4 CalibTargetGeometry::getTransform() const { return tf(extrinsic_); }

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

void CalibTargetGeometry::setExtrinsic(const Mat4 &extrinsic) {
  extrinsic_ = tf_vec(extrinsic);
}

} // namespace xyz
