#include "CalibTargetGeometry.hpp"

namespace xyz {

CalibTargetGeometry::CalibTargetGeometry(const int target_id_,
                                         const Vec7 &extrinsic_,
                                         const std::map<int, Vec3> &points_)
    : target_id{target_id_}, extrinsic{extrinsic_}, points{points_} {}

void CalibTargetGeometry::setExtrinsic(const Mat4 &transform) {
  const Vec7 data = tf_vec(transform);
  for (int i = 0; i < 7; ++i) {
    extrinsic(i) = data(i);
  }
}

} // namespace xyz
