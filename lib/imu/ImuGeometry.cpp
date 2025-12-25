#include "ImuGeometry.hpp"

namespace xyz {

ImuGeometry::ImuGeometry(const int imu_id_,
                         const ImuParams &imu_params_,
                         const VecX &extrinsic_)
    : imu_id{imu_id_}, imu_params{imu_params_}, extrinsic{extrinsic_} {}

void ImuGeometry::setExtrinsic(const Mat4 &transform) {
  const Vec7 data = tf_vec(transform);
  for (int i = 0; i < 7; ++i) {
    extrinsic(i) = data(i);
  }
}

} // namespace xyz
