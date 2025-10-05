#include "ImuGeometry.hpp"

namespace xyz {

ImuGeometry::ImuGeometry(const int imu_index,
                         const ImuParams &imu_params,
                         const VecX &extrinsic)
    : imu_index_{imu_index}, imu_params_{imu_params}, extrinsic_{extrinsic} {}

int ImuGeometry::getImuIndex() const { return imu_index_; }

ImuParams ImuGeometry::getImuParams() const { return imu_params_; }

VecX ImuGeometry::getExtrinsic() const { return extrinsic_; }

double *ImuGeometry::getExtrinsicPtr() { return extrinsic_.data(); }

Mat4 ImuGeometry::getTransform() const { return tf(extrinsic_); }

void ImuGeometry::setExtrinsic(const Mat4 &extrinsic) {
  extrinsic_ = tf_vec(extrinsic);
}

} // namespace xyz
