#pragma once

#include "ImuParams.hpp"

namespace cartesian {

// Forward declaration
struct ImuGeometry;
using ImuGeometryPtr = std::shared_ptr<ImuGeometry>;

/** IMU Geometry */
struct ImuGeometry {
  int imu_id;
  ImuParams imu_params;
  VecX extrinsic;

  ImuGeometry() = delete;
  ImuGeometry(const int imu_id_,
              const ImuParams &imu_params_,
              const VecX &extrinsic_);
  virtual ~ImuGeometry() = default;

  /** Set extrinsic */
  void setExtrinsic(const Mat4 &transform);
};

} // namespace cartesian
