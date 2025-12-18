#pragma once

#include "ImuParams.hpp"

namespace xyz {

// Forward declaration
class ImuGeometry;
using ImuGeometryPtr = std::shared_ptr<ImuGeometry>;

/** IMU Geometry */
class ImuGeometry {
private:
  int imu_id_;
  ImuParams imu_params_;
  VecX extrinsic_;

public:
  ImuGeometry() = delete;
  ImuGeometry(const int imu_id,
              const ImuParams &imu_params,
              const VecX &extrinsic);
  virtual ~ImuGeometry() = default;

  /** Get Imu index */
  int getImuId() const;

  /** Get Imu Params */
  ImuParams getImuParams() const;

  /** Get extrinsic **/
  VecX getExtrinsic() const;

  /** Get extrinsic **/
  double *getExtrinsicPtr();

  /** Get transform T_body_camera **/
  Mat4 getTransform() const;

  /** Set extrinsic */
  void setExtrinsic(const Mat4 &extrinsic);
};

} // namespace xyz
