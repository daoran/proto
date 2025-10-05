#pragma once

#include "ImuParams.hpp"

namespace xyz {

/** IMU Geometry */
class ImuGeometry {
private:
  int imu_index_;
  ImuParams imu_params_;
  VecX extrinsic_;

public:
  ImuGeometry() = delete;
  ImuGeometry(const int imu_index,
              const ImuParams &imu_params,
              const VecX &extrinsic);
  virtual ~ImuGeometry() = default;

  /** Get Imu index */
  int getImuIndex() const;

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
