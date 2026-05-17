#pragma once
#include "../core/Core.hpp"
#include "../core/SO3.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuParams.hpp"
#include "../imu/ImuPreintegrate.hpp"
#include "../timeline/Timeline.hpp"
#include "SolvePnp.hpp"

namespace cartesian {

/** Calib Initializer **/
struct CalibInit {
  Mat4 initializeCameraImuExtrinsic(
      const Timeline &timeline,
      const std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
      const ImuParams &imu_params);
};

} // namespace cartesian
