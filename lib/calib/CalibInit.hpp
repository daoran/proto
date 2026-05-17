#pragma once
#include "../core/Core.hpp"
#include "../core/SO3.hpp"
#include "../calib/AprilGridConfig.hpp"
#include "../ceres/CalibCameraError.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuParams.hpp"
#include "../imu/ImuPreintegrate.hpp"
#include "../timeline/Timeline.hpp"
#include "CameraChain.hpp"
#include "CalibTargetChain.hpp"
#include "SolvePnp.hpp"

namespace cartesian {

/** Calib Initializer **/
struct CalibInit {
  /** Initialize camera intrinsics */
  static void initializeCameraIntrinsics(
      const std::map<int, CameraData> camera_data,
      const std::map<int, AprilGridConfig> target_configs,
      std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
      const bool verbose = false);

  /** Initialize camera-camera extrinsics */
  static void initializeCameraExtrinsics(
      const std::map<int, CameraData> camera_data,
      std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
      std::map<int, CalibTargetGeometryPtr> &target_geoms,
      const bool verbose = false);

  /** Initialize camera-imu extrinsics */
  static Mat4 initializeCameraImuExtrinsic(
      const Timeline &timeline,
      const std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
      const ImuParams &imu_params);
};

} // namespace cartesian
