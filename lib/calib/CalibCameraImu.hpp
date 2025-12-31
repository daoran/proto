#pragma once

#include "CalibProblem.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/ImuError.hpp"
#include "../ceres/CalibCameraImuError.hpp"

namespace xyz {

/** Camera-IMU Calibrator **/
struct CalibCameraImu : CalibProblem {
  using CameraResiduals = std::map<int, std::vector<CalibCameraImuErrorPtr>>;
  using ImuResiduals = std::map<int, ImuErrorPtr>;
  using CameraBuffers = std::map<int, CalibTargetMap>;

  std::map<timestamp_t, CameraResiduals> camera_resblocks;
  std::map<timestamp_t, ImuResiduals> imu_resblocks;

  bool verbose = false;
  bool imu_started = false;
  bool camera_started = false;
  bool initialized = false;

  ImuBuffer imu_buffer;
  CameraBuffers camera_buffers;

  CalibCameraImu() = default;
  CalibCameraImu(const std::string &config_file);
  ~CalibCameraImu() = default;

  /** Find target with the most observations */
  std::pair<int, int> findOptimalTarget();

  /** Estimate sensor pose T_WS */
  int estimateSensorPose(Mat4 &T_WS);

  /** Estimate camera pose T_C0T0 */
  int estimateCameraPose(Mat4 &T_C0T0);

  /** Add camera calibration view */
  void addView(const timestamp_t ts, const Mat4 &T_WS);

  /** Initialize */
  void initialize(const timestamp_t ts);

  /** Add IMU measurement */
  void addMeasurement(const timestamp_t ts,
                      const Vec3 &imu_acc,
                      const Vec3 &imu_gyr);

  /** Add camera measurement */
  void addMeasurement(const timestamp_t ts,
                      const int camera_id,
                      const CalibTargetPtr &calib_target);

  /** Solve */
  void solve();
};

} // namespace xyz
