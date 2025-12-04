#pragma once
#include <filesystem>

#include "../Core.hpp"
#include "../camera/CameraGeometry.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuGeometry.hpp"
#include "../ceres/PoseManifold.hpp"

#include "AprilGrid.hpp"
#include "AprilGridConfig.hpp"
#include "AprilGridDetector.hpp"
#include "CalibTargetGeometry.hpp"

namespace xyz {

using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;
using ImuGeometryPtr = std::shared_ptr<ImuGeometry>;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CalibTargetMap = std::map<int, CalibTargetPtr>;
using CalibTargetGeometryPtr = std::shared_ptr<CalibTargetGeometry>;
using CameraData = std::map<timestamp_t, CalibTargetMap>;

/** Calibration Data */
class CalibData {
protected:
  // Settings
  fs::path config_path_;
  fs::path data_path_;

  // Calibration Target
  std::map<int, AprilGridConfig> target_configs_;

  // Data
  std::map<int, CameraData> camera_data_;
  std::map<int, ImuBuffer> imu_data_;
  std::map<int, CameraGeometryPtr> camera_geometries_;
  std::map<int, ImuGeometryPtr> imu_geometries_;
  std::map<int, CalibTargetGeometryPtr> target_geometries_;

  /** Load Camera Data */
  void loadCameraData(const int camera_id);

  /** Load IMU Data */
  void loadImuData(const int imu_id);

  /** Check if we have a camera measurement already */
  bool hasCameraMeasurement(const timestamp_t ts,
                            const int camera_id,
                            const int target_id) const;

  /** Print settings */
  void printSettings(FILE *fp) const;

  /** Print calibration target configs */
  void printCalibTargetConfigs(FILE *fp) const;

  /** Print camera geometries */
  void printCameraGeometries(FILE *fp, const bool max_digits = false) const;

  /** Print IMU geometries */
  void printImuGeometries(FILE *fp, const bool max_digits = false) const;

  /** Print target points */
  void printTargetPoints(FILE *fp) const;

public:
  CalibData() = delete;
  CalibData(const std::string &config_path);
  virtual ~CalibData() = default;

  /** Add Camera */
  void addCamera(const int camera_id,
                 const std::string &camera_model,
                 const Vec2i &resolution,
                 const VecX &intrinsic,
                 const Vec7 &extrinsic);

  /** Add Imu */
  void addImu(const int imu_id,
              const ImuParams &imu_params,
              const Vec7 &extrinsic);

  /** Add camera measurement */
  void addCameraMeasurement(const timestamp_t ts,
                            const int camera_id,
                            const std::shared_ptr<CalibTarget> &calib_target);

  /** Add calibration target point */
  void addTargetPoint(const int target_id,
                      const int point_id,
                      const Vec3 &point);

  /** Get number of cameras */
  int getNumCameras() const;

  /** Get number of IMUs */
  int getNumImus() const;

  /** Get number of targets */
  int getNumTargets() const;

  /** Get all camera data */
  std::map<int, CameraData> &getAllCameraData();

  /** Get camera data */
  CameraData &getCameraData(const int camera_id);

  /** Get all IMU data */
  std::map<int, ImuBuffer> &getAllImuData();

  /** Get IMU data */
  ImuBuffer &getImuData(const int imu_id);

  /** Get camera geometries */
  std::map<int, CameraGeometryPtr> &getAllCameraGeometries();

  /** Get camera geometry */
  CameraGeometryPtr &getCameraGeometry(const int camera_id);

  /** Get IMU geometries */
  std::map<int, ImuGeometryPtr> &getAllImuGeometries();

  /** Get IMU geometry */
  ImuGeometryPtr &getImuGeometry(const int imu_id);

  /** Get calibration target geometries */
  std::map<int, CalibTargetGeometryPtr> &getAllTargetGeometries();

  /** Get calibration target geometry */
  CalibTargetGeometryPtr &getTargetGeometry(const int target_id);

  /** Get target point */
  Vec3 &getTargetPoint(const int target_id, const int point_id);

  /** Print summary */
  void printSummary(FILE *fp, const bool max_digits = false) const;

  /** Save results */
  void saveResults(const std::string &save_path) const;
};

} // namespace xyz
