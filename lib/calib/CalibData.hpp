#pragma once
#include <ceres/ceres.h>

#include "../Core.hpp"
#include "../camera/CameraGeometry.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuGeometry.hpp"
#include "../ceres/PoseManifold.hpp"

#include "AprilGrid.hpp"

namespace xyz {

using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;
using ImuGeometryPtr = std::shared_ptr<ImuGeometry>;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CameraData = std::map<timestamp_t, CalibTargetPtr>;

/** Calibration Data */
class CalibData {
protected:
  // Settings
  std::string config_path_;
  std::string data_path_;

  // Calibration Target
  std::string target_type_;
  int tag_rows_;
  int tag_cols_;
  double tag_size_;
  double tag_spacing_;

  // Data
  std::map<int, CameraData> camera_data_;
  std::map<int, ImuBuffer> imu_data_;
  std::map<int, CameraGeometryPtr> camera_geometries_;
  std::map<int, ImuGeometryPtr> imu_geometries_;
  std::map<int, Vec3> target_points_;

  /** Load Camera Data */
  void loadCameraData(const int camera_index);

  /** Load IMU Data */
  void loadImuData(const int imu_index);

  /** Check if we have a camera measurement already */
  bool hasCameraMeasurement(const timestamp_t ts, const int camera_index) const;

  /** Print settings */
  void printSettings(FILE *fp) const;

  /** Print calibration target */
  void printCalibTarget(FILE *fp) const;

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
  void addCamera(const int camera_index,
                 const std::string &camera_model,
                 const Vec2i &resolution,
                 const VecX &intrinsic,
                 const Vec7 &extrinsic);

  /** Add Imu */
  void addImu(const int imu_index,
              const ImuParams &imu_params,
              const Vec7 &extrinsic);

  /** Add camera measurement */
  void addCameraMeasurement(const timestamp_t ts,
                            const int camera_index,
                            const std::shared_ptr<CalibTarget> &calib_target);

  /** Add calibration target point */
  void addTargetPoint(const int point_id, const Vec3 &point);

  /** Get number of cameras */
  int getNumCameras() const;

  /** Get number of IMUs */
  int getNumImus() const;

  /** Get all camera data */
  std::map<int, CameraData> &getAllCameraData();

  /** Get camera data */
  CameraData &getCameraData(const int camera_index);

  /** Get all IMU data */
  std::map<int, ImuBuffer> &getAllImuData();

  /** Get IMU data */
  ImuBuffer &getImuData(const int imu_index);

  /** Get camera geometries */
  std::map<int, CameraGeometryPtr> &getAllCameraGeometries();

  /** Get camera geometry */
  CameraGeometryPtr &getCameraGeometry(const int camera_index);

  /** Get IMU geometries */
  std::map<int, ImuGeometryPtr> &getAllImuGeometries();

  /** Get IMU geometry */
  ImuGeometryPtr &getImuGeometry(const int imu_index);

  /** Get target point */
  Vec3 &getTargetPoint(const int point_id);

  /** Print summary */
  void printSummary(FILE *fp, const bool max_digits = false) const;

  /** Save results */
  void saveResults(const std::string &save_path) const;
};

} // namespace xyz
