#pragma once
#include <filesystem>

#include "../core/Core.hpp"
#include "../camera/CameraGeometry.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuGeometry.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

#include "SolvePnp.hpp"
#include "AprilGrid.hpp"
#include "AprilGridConfig.hpp"
#include "AprilGridDetector.hpp"
#include "CalibTargetGeometry.hpp"

namespace xyz {

/** Calibration Problem */
struct CalibProblem {
  // Settings
  bool verbose = false;
  fs::path config_path;
  fs::path data_path;

  // Calibration Target
  std::map<int, AprilGridConfig> target_configs;

  // Data
  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, Vec7> poses;
  std::map<timestamp_t, Vec9> speed_and_biases;
  Vec7 target_pose;

  std::map<int, CameraData> camera_data;
  std::map<int, ImuBuffer> imu_data;
  std::map<int, CalibTargetGeometryPtr> target_geometries;
  std::map<int, CameraGeometryPtr> camera_geometries;
  std::map<int, ImuGeometryPtr> imu_geometries;

  // Ceres
  ceres::Problem::Options prob_options;
  std::shared_ptr<ceres::Problem> problem;
  PoseManifold pose_plus;

  CalibProblem();
  CalibProblem(const std::string &config_path_);
  virtual ~CalibProblem() = default;

  /*****************************************************************************
   * Load methods
   ****************************************************************************/

  /** Load Camera Data */
  void loadCameraData(const int camera_id);

  /** Load IMU Data */
  void loadImuData(const int imu_id);

  /*****************************************************************************
   * Camera methods
   ****************************************************************************/

  /** Add Camera */
  void addCamera(const int camera_id,
                 const std::string &camera_model,
                 const Vec2i &resolution,
                 const VecX &intrinsic,
                 const Vec7 &extrinsic);

  /** Add camera measurement */
  void addCameraMeasurement(const timestamp_t ts,
                            const int camera_id,
                            const CalibTargetPtr &calib_target);

  /** Check if we have a camera measurement already */
  bool hasCameraMeasurement(const timestamp_t ts,
                            const int camera_id,
                            const int target_id) const;

  /** Get number of cameras */
  int getNumCameras() const;

  /** Get all camera data */
  std::map<int, CameraData> &getAllCameraData();

  /** Get camera data */
  CameraData &getCameraData(const int camera_id);

  /** Get camera geometries */
  std::map<int, CameraGeometryPtr> &getAllCameraGeometries();

  /** Get camera geometry */
  CameraGeometryPtr &getCameraGeometry(const int camera_id);

  /** Initialize camera intrinsic */
  void initializeCameraIntrinsics();

  /** Initialize camera extrinsic */
  void initializeCameraExtrinsics();

  /*****************************************************************************
   * IMU methods
   ****************************************************************************/

  /** Add Imu */
  void addImu(const int imu_id,
              const ImuParams &imu_params,
              const Vec7 &extrinsic);

  /** Get number of IMUs */
  int getNumImus() const;

  /** Get all IMU data */
  std::map<int, ImuBuffer> &getAllImuData();

  /** Get IMU data */
  ImuBuffer &getImuData(const int imu_id);

  /** Get IMU geometries */
  std::map<int, ImuGeometryPtr> &getAllImuGeometries();

  /** Get IMU geometry */
  ImuGeometryPtr &getImuGeometry(const int imu_id);

  /*****************************************************************************
   * Calibration target methods
   ****************************************************************************/

  /** Add calibration target */
  void addTarget(const AprilGridConfig &config, const Vec7 &extrinsic);

  /** Set target pose */
  void setTargetPose(const Mat4 &pose);

  /** Get target pose */
  Vec7 &getTargetPose();

  /** Get target pose pointer */
  double *getTargetPosePtr();

  /** Get number of targets */
  int getNumTargets() const;

  /** Get calibration target geometries */
  std::map<int, CalibTargetGeometryPtr> &getAllTargetGeometries();

  /** Get calibration target geometry */
  CalibTargetGeometryPtr &getTargetGeometry(const int target_id);

  /** Get target point */
  Vec3 &getTargetPoint(const int target_id, const int point_id);

  /*****************************************************************************
   * Pose methods
   ****************************************************************************/

  /** Add pose */
  void addPose(const timestamp_t ts, const Mat4 &pose);

  /** Get pose */
  Vec7 &getPose(const timestamp_t ts);

  /** Get pose pointer  */
  double *getPosePtr(const timestamp_t ts);

  /*****************************************************************************
   * Speed and biases methods
   ****************************************************************************/

  /** Add speed and biases */
  void addSpeedAndBiases(const timestamp_t ts,
                         const Vec3 &v_WS,
                         const Vec3 &bias_acc = zeros(3, 1),
                         const Vec3 &bias_gyr = zeros(3, 1));

  /** Get pose */
  Vec9 &getSpeedAndBiases(const timestamp_t ts);

  /** Get pose pointer */
  double *getSpeedAndBiasesPtr(const timestamp_t ts);

  /*****************************************************************************
   * Ceres
   ****************************************************************************/

  /** Add residual block */
  void addResidualBlock(ResidualBlock *resblock);

  /*****************************************************************************
   * Misc methods
   ****************************************************************************/

  /** Print settings */
  void printSettings(FILE *fp) const;

  /** Print calibration target configs */
  void printCalibTargetConfigs(FILE *fp) const;

  /** Print target geometries */
  void printTargetGeometries(FILE *fp, const bool max_digits = false) const;

  /** Print camera geometries */
  void printCameraGeometries(FILE *fp, const bool max_digits = false) const;

  /** Print IMU geometries */
  void printImuGeometries(FILE *fp, const bool max_digits = false) const;

  /** Print target points */
  void printTargetPoints(FILE *fp) const;

  /** Print summary */
  void printSummary(FILE *fp, const bool max_digits = false) const;

  /** Save results */
  void saveResults(const std::string &save_path) const;
};

} // namespace xyz
