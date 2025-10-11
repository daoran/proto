#pragma once

#include "CalibData.hpp"
#include "CalibView.hpp"
#include "../ceres/ImuError.hpp"
#include "../ceres/PoseManifold.hpp"

namespace xyz {

/** Camera-IMU Calibrator **/
class CalibCameraImu : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseManifold pose_plus_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, std::shared_ptr<CalibView>> calib_views_;

  bool imu_started_ = false;
  bool camera_started_ = false;
  ImuBuffer imu_buffer_;
  std::map<int, CalibTargetPtr> camera_buffer_;

  /** Add camera calibration view */
  void addView();

public:
  CalibCameraImu(const std::string &config_file);

  /** Add IMU measurement */
  void addMeasurement(const timestamp_t ts,
                      const Vec3 &imu_acc,
                      const Vec3 &imu_gyr);

  /** Add camera measurement */
  void addMeasurement(const int camera_index,
                      const CalibTargetPtr &calib_target);

  /** Solve */
  void solve();
};

} // namespace xyz
