#pragma once

#include "CalibData.hpp"
#include "CalibView.hpp"
#include "../ImuError.hpp"

namespace xyz {

/** Camera-IMU Calibrator **/
class CalibCameraImu : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseManifold pose_plus_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, std::shared_ptr<CalibView>> calib_views_;

  ImuBuffer imu_buffer_;

public:
  CalibCameraImu(const std::string &config_file);

  /** Add Imu measurement */
  void addImuMeasurement(const timestamp_t ts,
                         const Vec3 &imu_acc,
                         const Vec3 &imu_gyr);

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetPtr> &measurements);

  /** Solve */
  void solve();
};

} // namespace xyz
