#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "CalibView.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

namespace xyz {

/** Camera Calibrator **/
class CalibCamera : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseManifold pose_plus_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, std::shared_ptr<CalibView>> calib_views_;

public:
  CalibCamera(const std::string &config_file);

  /** Initialize camera intrinsic */
  void initializeIntrinsics();

  /** Initialize camera extrinsic */
  void initializeExtrinsics();

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetPtr> &measurements);

  /** Solve */
  void solve();
};

} // namespace xyz
