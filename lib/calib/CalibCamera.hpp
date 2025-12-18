#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

namespace xyz {

/** Camera Calibrator **/
class CalibCamera : public CalibData {
private:
  using CameraResiduals = std::map<int, std::vector<CalibCameraErrorPtr>>;

  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseManifold pose_plus_;
  std::map<timestamp_t, CameraResiduals> resblocks_;

public:
  CalibCamera();
  CalibCamera(const std::string &config_file);
  virtual ~CalibCamera() = default;

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetMap> &measurements);

  /** Solve */
  void solve();
};

} // namespace xyz
