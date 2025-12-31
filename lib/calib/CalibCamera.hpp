#pragma once

#include "SolvePnp.hpp"
#include "CalibProblem.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

namespace cartesian {

/** Camera Calibrator **/
struct CalibCamera : CalibProblem {
  using CameraResiduals = std::map<int, std::vector<CalibCameraErrorPtr>>;
  std::map<timestamp_t, CameraResiduals> resblocks;


  CalibCamera() = default;
  CalibCamera(const std::string &config_file);
  virtual ~CalibCamera() = default;

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetMap> &measurements);

  /** Solve */
  void solve();
};

} // namespace cartesian
