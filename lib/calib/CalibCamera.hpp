#pragma once

#include "SolvePnp.hpp"
#include "CalibData.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

namespace xyz {

using CalibCameraErrorPtr = std::shared_ptr<CalibCameraError>;
using CameraResiduals = std::map<int, std::vector<CalibCameraErrorPtr>>;

/** Camera Calibrator **/
class CalibCamera : public CalibData {
private:
  ceres::Problem::Options prob_options_;
  std::shared_ptr<ceres::Problem> problem_;
  PoseManifold pose_plus_;

  std::set<timestamp_t> timestamps_;
  std::map<timestamp_t, Vec7> poses_;
  std::map<timestamp_t, CameraResiduals> resblocks_;

public:
  CalibCamera();
  CalibCamera(const std::string &config_file);
  virtual ~CalibCamera() = default;

  /** Initialize camera intrinsic */
  void initializeIntrinsics();

  /** Initialize camera extrinsic */
  void initializeExtrinsics();

  /** Add camera calibration view */
  void addView(const std::map<int, CalibTargetMap> &measurements);

  /** Solve */
  void solve();
};

} // namespace xyz
