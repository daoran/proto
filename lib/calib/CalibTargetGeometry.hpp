#pragma once
#include "../core/Core.hpp"

namespace cartesian {

// Forward declaration
struct CalibTargetGeometry;
using CalibTargetGeometryPtr = std::shared_ptr<CalibTargetGeometry>;

/** Calibration Target Geometry */
struct CalibTargetGeometry {
  int target_id;
  Vec7 extrinsic;
  std::map<int, Vec3> points;

  CalibTargetGeometry() = delete;
  CalibTargetGeometry(const int target_id_,
                      const Vec7 &extrinsic_,
                      const std::map<int, Vec3> &target_points_);
  virtual ~CalibTargetGeometry() = default;

  /** Set extrinsic */
  void setExtrinsic(const Mat4 &extrinsic);
};

} // namespace cartesian
