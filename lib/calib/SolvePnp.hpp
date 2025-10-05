#pragma once
#include <ceres/ceres.h>

#include "../Core.hpp"
#include "../camera/CameraGeometry.hpp"

#include "CalibTarget.hpp"

namespace xyz {

/** SolvePnp */
class SolvePnp {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;

public:
  SolvePnp() = delete;
  SolvePnp(const std::shared_ptr<CameraGeometry> &camera_geometry);
  virtual ~SolvePnp() = default;

  /** Estimate relative pose */
  int estimate(const Vec2s &keypoints,
               const Vec3s &object_points,
               Mat4 &T_camera_object);

  static int estimate(const std::shared_ptr<CameraGeometry> &camera_geometry,
                      const std::shared_ptr<CalibTarget> &calib_target,
                      Mat4 &T_camera_target) {
    // Get calibration target measurements
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    calib_target->getMeasurements(tag_ids,
                                  corner_indicies,
                                  keypoints,
                                  object_points);
    if (keypoints.size() < 10) {
      return -1;
    }

    // Estimate relative pose
    SolvePnp pnp{camera_geometry};
    return pnp.estimate(keypoints, object_points, T_camera_target);
  }
};

} // namespace xyz
