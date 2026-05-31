#pragma once

#include "../core/Core.hpp"

namespace cartesian {

// Forward declaration
struct CalibTarget;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CalibTargetMap = std::map<int, CalibTargetPtr>;
using CalibTargetData = std::map<timestamp_t, CalibTargetMap>;

/** Calibration Target */
struct CalibTarget {
  std::string target_type;
  timestamp_t ts = 0;
  int camera_id = 0;
  int target_id = 0;

  CalibTarget(const std::string &target_type,
              const timestamp_t &timestamp,
              const int camera_id,
              const int target_id);
  virtual ~CalibTarget() = default;

  /** Check if detected */
  virtual bool detected() const = 0;

  /** Get number detected */
  virtual int get_num_detected() const = 0;

  /** Get measurements **/
  virtual void get_measurements(std::vector<int> &point_ids,
                                Vec2s &keypoints,
                                Vec3s &object_points) const = 0;
};

} // namespace cartesian
