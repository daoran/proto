#pragma once

#include "../core/Core.hpp"

namespace cartesian {

// Forward declaration
class CalibTarget;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CalibTargetMap = std::map<int, CalibTargetPtr>;
using CameraData = std::map<timestamp_t, CalibTargetMap>;

/** Calibration Target */
class CalibTarget {
private:
  std::string target_type_;
  timestamp_t ts_ = 0;
  int camera_id_ = 0;
  int target_id_ = 0;

public:
  CalibTarget(const std::string &target_type,
              const timestamp_t &timestamp,
              const int camera_id,
              const int target_id);
  virtual ~CalibTarget() = default;

  /** Check if detected */
  virtual bool detected() const = 0;

  /** Get number detected */
  virtual int getNumDetected() const = 0;

  /** Get Target Type **/
  std::string getTargetType() const;

  /** Get Timestamp **/
  timestamp_t getTimestamp() const;

  /** Get Camera ID **/
  int getCameraId() const;

  /** Get Target ID **/
  int getTargetId() const;

  /** Get measurements **/
  virtual void getMeasurements(std::vector<int> &point_ids,
                               Vec2s &keypoints,
                               Vec3s &object_points) const = 0;
};

} // namespace cartesian
