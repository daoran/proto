#pragma once

#include "../core/Core.hpp"
#include "../calib/CalibTarget.hpp"
#include "TimelineEvent.hpp"

namespace xyz {

struct CalibTargetEvent : TimelineEvent {
  int camera_id;
  int target_id;
  std::shared_ptr<CalibTarget> calib_target;

  CalibTargetEvent(const timestamp_t ts_,
                   const int camera_id_,
                   const int target_id_,
                   const std::shared_ptr<CalibTarget> &calib_target_);

  virtual ~CalibTargetEvent() = default;
};

} // namespace xyz
