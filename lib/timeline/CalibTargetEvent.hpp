#pragma once
#include "../Core.hpp"
#include "../calib/CalibTarget.hpp"
#include "TimelineEvent.hpp"

namespace xyz {

struct CalibTargetEvent : TimelineEvent {
  int camera_index;
  std::shared_ptr<CalibTarget> calib_target;

  CalibTargetEvent(const timestamp_t ts_,
                   const int camera_index_,
                   const std::shared_ptr<CalibTarget> &calib_target_);

  virtual ~CalibTargetEvent() = default;
};

} // namespace xyz
