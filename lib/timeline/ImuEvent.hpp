#pragma once

#include "../core/Core.hpp"
#include "TimelineEvent.hpp"

namespace cartesian {

struct ImuEvent : TimelineEvent {
  Vec3 acc = zeros(3, 1);
  Vec3 gyr = zeros(3, 1);

  ImuEvent(const timestamp_t ts_, const Vec3 &acc_, const Vec3 &gyr_);
  virtual ~ImuEvent() = default;
};

} // namespace cartesian
