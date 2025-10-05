#pragma once
#include "../Core.hpp"
#include "TimelineEvent.hpp"

namespace xyz {

struct ImuEvent : TimelineEvent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Vec3 acc = zeros(3, 1);
  Vec3 gyr = zeros(3, 1);

  ImuEvent(const timestamp_t ts_, const Vec3 &acc_, const Vec3 &gyr_);
  virtual ~ImuEvent() = default;
};

} // namespace xyz
