#pragma once
#include "../Core.hpp"

namespace xyz {

struct TimelineEvent {
  std::string type;
  timestamp_t ts;

  TimelineEvent();
  TimelineEvent(const std::string &type_, const timestamp_t &ts_);
  virtual ~TimelineEvent() = default;
};

} // namespace xyz
