#include "TimelineEvent.hpp"

namespace cartesian {

TimelineEvent::TimelineEvent() : type{""}, ts{0} {}

TimelineEvent::TimelineEvent(const std::string &type_, const timestamp_t &ts_)
    : type{type_}, ts{ts_} {}

} // namespace cartesian
