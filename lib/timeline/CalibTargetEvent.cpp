#include "CalibTargetEvent.hpp"

namespace xyz {

CalibTargetEvent::CalibTargetEvent(
    const timestamp_t ts_,
    const int camera_index_,
    const std::shared_ptr<CalibTarget> &calib_target_)
    : TimelineEvent{"CalibTargetEvent", ts_}, camera_index{camera_index_},
      calib_target{calib_target_} {}

} // namespace xyz
