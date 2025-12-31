#include "CalibTargetEvent.hpp"

namespace cartesian {

CalibTargetEvent::CalibTargetEvent(
    const timestamp_t ts_,
    const int camera_id_,
    const int target_id_,
    const std::shared_ptr<CalibTarget> &calib_target_)
    : TimelineEvent{"CalibTargetEvent", ts_},
      camera_id{camera_id_},
      target_id{target_id_},
      calib_target{calib_target_} {}

} // namespace cartesian
