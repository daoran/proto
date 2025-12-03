#include "CalibTarget.hpp"

namespace xyz {

CalibTarget::CalibTarget(const std::string &target_type,
                         const timestamp_t &ts,
                         const int camera_id,
                         const int target_id)
    : target_type_{target_type}, ts_{ts}, camera_id_{camera_id},
      target_id_{target_id} {}

std::string CalibTarget::getTargetType() const { return target_type_; }

timestamp_t CalibTarget::getTimestamp() const { return ts_; }

int CalibTarget::getCameraId() const { return camera_id_; }

int CalibTarget::getTargetId() const { return target_id_; }

} // namespace xyz
