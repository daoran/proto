#include "CalibTarget.hpp"

namespace cartesian {

CalibTarget::CalibTarget(const std::string &target_type,
                         const timestamp_t &ts,
                         const int camera_id,
                         const int target_id)
    : target_type_{target_type}, ts_{ts}, camera_id_{camera_id},
      target_id_{target_id} {}

std::string CalibTarget::get_target_type() const { return target_type_; }
timestamp_t CalibTarget::get_timestamp() const { return ts_; }
int CalibTarget::get_camera_id() const { return camera_id_; }
int CalibTarget::get_target_id() const { return target_id_; }

} // namespace cartesian
