#include "CalibTarget.hpp"

namespace cartesian {

CalibTarget::CalibTarget(const std::string &target_type,
                         const timestamp_t &ts,
                         const int camera_id,
                         const int target_id)
    : target_type{target_type}, ts{ts}, camera_id{camera_id},
      target_id{target_id} {}

} // namespace cartesian
