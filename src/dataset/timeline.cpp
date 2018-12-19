#include "prototype/dataset/timeline.hpp"

namespace proto {

timeline_event_t::timeline_event_t() {}

timeline_event_t::timeline_event_t(const long ts_,
                                   const vec3_t &a_m_,
                                   const vec3_t &w_m_)
    : type{IMU_EVENT}, ts{ts_}, a_m{a_m_}, w_m{w_m_} {}

timeline_event_t::timeline_event_t(const long ts_,
                                   const int camera_index_,
                                   const std::string &image_path_)
    : type{CAMERA_EVENT}, ts{ts_}, camera_index{camera_index_},
      image_path{image_path_} {}

timeline_event_t::~timeline_event_t() {}

timeline_t::timeline_t() {}

timeline_t::~timeline_t() {}

} // namespace proto
