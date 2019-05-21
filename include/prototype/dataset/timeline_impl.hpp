#ifndef PROTOTYPE_DATASET_TIMELINE_IMPL_HPP
#define PROTOTYPE_DATASET_TIMELINE_IMPL_HPP

#include "prototype/dataset/timeline.hpp"

namespace proto {

template <typename T>
timeline_event_t<T>::timeline_event_t() {}

template <typename T>
timeline_event_t<T>::timeline_event_t(const T ts_,
                                      const vec3_t &a_m_,
                                      const vec3_t &w_m_)
    : type{IMU_EVENT}, ts{ts_}, a_m{a_m_}, w_m{w_m_} {}

template <typename T>
timeline_event_t<T>::timeline_event_t(const T ts_,
                                      const int camera_index_,
                                      const std::string &image_path_)
    : type{CAMERA_EVENT}, ts{ts_}, camera_index{camera_index_},
      image_path{image_path_} {}

template <typename T>
timeline_event_t<T>::timeline_event_t(const T ts_,
                                      const std::string &object_name_,
                                      const vec3_t &r_WM_,
                                      const quat_t &q_WM_)
    : type{VICON_EVENT}, ts{ts_},
      object_name{object_name_}, r_WM{r_WM_}, q_WM{q_WM_} {}

template <typename T>
timeline_event_t<T>::timeline_event_t(const T ts_,
                                      const int camera_index_,
                                      const aprilgrid_t &grid_)
    : type{APRILGRID_EVENT}, ts{ts_}, camera_index{camera_index_}, grid{grid_} {
}

template <typename T>
timeline_event_t<T>::~timeline_event_t() {}

template <typename T>
timeline_t<T>::timeline_t() {}

template <typename T>
timeline_t<T>::~timeline_t() {}

template <typename T>
void timeline_add_event(timeline_t<T> &timeline,
                        const timeline_event_t<T> &event) {
  const auto &ts = event.ts;
  timeline.data.insert({ts, event});
  timeline.timestamps.insert(ts);
}

} //  namespace proto
#endif // PROTOTYPE_DATASET_TIMELINE_IMPL_HPP
