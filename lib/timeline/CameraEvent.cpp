#include "CameraEvent.hpp"

namespace xyz {

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_index_,
                         const std::string &image_path_)
    : TimelineEvent{"CameraEvent", ts_}, camera_index{camera_index_},
      image_path{image_path_} {}

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_index_,
                         const cv::Mat &frame_)
    : TimelineEvent{"CameraEvent", ts_},
      camera_index{camera_index_}, frame{frame_} {}

} // namespace xyz
