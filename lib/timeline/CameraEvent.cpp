#include "CameraEvent.hpp"

namespace xyz {

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_id_,
                         const std::string &image_path_)
    : TimelineEvent{"CameraEvent", ts_}, camera_id{camera_id_},
      image_path{image_path_} {}

CameraEvent::CameraEvent(const timestamp_t ts_,
                         const int camera_id_,
                         const cv::Mat &frame_)
    : TimelineEvent{"CameraEvent", ts_}, camera_id{camera_id_}, frame{frame_} {}

} // namespace xyz
