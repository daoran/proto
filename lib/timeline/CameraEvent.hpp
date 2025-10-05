#pragma once
#include "../Core.hpp"
#include "TimelineEvent.hpp"

namespace xyz {

struct CameraEvent : TimelineEvent {
  int camera_index = -1;
  std::string image_path;
  cv::Mat frame;

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const std::string &image_path_);

  CameraEvent(const timestamp_t ts_,
              const int camera_index_,
              const cv::Mat &frame_);

  virtual ~CameraEvent() = default;
};

} // namespace xyz
