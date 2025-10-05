#include "Timeline.hpp"

namespace xyz {

Timeline::~Timeline() {
  for (auto &kv : data) {
    delete kv.second;
  }
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const std::string &image_path) {
  timestamps.insert(ts);
  data.insert({ts, new CameraEvent{ts, camera_index, image_path}});
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const cv::Mat &image) {
  timestamps.insert(ts);
  data.insert({ts, new CameraEvent{ts, camera_index, image}});
}

void Timeline::add(const timestamp_t &ts,
                   const Vec3 &acc,
                   const Vec3 &gyr) {
  timestamps.insert(ts);
  data.insert({ts, new ImuEvent{ts, acc, gyr}});
}

void Timeline::add(const timestamp_t &ts,
                   const int camera_index,
                   const std::shared_ptr<CalibTarget> &calib_target) {
  timestamps.insert(ts);
  data.insert({ts, new CalibTargetEvent{ts, camera_index, calib_target}});
}

int Timeline::getNumEvents(const timestamp_t &ts) {
  const auto range = data.equal_range(ts);
  return std::distance(range.first, range.second);
}

std::vector<TimelineEvent *> Timeline::getEvents(const timestamp_t &ts) {
  std::vector<TimelineEvent *> events;

  const auto range = data.equal_range(ts);
  for (auto it = range.first; it != range.second; it++) {
    events.push_back(it->second);
  }

  return events;
}
} // namespace xyz
