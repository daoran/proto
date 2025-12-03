#pragma once
#include "../Core.hpp"
#include "CalibTargetEvent.hpp"
#include "CameraEvent.hpp"
#include "ImuEvent.hpp"

namespace xyz {

/** Timeline **/
struct Timeline {
  std::set<timestamp_t> timestamps;
  std::multimap<timestamp_t, TimelineEvent *> data;

  Timeline() = default;

  virtual ~Timeline();

  /** Add camera event **/
  void add(const timestamp_t &ts,
           const int camera_id,
           const std::string &image_path);

  /** Add camera event **/
  void add(const timestamp_t &ts, const int camera_id, const cv::Mat &image);

  /** Add imu event **/
  void add(const timestamp_t &ts, const Vec3 &acc, const Vec3 &gyr);

  /** Add calibration target event **/
  void add(const timestamp_t &ts,
           const int camera_id,
           const int target_id,
           const std::shared_ptr<CalibTarget> &calib_target);

  /** Get number of events **/
  int getNumEvents(const timestamp_t &ts);

  /** Get events **/
  std::vector<TimelineEvent *> getEvents(const timestamp_t &ts);
};

} // namespace xyz
