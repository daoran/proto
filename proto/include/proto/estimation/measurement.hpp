#ifndef PROTO_ESTIMATION_MEASUREMENT_HPP
#define PROTO_ESTIMATION_MEASUREMENT_HPP

#include <map>
#include <deque>

#include "proto/core/core.hpp"

namespace proto {

#define EVENT_CAM0 0
#define EVENT_CAM1 1
#define EVENT_ACCEL0 2
#define EVENT_GYRO0 3

struct vi_data_t {
  // Gyroscope
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro;

  // Accelerometer
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel;

  // Camera
  std::map<int, std::deque<timestamp_t>> camera;

  vi_data_t() {}
};

void vi_data_add_gyro(vi_data_t &data,
                      const timestamp_t &ts,
                      const vec3_t &gyro_data);

void vi_data_add_accel(vi_data_t &data,
                       const timestamp_t &ts,
                       const vec3_t &accel_data);

void vi_data_add_image(vi_data_t &data,
                       const int cam_idx,
                       const timestamp_t &ts);

void vi_data_lerp(vi_data_t &data);

} // namespace proto
#endif // PROTO_ESTIMATION_MEASUREMENT_HPP
