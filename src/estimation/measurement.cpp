#include "prototype/estimation/measurement.hpp"

namespace proto {


image_t::image_t() {}

image_t::image_t(const timestamp_t ts_, const int width_, const int height_)
    : ts{ts_}, width{width_}, height{height_} {
  data = new double[width * height];
}

image_t::image_t(const timestamp_t ts_,
                 const int width_,
                 const int height_,
                 double *data_)
    : ts{ts_}, width{width_}, height{height_}, data{data_} {}

image_t::~image_t() {
  if (data) {
    free(data);
  }
}

void vi_data_add_gyro(vi_data_t &data,
                      const timestamp_t &ts,
                      const vec3_t &gyro_data) {
  data.gyro_ts.push_back(ts);
  data.gyro.push_back(gyro_data);
}

void vi_data_add_accel(vi_data_t &data,
                       const timestamp_t &ts,
                       const vec3_t &accel_data) {
  data.accel_ts.push_back(ts);
  data.accel.push_back(accel_data);
}

void vi_data_add_image(vi_data_t &data,
                       const int cam_idx,
                       const timestamp_t &ts) {
  if (ts > data.accel_ts.front() && ts > data.gyro_ts.front()) {
    return;
  }

  data.camera[cam_idx].push_back(ts);
}

void vi_data_lerp(vi_data_t &data) {
  // Interpolate accelerometer and gyroscope against each other which ever has
  // a faster rate. This is so that the accelerometer and gyroscope
  // measurements are "synchronized" via linear interpolation
  lerp_data(data.accel_ts, data.accel, data.gyro_ts, data.gyro);

  // Interpoate more IMU measurements at the camera timestamps
  for (size_t i = 0; i < data.camera.size(); i++) {
    lerp_data(data.camera[i], data.gyro_ts, data.gyro, true);
    lerp_data(data.camera[i], data.accel_ts, data.accel, true);
  }
}

std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1) {
  // Determine whether t0 or t1 has a higher rate?
  // Then create interpolation timestamps
  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  std::deque<timestamp_t> base_timestamps;

  if (t0.size() > t1.size()) {
    ts_start = t1.front();
    ts_end = t1.back();
    base_timestamps = t0;
  } else {
    ts_start = t0.front();
    ts_end = t0.back();
    base_timestamps = t1;
  }

  // Form interpolation timestamps
  std::deque<timestamp_t> lerp_ts;
  for (const auto ts : base_timestamps) {
    if (ts >= ts_start && ts <= ts_end) {
      lerp_ts.push_back(ts);
    }
  }

  return lerp_ts;
}

void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old) {
  std::deque<timestamp_t> result_ts;
  std::deque<vec3_t> result_data;

  timestamp_t t0 = target_ts.front();
  timestamp_t t1 = 0;
  vec3_t v0 = target_data.front();
  vec3_t v1 = zeros(3, 1);

  // Loop through target signal
  size_t lerp_idx = 0;
  for (size_t i = 1; i < target_ts.size(); i++) {
    const timestamp_t ts = target_ts[i];
    const vec3_t data = target_data[i];

    // Interpolate
    const bool do_interp = ((ts - lerp_ts[lerp_idx]) * 1e-9) > 0;
    if (do_interp) {
      t1 = ts;
      v1 = data;

      // Loop through interpolation points
      while (lerp_idx < lerp_ts.size()) {
        // Check if interp point is beyond interp end point
        if (t1 < lerp_ts[lerp_idx]) {
          break;
        }

        // Calculate interpolation parameter alpha
        const double num = (lerp_ts[lerp_idx] - t0) * 1e-9;
        const double den = (t1 - t0) * 1e-9;
        const double alpha = num / den;

        // Lerp and add to results
        result_data.push_back(lerp(v0, v1, alpha));
        result_ts.push_back(lerp_ts[lerp_idx]);
        lerp_idx++;
      }

      // Shift interpolation end point to start point
      t0 = t1;
      v0 = v1;

      // Reset interpolation end point
      t1 = 0;
      v1 = zeros(3, 1);
    }

    // Add end point into results, since we are retaining the old data.
    if (keep_old) {
      result_ts.push_back(ts);
      result_data.push_back(data);
    }
  }

  target_ts = result_ts;
  target_data = result_data;
}

static void align_front(const std::deque<timestamp_t> &reference,
                        std::deque<timestamp_t> &target,
                        std::deque<vec3_t> &data) {
  const auto front = reference.front();
  while (true) {
    if (target.front() < front) {
      target.pop_front();
      data.pop_front();
    } else {
      break;
    }
  }
}

static void align_back(const std::deque<timestamp_t> &reference,
                       std::deque<timestamp_t> &target,
                       std::deque<vec3_t> &data) {
  const auto back = reference.back();
  while (true) {
    if (target.back() > back) {
      target.pop_back();
      data.pop_back();
    } else {
      break;
    }
  }
}

void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1) {
  // Create interpolate timestamps
  auto lerp_ts = lerp_timestamps(ts0, ts1);

  // Interpolate
  if (ts0.size() > ts1.size()) {
    lerp_data(lerp_ts, ts1, vs1);
  } else {
    lerp_data(lerp_ts, ts0, vs0);
  }

  // Chop the front and back so both timestamps and data are sync-ed
  align_front(lerp_ts, ts1, vs1);
  align_front(lerp_ts, ts0, vs0);
  align_back(lerp_ts, ts1, vs1);
  align_back(lerp_ts, ts0, vs0);
}

} // namespace proto
