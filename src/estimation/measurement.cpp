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

std::deque<timestamp_t> interp_timestamps(const std::deque<timestamp_t> &t0,
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
  std::deque<timestamp_t> interp_ts;
  for (const auto ts : base_timestamps) {
    if (ts > ts_start && ts < ts_end) {
      interp_ts.push_back(ts);
    }
  }

  return interp_ts;
}

void interp_data(const std::deque<timestamp_t> &interp_ts,
                 std::deque<timestamp_t> &target_ts,
                 std::deque<vec3_t> &target_data) {
  std::deque<timestamp_t> result_ts;
  std::deque<vec3_t> result_data;

  timestamp_t t0 = target_ts.front();
  timestamp_t t1 = 0;
  vec3_t v0 = target_data.front();
  vec3_t v1 = zeros(3, 1);

  size_t interp_idx = 0;
  for (size_t i = 1; i < target_ts.size(); i++) {
    const timestamp_t ts = target_ts[i];
    const vec3_t data = target_data[i];

    // Interpolate
    const bool do_interp = ((ts - interp_ts[interp_idx]) * 1e-9) > 0;
    if (do_interp) {
      t1 = ts;
      v1 = data;

      // Loop through interpolation points
      while (interp_idx < interp_ts.size()) {
        // Check if interp point is beyond interp end point
        if (t1 < interp_ts[interp_idx]) {
          break;
        }

        // Calculate interpolation parameter alpha
        const double num = (interp_ts[interp_idx] - t0) * 1e-9;
        const double den = (t1 - t0) * 1e-9;
        const double alpha = num / den;

        // Linear interpolate vector
        const double x = (1.0 - alpha) * v0(0) + alpha * v1(0);
        const double y = (1.0 - alpha) * v0(1) + alpha * v1(1);
        const double z = (1.0 - alpha) * v0(2) + alpha * v1(2);

        result_data.emplace_back(x, y, z);
        result_ts.emplace_back(interp_ts[interp_idx]);
        interp_idx++;
      }

      // Shift interpolation end point to start point
      t0 = t1;
      v0 = v1;

      // Reset interpolation end point
      t1 = 0;
      v1 = zeros(3, 1);
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

void sync_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1) {
  // Create interpolate timestamps
  auto interp_ts = interp_timestamps(ts0, ts1);

  // Interpolate
  if (ts0.size() > ts1.size()) {
    interp_data(interp_ts, ts1, vs1);
  } else {
    interp_data(interp_ts, ts0, vs0);
  }

  // Chop the front and back so both timestamps and data are sync-ed
  align_front(interp_ts, ts1, vs1);
  align_front(interp_ts, ts0, vs0);
  align_back(interp_ts, ts1, vs1);
  align_back(interp_ts, ts0, vs0);
}

} // namespace proto
