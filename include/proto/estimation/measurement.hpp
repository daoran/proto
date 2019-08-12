#ifndef PROTO_ESTIMATION_MEASUREMENT_HPP
#define PROTO_ESTIMATION_MEASUREMENT_HPP

#include <map>
#include <deque>

#include "proto/core/math.hpp"
#include "proto/core/time.hpp"

namespace proto {

struct measurement_t {
  timestamp_t ts = 0;

  measurement_t() {}
};

struct gyro_t : measurement_t {
  double x = 0;
  double y = 0;
  double z = 0;

  gyro_t() {}

  vec3_t data() {
    return vec3_t{x, y, z};
  }
};

struct accel_t {
  double x = 0;
  double y = 0;
  double z = 0;

  accel_t() {}

  vec3_t data() {
    return vec3_t{x, y, z};
  }
};

struct image_t : measurement_t {
  timestamp_t ts = 0;
  int width = 0;
  int height = 0;
  double *data = nullptr;

  image_t();
  image_t(const timestamp_t ts, const int width, const int height);
  image_t(const timestamp_t ts,
          const int width,
          const int height,
          double *data);
  ~image_t();
};

#define EVENT_CAM0 0
#define EVENT_CAM1 1
#define EVENT_ACCEL0 2
#define EVENT_GYRO0 3

struct imu_data_t {
  bool gyro_started = false;
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro;

  bool accel_started = false;
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel;
};

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

/**
 * Let `t0` and `t1` be timestamps from two signals. If one of them is measured
 * at a higher rate, the goal is to interpolate the lower rate signal so that
 * it aligns with the higher rate one.
 *
 * This function will determine which timestamp deque will become the reference
 * signal and the other will become the target signal. Based on this the
 * interpolation points will be based on the reference signal.
 *
 * Additionally, this function ensures the interpolation timestamps are
 * achievable by:
 *
 * - interp start > target start
 * - interp end < target end
 *
 * **Note**: This function will not include timestamps from the target
 * (lower-rate) signal. The returned interpolation timestamps only returns
 * **interpolation points** to match the reference signal (higher-rate).
 *
 * @returns Interpolation timestamps from two timestamp deques `t0` and `t1`.
 */
std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1);

/**
 * Given the interpolation timestamps `lerp_ts`, target timestamps
 * `target_ts` and target data `target_data`. This function will interpolate
 * the `target_data` at the interpolation points.
 */
void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old = false);

/**
 * Given two data signals with timestamps `ts0`, `vs0`, `ts1`, and `vs1`, this
 * function determines which data signal is at a lower rate and performs linear
 * interpolation inorder to synchronize against the higher rate data signal.
 *
 * The outcome of this function is that both data signals will have:
 *
 * - Same number of timestamps.
 * - Lower-rate data will be interpolated against the higher rate data.
 *
 * **Note**: This function will drop values from the start and end of both
 * signals inorder to synchronize them.
 */
void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1);

} // namespace proto
#endif // PROTO_ESTIMATION_MEASUREMENT_HPP
