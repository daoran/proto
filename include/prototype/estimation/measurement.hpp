#ifndef PROTOTYPE_ESTIMATION_MEASUREMENT_HPP
#define PROTOTYPE_ESTIMATION_MEASUREMENT_HPP

#include <deque>

#include "prototype/core/math.hpp"
#include "prototype/core/time.hpp"

namespace proto {

// struct imu_measurements_t {
//   std::deque<timestamp_t> ts;
//   std::deque<vec3_t> gyro;
//   std::deque<vec3_t> accel;
// };

struct image_t {
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

struct vi_aligner_t {
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro;
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel;

  vi_aligner_t() {}

  void addGyro(const timestamp_t &ts, const vec3_t &gyro_data) {
    gyro_ts.push_back(ts);
    gyro.push_back(gyro_data);
  }

  void addAccel(const timestamp_t &ts, const vec3_t &accel_data) {
    accel_ts.push_back(ts);
    accel.push_back(accel_data);
  }
};

/**
 * Let `t0` and `t1` be timestamps from two signals. If one of them is measured
 * at a higher rate, then inorder to synchronize both is to interpolate the
 * lower rate signal.
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
std::deque<timestamp_t> interp_timestamps(const std::deque<timestamp_t> &t0,
                                          const std::deque<timestamp_t> &t1);

/**
 * Given the interpolation timestamps `interp_ts`, target timestamps
 * `target_ts` and target data `target_data`. This function will interpolate
 * the `target_data` at the interpolation points.
 */
void interp_data(const std::deque<timestamp_t> &interp_ts,
                 std::deque<timestamp_t> &target_ts,
                 std::deque<vec3_t> &target_data);

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
void sync_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1);

} // namespace proto
#endif // PROTOTYPE_ESTIMATION_MEASUREMENT_HPP
