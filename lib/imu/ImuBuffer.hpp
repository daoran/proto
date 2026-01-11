#pragma once
#include "../core/Core.hpp"

namespace cartesian {

struct ImuBuffer {
  std::deque<timestamp_t> timestamps;
  std::deque<Vec3> acc_data;
  std::deque<Vec3> gyr_data;

  ImuBuffer() = default;
  virtual ~ImuBuffer() = default;

  /** Get number of measurements */
  int size() const;

  /** Get timestamp */
  timestamp_t getTimestamp(const int index) const;

  /** Get accelerometer measurement */
  Vec3 getAcc(const int index) const;

  /** Get gyroscope measurement */
  Vec3 getGyr(const int index) const;

  /** Add measurement */
  void add(const timestamp_t ts, const Vec3 &acc, const Vec3 gyr);

  /** Trim */
  void trim(const timestamp_t ts_end);

  /** Extract */
  ImuBuffer extract(const timestamp_t ts_start, const timestamp_t ts_end);

  /** Estimate attitude */
  Mat3 estimateAttitude(const int limit = -1) const;

  /** Print */
  void print() const;
};

} // namespace cartesian
