#pragma once
#include "../core/Core.hpp"

namespace xyz {

class ImuBuffer {
private:
  std::deque<timestamp_t> timestamps_;
  std::deque<Vec3> acc_data_;
  std::deque<Vec3> gyr_data_;

public:
  ImuBuffer() = default;
  virtual ~ImuBuffer() = default;

  int getNumMeasurements() const;

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

} // namespace xyz
