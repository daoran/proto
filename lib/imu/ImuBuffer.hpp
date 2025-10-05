#pragma once
#include "Core.hpp"

namespace xyz {

class ImuBuffer {
private:
  std::vector<timestamp_t> timestamps_;
  std::vector<Vec3> acc_data_;
  std::vector<Vec3> gyr_data_;

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
};

} // namespace xyz
