#include "ImuBuffer.hpp"

namespace xyz {

int ImuBuffer::getNumMeasurements() const { return timestamps_.size(); }

timestamp_t ImuBuffer::getTimestamp(const int index) const {
  return timestamps_.at(index);
}

Vec3 ImuBuffer::getAcc(const int index) const { return acc_data_.at(index); }

Vec3 ImuBuffer::getGyr(const int index) const { return gyr_data_.at(index); }

void ImuBuffer::add(const timestamp_t ts, const Vec3 &acc, const Vec3 gyr) {
  timestamps_.push_back(ts);
  acc_data_.push_back(acc);
  gyr_data_.push_back(gyr);
}

} // namespace xyz
