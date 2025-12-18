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

void ImuBuffer::trim(const timestamp_t ts_end) {
  // Pre-check
  if (timestamps_.size() == 0) {
    return;
  }

  // Make sure the trim timestamp is after the first imu measurement
  if (ts_end < timestamps_.front()) {
    return;
  }

  // Trim IMU measurements
  while (ts_end > timestamps_.front()) {
    timestamps_.pop_front();
    acc_data_.pop_front();
    gyr_data_.pop_front();
  }
}

ImuBuffer ImuBuffer::extract(const timestamp_t ts_start,
                             const timestamp_t ts_end) {
  assert(ts_start >= timestamps_.front());
  assert(ts_end <= timestamps_.back());

  // Extract data between ts_start and ts_end
  ImuBuffer buf;
  for (size_t k = 0; k < timestamps_.size(); k++) {
    // Check if within the extraction zone
    auto ts_k = timestamps_[k];
    if (ts_k < ts_start) {
      continue;
    }

    // Setup
    const Vec3 acc_k = acc_data_[k];
    const Vec3 gyr_k = gyr_data_[k];

    // Interpolate start
    if (buf.timestamps_.size() == 0 and ts_k > ts_start) {
      const timestamp_t ts_km1 = timestamps_[k - 1];
      const Vec3 acc_km1 = acc_data_[k - 1];
      const Vec3 gyr_km1 = gyr_data_[k - 1];

      const auto alpha = ts2sec(ts_start - ts_km1) / ts2sec(ts_k - ts_km1);
      const Vec3 acc_lerped = (1.0 - alpha) * acc_km1 + alpha * acc_k;
      const Vec3 gyr_lerped = (1.0 - alpha) * gyr_km1 + alpha * gyr_k;
      buf.add(ts_start, acc_lerped, gyr_lerped);
    }

    // Interpolate end
    if (ts_k > ts_end) {
      const timestamp_t ts_km1 = timestamps_[k - 1];
      const Vec3 acc_km1 = acc_data_[k - 1];
      const Vec3 gyr_km1 = gyr_data_[k - 1];

      const auto alpha = ts2sec(ts_end - ts_km1) / ts2sec(ts_k - ts_km1);
      const Vec3 acc_lerped = (1.0 - alpha) * acc_km1 + alpha * acc_k;
      const Vec3 gyr_lerped = (1.0 - alpha) * gyr_km1 + alpha * gyr_k;
      buf.add(ts_end, acc_lerped, gyr_lerped);
      break;
    }

    // Add to subset
    buf.add(ts_k, acc_k, gyr_k);

    // End?
    if (ts_k >= ts_end) {
      break;
    }
  }

  return buf;
}

Mat3 ImuBuffer::estimateAttitude(const int limit) const {
  // Sample IMU measurements
  Vec3 sum_angular_vel{0.0, 0.0, 0.0};
  Vec3 sum_linear_acc{0.0, 0.0, 0.0};
  auto buf_size = (limit == -1) ? timestamps_.size() : limit;
  for (size_t k = 0; k < buf_size; k++) {
    sum_angular_vel += gyr_data_[k];
    sum_linear_acc += acc_data_[k];
  }

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  const Vec3 mean_accel = sum_linear_acc / buf_size;
  const Vec3 gravity{0.0, 0.0, -9.81};
  const Mat3 C_WS = vecs2rot(mean_accel, -gravity);

  return C_WS;
}

void ImuBuffer::print() const {
  auto buf_size = timestamps_.size();
  for (size_t k = 0; k < buf_size; k++) {
    const timestamp_t ts = timestamps_[k];
    const Vec3 &acc = acc_data_[k];
    const Vec3 &gyr = gyr_data_[k];
    printf("ts: %18ld, ", ts);
    printf("acc: [%4.4f, %4.4f, %4.4f], ", acc.x(), acc.y(), acc.z());
    printf("gyr: [%4.4f, %4.4f, %4.4f]", gyr.x(), gyr.y(), gyr.z());
    printf("\n");
  }
}

} // namespace xyz
