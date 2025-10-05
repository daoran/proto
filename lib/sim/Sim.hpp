#pragma once
#include "../Core.hpp"
#include "../imu/ImuBuffer.hpp"

namespace xyz {

struct Sim {
  // Data
  std::vector<timestamp_t> timestamps;
  std::map<timestamp_t, mat4_t> poses;
  std::map<timestamp_t, Vec3> vel;
  std::map<timestamp_t, Vec3> imu_acc;
  std::map<timestamp_t, Vec3> imu_gyr;

  Sim();

  /** Return number of measurements */
  int getNumMeasurements() const;

  /** Extract subset of data */
  ImuBuffer formImuBuffer(const int start_index, const int end_index) const;

  /** Save simulation data */
  void save(const std::string &save_path) const;
};

} // namespace xyz
