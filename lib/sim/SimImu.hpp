#pragma once
#include "../core/Core.hpp"
#include "../imu/imu.hpp"

namespace cartesian {

struct SimImu {
  // Data
  std::vector<timestamp_t> timestamps;
  std::map<timestamp_t, Mat4> poses;
  std::map<timestamp_t, Vec3> vel;
  std::map<timestamp_t, Vec3> imu_acc;
  std::map<timestamp_t, Vec3> imu_gyr;

  SimImu();
  virtual ~SimImu() = default;

  /** Return number of measurements */
  int get_num_measurements() const;

  /** Extract subset of data */
  ImuBuffer form_imu_buffer(const int start_index, const int end_index) const;

  /** Save simulation data */
  void save(const std::string &save_path) const;
};

} // namespace cartesian
