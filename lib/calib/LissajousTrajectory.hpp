#pragma once

#include "../core/Core.hpp"

namespace cartesian {

/**
 * Lissajous Trajectory
 */
struct LissajousTrajectory {
  const std::string traj_type = "figure8";
  const timestamp_t ts_start;   // Start timestamp
  const Mat4 T_WT;              // Target pose
  const Mat4 T_TO;              // Calibration origin
  const double calib_width;     // Calibration width
  const double calib_height;    // Calibration height
  const double R;               // Distance in-front of calibration target
  const double T;               // Period - Time it takes to complete [secs]
  const Vec3 g{0.0, 0.0, 9.81}; // Gravity vector

  double A;     // Amplitude in x-axis
  double B;     // Amplitude in y-axis
  double a;     // Angular velocity in x-axis
  double b;     // Angular velocity in y-axis
  double delta; // Phase offset

  double f; // Frequency
  double w; // Angular velocity

  double yaw_scale;   // Yaw scale
  double pitch_scale; // Pitch scale

  double pitch_bound;
  double yaw_bound;

  LissajousTrajectory() = delete;
  LissajousTrajectory(const std::string &traj_type_,
                      const timestamp_t ts_start_,
                      const Mat4 &T_WT_,
                      const Mat4 &T_TO_,
                      const double calib_width_,
                      const double calib_height_,
                      const double R_,
                      const double T_);
  virtual ~LissajousTrajectory() = default;

  Quat get_q_OS(const timestamp_t ts_k) const;
  Quat get_q_OS_dot(const timestamp_t ts_k) const;

  Mat4 get_pose(const timestamp_t ts_k) const;
  Vec3 get_velocity(const timestamp_t ts_k) const;
  Vec3 get_acceleration(const timestamp_t ts_k) const;
  Vec3 get_angular_velocity(const timestamp_t ts_k) const;
  int save(const std::string &save_path) const;
};

} // namespace cartesian
