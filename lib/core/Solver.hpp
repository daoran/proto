#pragma once

#include <unordered_map>

#include "Core.hpp"

#include "imu/ImuGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "ceres/ParamBlock.hpp"
#include "ceres/ResidualBlock.hpp"

namespace cartesian {

/**
 * Solver base-class
 */
class Solver {
private:
  // Settings
  bool verbose_ = false;
  int max_iter_ = 30;

  double initial_cost = 0.0;
  double final_cost = 0.0;
  double num_iterations = 0.0;

  std::unorderd_map<int, std::shared_ptr<ResidualBlock>> residual_blocks_;
  std::unorderd_map<int, std::shared_ptr<CameraGeometry>> camera_geometries_;
  std::unorderd_map<int, std::shared_ptr<ImuGeometry>> imu_geometries_;

public:
  Solver() = default;
  virtual ~Solver() = default;

  /** Set verbose */
  bool setVerbose(const bool verbose);

  /** Set max iterations */
  bool setMaxIter(const int max_iter);

  /** Return number of residual blocks */
  size_t getNumResidualBlocks() const;

  /** Add camera geometry */
  int addCameraGeometry(std::shared_ptr<CameraGeometry> &camera_geometry);

  /** Add imu geometry */
  int addImuGeometry(std::shared_ptr<CameraGeometry> &imu_geometry);

  /** Solve */
  virtual void solve() = 0;
};

} // namespace cartesian
