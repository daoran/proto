#pragma once

#include "Core.hpp"
#include "ParamBlock.hpp"

namespace xyz {

/**
 * Solver base-class
 */
class Solver {
private:
  // Settings
  bool verbose_ = false;
  int max_iter_ = 30;

  real_t initial_cost = 0.0;
  real_t final_cost = 0.0;
  real_t num_iterations = 0.0;

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

/**
 * Ceres Solver
 */
class CeresSolver : public Solver {
private:
  ceres::Problem::Options prob_options_;
  ceres::Solver::Options solver_options;
  ceres::Solver::Summary solver_summary_;
  std::unique_ptr<ceres::Problem> problem_;
  PoseLocalParameterization pose_plus_;

public:
  CeresSolver() {
    // Problem options
    prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options_.enable_fast_removal = true;
    problem_ = std::make_unique<ceres::Problem>(prob_options_);

    // Solver options
    solver_options_.minimizer_progress_to_stdout = true;
    solver_options_.max_num_iterations = 30;
    solver_options_.num_threads = 1;
    solver_options_.initial_trust_region_radius = 10; // Default: 1e4
    solver_options_.min_trust_region_radius = 1e-50;  // Default: 1e-32
    solver_options_.function_tolerance = 1e-20;       // Default: 1e-6
    solver_options_.gradient_tolerance = 1e-20;       // Default: 1e-10
    solver_options_.parameter_tolerance = 1e-20;      // Default: 1e-8
  }

  void solve() {
    ceres::Solve(solver_options_, problem_.get(), &solve_summary_);
    std::cout << summary.FullReport() << std::endl << std::endl;
  }
};

} // namespace xyz
