#include "CalibCamera.hpp"

#include "../timeline/Timeline.hpp"

namespace xyz {

CalibCamera::CalibCamera() {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);
}

CalibCamera::CalibCamera(const std::string &config_file)
    : CalibData{config_file} {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);
}

void CalibCamera::addView(const std::map<int, CalibTargetMap> &measurements) {
  // Lambda function - Find the target with the most observations
  auto find_optimal_target =
      [&](const std::map<int, CalibTargetMap> &measurements) {
        int best_count = 0;
        int optimal_camera_id = 0;
        int optimal_target_id = 0;

        for (const auto &[camera_id, targets] : measurements) {
          // Find the most observed calibration target by camera
          int camera_target_count = 0;
          int camera_target_id = -1;
          for (const auto &[target_id, target] : targets) {
            if (target->getNumDetected() > camera_target_count) {
              camera_target_count = target->getNumDetected();
              camera_target_id = target_id;
            }
          }

          // Track the most observed calibration target by all cameras
          if (camera_target_count > best_count) {
            best_count = camera_target_count;
            optimal_camera_id = camera_id;
            optimal_target_id = camera_target_id;
          }
        }

        return std::pair<int, int>{optimal_camera_id, optimal_target_id};
      };

  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = find_optimal_target(measurements);
  const auto target = measurements.at(camera_id).at(target_id);
  target->getMeasurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries_[camera_id];
  const auto target_geometry = target_geometries_[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return;
  }

  // Form T_C0T0
  const Mat4 T_C0Ci = camera_geometry->getTransform();
  const Mat4 T_TjT0 = target_geometry->getTransform().inverse();
  Mat4 T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

  // Add camera measurement if it does not exist
  const timestamp_t ts = target->getTimestamp();
  for (const auto &[camera_id, targets] : measurements) {
    for (const auto &[target_id, target] : targets) {
      if (hasCameraMeasurement(ts, camera_id, target_id) == false) {
        addCameraMeasurement(ts, camera_id, target);
      }
    }
  }

  // Add timestamp and pose
  addPose(ts, T_C0T0);

  // Add residual blocks
  for (const auto &[camera_id, targets] : measurements) {
    const auto camera_geometry = getCameraGeometry(camera_id);

    for (const auto &[target_id, target] : targets) {
      // Check if detected
      const auto target_geometry = getTargetGeometry(target_id);
      if (target->detected() == false) {
        continue;
      }

      // Get calibration target measurements
      std::vector<int> point_ids;
      std::vector<int> corner_indicies;
      Vec2s keypoints;
      Vec3s object_points;
      target->getMeasurements(point_ids, keypoints, object_points);
      if (keypoints.size() < 10) {
        continue;
      }

      // Form and add residual blocks
      for (size_t i = 0; i < point_ids.size(); i++) {
        const int point_id = point_ids[i];
        Vec3 &pt = target_geometry->getPoint(point_id);
        problem_->AddParameterBlock(pt.data(), 3);
        problem_->SetParameterBlockConstant(pt.data());

        auto resblock = CalibCameraError::create(camera_geometry,
                                                 target_geometry,
                                                 point_id,
                                                 poses_[ts].data(),
                                                 keypoints[i]);
        resblocks_[ts][camera_id].push_back(resblock);
        problem_->AddResidualBlock(resblock.get(),
                                   nullptr,
                                   resblock->getParamPtrs());
      }
    }
  }
}

void CalibCamera::solve() {
  // Pre-check
  if (camera_data_.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries_.size() == 0) {
    FATAL("No cameras added?");
  }
  if (target_configs_.size() == 0 || target_geometries_.size() == 0) {
    FATAL("No targets added?");
  }

  // Initialize intrinsics and extrinsics
  initializeCameraIntrinsics();
  initializeCameraExtrinsics();

  // Add camera intrinsic and extrinsics to problem
  for (auto &[camera_id, camera_geometry] : camera_geometries_) {
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    problem_->AddParameterBlock(intrinsic, intrinsic_size);
    problem_->AddParameterBlock(extrinsic, 7);
    problem_->SetManifold(extrinsic, &pose_plus_);

    if (camera_id == 0) {
      problem_->SetParameterBlockConstant(extrinsic);
    }
  }

  // Form timeline
  Timeline timeline;
  for (const auto &[camera_id, measurements] : camera_data_) {
    for (const auto &[ts, targets] : measurements) {
      for (const auto &[target_id, target] : targets) {
        timeline.add(ts, camera_id, target);
      }
    }
  }

  // Loop through timeline
  for (const auto ts : timeline.timestamps) {
    std::map<int, CalibTargetMap> viewset;
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto camera_id = target_event->camera_id;
        const auto target_id = target_event->target_id;
        const auto calib_target = target_event->calib_target;
        viewset[camera_id][target_id] = calib_target;
      }
    }
    addView(viewset);
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 30;
  options.num_threads = 1;
  options.initial_trust_region_radius = 10; // Default: 1e4
  options.min_trust_region_radius = 1e-50;  // Default: 1e-32
  options.function_tolerance = 1e-20;       // Default: 1e-6
  options.gradient_tolerance = 1e-20;       // Default: 1e-10
  options.parameter_tolerance = 1e-20;      // Default: 1e-8

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);
  std::cout << summary.FullReport() << std::endl << std::endl;
  printSummary(stdout);
}

} // namespace xyz
