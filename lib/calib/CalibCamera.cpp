#include "CalibCamera.hpp"
#include "CameraChain.hpp"
#include "CalibTargetChain.hpp"

#include "../timeline/Timeline.hpp"

namespace xyz {

CalibCamera::CalibCamera(const std::string &config_file)
    : CalibData{config_file} {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);
}

void CalibCamera::initializeIntrinsics() {
  // Find the target with the most observations for a particular camera
  auto find_optimal_target = [&](const CameraData &camera_data) {
    std::map<int, int> target_count; // target_id, count

    // Initialize map
    for (const auto &[target_id, _] : target_configs_) {
      target_count[target_id] = 0;
    }

    // Count
    for (const auto &[ts, targets] : camera_data) {
      for (const auto &[target_id, target] : targets) {
        target_count[target_id] += target->getNumDetected();
      }
    }

    // Find best count
    int best_target_count = 0;
    int best_target_id = -1;
    for (const auto &[target_id, count] : target_count) {
      if (count > best_target_count) {
        best_target_count = count;
        best_target_id = target_id;
      }
    }

    return best_target_id;
  };

  // Solve intrinsics of a single camera
  auto solve_intrinsics = [&](const int camera_id,
                              const CameraData &camera_data) {
    // Setup Problem
    std::map<timestamp_t, Vec7> relposes;
    std::vector<std::shared_ptr<CalibCameraError>> resblocks;
    auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

    // -- Add camera to problem
    auto camera_geometry = getCameraGeometry(camera_id);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus_);
    init_problem->SetParameterBlockConstant(extrinsic);

    // -- Add calibration target to problem
    const auto optimal_target_id = find_optimal_target(camera_data);
    const auto target_config = target_configs_[optimal_target_id];
    const auto points = target_config.getObjectPoints();
    const auto target_ext = tf_vec(I(4));
    auto target0 = std::make_shared<CalibTargetGeometry>(0, target_ext, points);
    init_problem->AddParameterBlock(target0->getExtrinsicPtr(), 7);
    init_problem->SetManifold(target0->getExtrinsicPtr(), &pose_plus_);
    init_problem->SetParameterBlockConstant(target0->getExtrinsicPtr());

    // -- Build problem
    for (const auto &[ts, targets] : camera_data) {
      // Check if detected
      const auto target = targets.at(optimal_target_id);
      if (target->detected() == false) {
        continue;
      }

      // Get calibration target measurements
      std::vector<int> point_ids;
      Vec2s keypoints;
      Vec3s object_points;
      target->getMeasurements(point_ids, keypoints, object_points);
      if (keypoints.size() < 10) {
        continue;
      }

      // Estimate relative pose T_CT
      Mat4 T_CT;
      SolvePnp pnp{camera_geometry};
      int status = pnp.estimate(keypoints, object_points, T_CT);
      if (status != 0) {
        continue;
      }

      // Add pose
      Mat2 covar = I(2);
      relposes[ts] = tf_vec(T_CT);
      init_problem->AddParameterBlock(relposes[ts].data(), 7);
      init_problem->SetManifold(relposes[ts].data(), &pose_plus_);

      // Add residual blocks
      for (size_t i = 0; i < point_ids.size(); i++) {
        const int point_id = point_ids[i];
        Vec3 &pt = target0->getPoint(point_id);
        init_problem->AddParameterBlock(pt.data(), 3);
        init_problem->SetParameterBlockConstant(pt.data());

        auto resblock = CalibCameraError::create(camera_geometry,
                                                 target0,
                                                 point_id,
                                                 relposes[ts].data(),
                                                 keypoints[i],
                                                 covar);
        resblocks.push_back(resblock);
        init_problem->AddResidualBlock(resblock.get(),
                                       nullptr,
                                       resblock->getParamPtrs());
      }
    }

    // -- Solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 30;
    options.num_threads = 1;
    options.initial_trust_region_radius = 10; // Default: 1e4
    options.min_trust_region_radius = 1e-50;  // Default: 1e-32
    options.function_tolerance = 1e-20;       // Default: 1e-6
    options.gradient_tolerance = 1e-20;       // Default: 1e-10
    options.parameter_tolerance = 1e-20;      // Default: 1e-8

    // -- Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, init_problem.get(), &summary);
    std::cout << summary.FullReport() << std::endl << std::endl;

    std::vector<double> reproj_errors;
    for (const auto &resblock : resblocks) {
      double error = 0.0;
      if (resblock->getReprojError(&error)) {
        reproj_errors.push_back(error);
      }
    }
    printf("mean reproj error: %f\n", mean(reproj_errors));
    printf("rmse reproj error: %f\n", rmse(reproj_errors));
    printf("median reproj error: %f\n", median(reproj_errors));
  };

  // Solve all camera intrinsics
  for (const auto &[camera_id, camera_data] : getAllCameraData()) {
    solve_intrinsics(camera_id, camera_data);
  }
}

void CalibCamera::initializeExtrinsics() {
  // Initialize camera extrinsics
  CameraChain camchain(getAllCameraGeometries(), getAllCameraData());
  for (auto &[camera_id, camera_geometry] : getAllCameraGeometries()) {
    Mat4 T_CiCj;
    if (camchain.find(0, camera_id, T_CiCj) != 0) {
      FATAL("No observations between camera0 and camera%d\n", camera_id);
    }
    camera_geometry->setExtrinsic(T_CiCj);
  }

  // Initialize calibration target extrinsics
  CalibTargetChain targetchain(getAllCameraGeometries(), getAllCameraData());
  for (auto &[target_id, target_geometry] : getAllTargetGeometries()) {
    Mat4 T_T0Tj;
    if (camchain.find(0, target_id, T_T0Tj) != 0) {
      FATAL("No observations between target0 and target%d\n", target_id);
    }
    target_geometry->setExtrinsic(T_T0Tj);
  }

  // Build problem
  Mat2 covar = I(2);
  std::map<timestamp_t, Vec7> relposes;
  std::map<int, Vec3> target_points;
  std::vector<std::shared_ptr<CalibCameraError>> resblocks;
  auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

  // -- Add calibration target geometries
  for (auto &[target_id, target_geometry] : getAllTargetGeometries()) {
    double *extrinsic = target_geometry->getExtrinsicPtr();
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus_);
    if (target_id == 0) {
      init_problem->SetParameterBlockConstant(extrinsic);
    }
  }

  // -- Add camera geometries
  for (const auto &[camera_id, camera_data] : getAllCameraData()) {
    auto camera_geometry = getCameraGeometry(camera_id);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus_);
    if (camera_id == 0) {
      init_problem->SetParameterBlockConstant(extrinsic);
    }
  }

  // -- Add camera measurements
  for (const auto &[camera_id, camera_data] : getAllCameraData()) {
    const auto camera_geometry = getCameraGeometry(camera_id);

    for (const auto &[ts, targets] : camera_data) {
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

        // Estimate relative pose T_C0T0
        Mat4 T_C0T0;
        if (relposes.count(ts) == 0) {
          // Solvepnp T_CiTj
          Mat4 T_CiTj;
          SolvePnp pnp{camera_geometry};
          int status = pnp.estimate(keypoints, object_points, T_CiTj);
          if (status != 0) {
            continue;
          }

          // Form T_C0T0
          const Mat4 T_C0Ci = camera_geometry->getTransform();
          const Mat4 T_TjT0 = target_geometry->getTransform().inverse();
          T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

          // Add pose
          relposes[ts] = tf_vec(T_C0T0);
          init_problem->AddParameterBlock(relposes[ts].data(), 7);
          init_problem->SetManifold(relposes[ts].data(), &pose_plus_);
        } else {
          T_C0T0 = tf(relposes[ts]);
        }

        // Add residual blocks
        for (size_t i = 0; i < point_ids.size(); i++) {
          const int point_id = point_ids[i];
          Vec3 &pt = target_geometry->getPoint(point_id);
          init_problem->AddParameterBlock(pt.data(), 3);
          init_problem->SetParameterBlockConstant(pt.data());

          auto resblock = CalibCameraError::create(camera_geometry,
                                                   target_geometry,
                                                   point_id,
                                                   relposes[ts].data(),
                                                   keypoints[i],
                                                   covar);
          resblocks.push_back(resblock);
          init_problem->AddResidualBlock(resblock.get(),
                                         nullptr,
                                         resblock->getParamPtrs());
        }
      }
    }
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
  ceres::Solve(options, init_problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl << std::endl;

  std::vector<double> reproj_errors;
  for (const auto &resblock : resblocks) {
    double error = 0.0;
    if (resblock->getReprojError(&error)) {
      reproj_errors.push_back(error);
    }
  }

  printf("mean reproj error: %f\n", mean(reproj_errors));
  printf("rmse reproj error: %f\n", rmse(reproj_errors));
  printf("median reproj error: %f\n", median(reproj_errors));
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
  const auto target_geometry = camera_geometries_[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return;
  }

  // Add camera measurement if it does not exist
  const timestamp_t ts = target->getTimestamp();
  for (const auto &[camera_id, targets] : measurements) {
    for (const auto &[target_id, target] : targets) {
      if (hasCameraMeasurement(ts, camera_id, target_id) == false) {
        addCameraMeasurement(ts, camera_id, target);
      }
    }
  }

  // Form T_C0T0
  const Mat4 T_C0Ci = camera_geometry->getTransform();
  const Mat4 T_TjT0 = target_geometry->getTransform().inverse();
  Mat4 T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

  // Add timestamp and pose
  timestamps_.insert(ts);
  poses_[ts] = tf_vec(T_C0T0);

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
  // Initialize intrinsics and extrinsics
  initializeIntrinsics();
  initializeExtrinsics();

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
        timeline.add(ts, camera_id, target_id, target);
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
