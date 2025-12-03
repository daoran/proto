#include "CalibCamera.hpp"
#include "CameraChain.hpp"

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
  auto find_optimal_target_id = [&](const CameraData &camera_data) {
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
    std::map<int, Vec3> target_points;
    std::vector<std::shared_ptr<CalibCameraError>> resblocks;
    auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

    // Add camera to problem
    auto camera_geometry = getCameraGeometry(camera_id);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus_);
    init_problem->SetParameterBlockConstant(extrinsic);

    // Build problem
    const auto optimal_target_id = find_optimal_target_id(camera_data);
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
        target_points[point_ids[i]] = object_points[i];
        Vec3 &pt = target_points[point_ids[i]];
        init_problem->AddParameterBlock(pt.data(), 3);
        init_problem->SetParameterBlockConstant(pt.data());

        auto resblock = CalibCameraError::create(camera_geometry,
                                                 relposes[ts].data(),
                                                 pt.data(),
                                                 keypoints[i],
                                                 covar);
        resblocks.push_back(resblock);
        init_problem->AddResidualBlock(resblock.get(),
                                       nullptr,
                                       resblock->getParamPtrs());
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
  };

  // Solve camera intrinsics
  for (const auto &[camera_id, camera_data] : getAllCameraData()) {
    solve_intrinsics(camera_id, camera_data);
  }
}

void CalibCamera::initializeExtrinsics() {
  // Setup camera chain
  CameraChain camchain(getAllCameraGeometries(), getAllCameraData());
  for (auto &[camera_id, camera_geometry] : getAllCameraGeometries()) {
    Mat4 T_CiCj;
    if (camchain.find(0, camera_id, T_CiCj) != 0) {
      FATAL("No observations between camera0 and camera%d\n", camera_id);
    }
    camera_geometry->setExtrinsic(T_CiCj);
  }

  // Setup Problem
  Mat2 covar = I(2);
  std::map<timestamp_t, Vec7> relposes;
  std::map<int, Vec3> target_points;
  std::vector<std::shared_ptr<CalibCameraError>> resblocks;
  auto init_problem = std::make_unique<ceres::Problem>(prob_options_);

  // Build problem
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

    // Add camera measurements
    for (const auto &[ts, targets] : camera_data) {
      for (const auto &[target_id, target] : targets) {
        // Check if detected
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

        // Estimate relative pose T_C0T
        // Mat4 T_C0T;
        // if (relposes.count(ts) == 0) {
        //   // Solvepnp T_CiF
        //   Mat4 T_CiF;
        //   SolvePnp pnp{camera_geometry};
        //   int status = pnp.estimate(keypoints, object_points, T_CiF);
        //   if (status != 0) {
        //     continue;
        //   }
        //
        //   // Form T_C0T
        //   const Mat4 T_C0Ci = camera_geometry->getTransform();
        //   T_C0T = T_C0Ci * T_CiF;
        //
        //   // Add pose
        //   relposes[ts] = tf_vec(T_C0T);
        //   init_problem->AddParameterBlock(relposes[ts].data(), 7);
        //   init_problem->SetManifold(relposes[ts].data(), &pose_plus_);
        // } else {
        //   T_C0T = tf(relposes[ts]);
        // }

        // // Add residual blocks
        // for (size_t i = 0; i < point_ids.size(); i++) {
        //   target_points[point_ids[i]] = object_points[i];
        //   Vec3 &pt = target_points[point_ids[i]];
        //   init_problem->AddParameterBlock(pt.data(), 3);
        //   init_problem->SetParameterBlockConstant(pt.data());
        //
        //   auto resblock = CalibCameraError::create(camera_geometry,
        //                                            relposes[ts].data(),
        //                                            pt.data(),
        //                                            keypoints[i],
        //                                            covar);
        //   resblocks.push_back(resblock);
        //   init_problem->AddResidualBlock(resblock.get(),
        //                                  nullptr,
        //                                  resblock->getParamPtrs());
        // }
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

void CalibCamera::addView(const std::map<int, CalibTargetPtr> &measurements) {
  // Check if AprilGrid was detected and get the best detected target
  int best_camera_id = 0;
  CalibTargetPtr best_target = nullptr;
  for (const auto &[camera_id, calib_target] : measurements) {
    // Check if calibration target is detected
    if (calib_target->detected() == false) {
      continue;
    }

    // Keep track of the best detected calibration target
    if (best_target == nullptr) {
      best_camera_id = camera_id;
      best_target = calib_target;
    } else if (calib_target->getNumDetected() > best_target->getNumDetected()) {
      best_camera_id = camera_id;
      best_target = calib_target;
    }
  }
  if (best_target == nullptr) {
    return;
  }

  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  best_target->getMeasurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return;
  }

  // Estimate relative pose T_CF
  Mat4 T_CiF;
  SolvePnp pnp{camera_geometries_[best_camera_id]};
  int status = pnp.estimate(keypoints, object_points, T_CiF);
  if (status != 0) {
    return;
  }

  // Add to calib data if it does not exist
  const timestamp_t ts = best_target->getTimestamp();
  for (const auto &[camera_id, calib_target] : measurements) {
    if (hasCameraMeasurement(ts, camera_id) == false) {
      addCameraMeasurement(ts, camera_id, calib_target);
    }
  }

  // Add calibration view
  // const Mat4 T_C0Ci = camera_geometries_[best_camera_id]->getTransform();
  // const Mat4 T_C0F = T_C0Ci * T_CiF;
  // const Vec7 pose = tf_vec(T_C0F);
  // timestamps_.insert(ts);
  // calib_views_[ts] = std::make_shared<CalibView>(problem_,
  //                                                ts,
  //                                                measurements,
  //                                                camera_geometries_,
  //                                                target_points_,
  //                                                pose);
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
  // Timeline timeline;
  // for (const auto &[camera_id, measurements] : camera_data_) {
  //   for (const auto &[ts, calib_target] : measurements) {
  //     timeline.add(ts, camera_id, calib_target);
  //   }
  // }

  // Loop through timeline
  // for (const auto ts : timeline.timestamps) {
  //   std::map<int, CalibTargetPtr> viewset;
  //   for (const auto &event : timeline.getEvents(ts)) {
  //     if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
  //       auto camera_id = target_event->camera_id;
  //       auto calib_target = target_event->calib_target;
  //       viewset[camera_id] = calib_target;
  //     }
  //   }
  //   addView(viewset);
  // }

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
