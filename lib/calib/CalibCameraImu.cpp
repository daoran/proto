#include <optional>

#include "CalibCameraImu.hpp"
#include "SolvePnp.hpp"

namespace xyz {

CalibCameraImu::CalibCameraImu(const std::string &config_file)
    : CalibProblem{config_file} {}

std::pair<int, int> CalibCameraImu::findOptimalTarget() {
  int best_count = 0;
  int optimal_camera_id = 0;
  int optimal_target_id = 0;

  for (const auto &[camera_id, targets] : camera_buffers) {
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
}

int CalibCameraImu::estimatePose(Mat4 &T_WS) {
  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = findOptimalTarget();
  const auto target = camera_buffers.at(camera_id).at(target_id);
  target->getMeasurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return -1;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries[camera_id];
  const auto target_geometry = target_geometries[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return -2;
  }

  // Form T_WS
  const int imu_id = 0;
  const Mat4 T_WT0 = tf(target_pose);
  const Mat4 T_T0Tj = tf(target_geometry->extrinsic);
  const Mat4 T_TjCi = T_CiTj.inverse();
  const Mat4 T_CiC0 = tf(camera_geometry->extrinsic).inverse();
  const Mat4 T_C0S = tf(imu_geometries[imu_id]->extrinsic);
  T_WS = T_WT0 * T_T0Tj * T_TjCi * T_CiC0 * T_C0S;

  return 0;
}

int CalibCameraImu::estimateRelativePose(Mat4 &T_C0T0) {
  // // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = findOptimalTarget();
  const auto target = camera_buffers.at(camera_id).at(target_id);
  target->getMeasurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return -1;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries[camera_id];
  const auto target_geometry = target_geometries[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return -2;
  }

  // Form relative pose T_C0T0
  const Mat4 T_TjT0 = tf(target_geometry->extrinsic).inverse();
  const Mat4 T_C0Ci = tf(camera_geometry->extrinsic);
  T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

  return 0;
}

void CalibCameraImu::addView(const timestamp_t ts, const Mat4 &T_WS) {
  // Lambda function - Get last two keys in std::map
  auto last_two_keys = [](const auto &m)
      -> std::optional<
          std::pair<typename std::decay_t<decltype(m)>::key_type,
                    typename std::decay_t<decltype(m)>::key_type>> {
    if (m.size() < 2) {
      return std::nullopt;
    }

    auto it = m.rbegin();
    auto last = it->first;
    ++it;
    auto second_last = it->first;

    return std::make_pair(second_last, last);
  };

  // Add pose
  addPose(ts, T_WS);

  // Add speed and biases
  if (poses.size() == 1) {
    const Vec3 v_WS_k{0.0, 0.0, 0.0};
    addSpeedAndBiases(ts, v_WS_k);

  } else {
    // Infer velocity from two poses T_WS_k and T_WS_km1
    const auto [ts_km1, ts_k] = *last_two_keys(poses);
    const double dt = ((ts_k - ts_km1) * 1e-9);
    double *pose_km1 = getPosePtr(ts_km1);
    double *pose_k = getPosePtr(ts_k);
    const Mat4 T_WS_km1 = tf(pose_km1);
    const Mat4 T_WS_k = tf(pose_k);
    const Vec3 r_WS_km1 = tf_trans(T_WS_km1);
    const Vec3 r_WS_k = tf_trans(T_WS_k);
    const Vec3 v_WS_k = (r_WS_k - r_WS_km1) / dt;
    addSpeedAndBiases(ts_k, v_WS_k);
  }

  // Add camera measurement if it does not exist
  for (const auto &[camera_id, targets] : camera_buffers) {
    for (const auto &[target_id, target] : targets) {
      if (hasCameraMeasurement(ts, camera_id, target_id) == false) {
        addCameraMeasurement(ts, camera_id, target);
      }
    }
  }

  // Add camera residual blocks
  double *pose_ptr = getPosePtr(ts);
  double *target_pose_ptr = getTargetPosePtr();
  auto imu_geometry = getImuGeometry(0);

  for (const auto &[camera_id, targets] : camera_buffers) {
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
        auto resblock = CalibCameraImuError::create(camera_geometry,
                                                    imu_geometry,
                                                    target_geometry,
                                                    pose_ptr,
                                                    target_pose_ptr,
                                                    point_ids[i],
                                                    keypoints[i]);
        camera_resblocks[ts][camera_id].push_back(resblock);
        addResidualBlock(resblock.get());
      }
    }
  }

  // Add imu residual block
  if (poses.size() >= 2) {
    // Add IMU factor
    const auto [ts_km1, ts_k] = *last_two_keys(poses);
    const int imu_id = 0;
    const auto &imu_params = getImuGeometry(imu_id)->imu_params;
    double *pose_km1 = getPosePtr(ts_km1);
    double *pose_k = getPosePtr(ts_k);
    double *sb_km1 = getSpeedAndBiasesPtr(ts_km1);
    double *sb_k = getSpeedAndBiasesPtr(ts_k);
    auto imu_data = imu_buffer.extract(ts_km1, ts_k);
    imu_buffer.trim(ts_k);

    auto resblock =
        ImuError::create(imu_params, imu_data, pose_km1, sb_km1, pose_k, sb_k);
    imu_resblocks[ts_km1][imu_id] = resblock;
    addResidualBlock(resblock.get());
  }

  // Clean up
  camera_buffers.clear();
}

void CalibCameraImu::initialize(const timestamp_t ts) {
  assert(initialized_ == false);

  // Estimate relative pose - T_C0T0
  Mat4 T_C0T0;
  if (estimateRelativePose(T_C0T0) != 0) {
    return;
  }

  // Estimate initial IMU attitude
  const Mat3 C_WS = imu_buffer.estimateAttitude();

  // Sensor pose - T_WS
  Mat4 T_WS = tf(C_WS, zeros(3, 1));

  // Target pose - T_WT0
  Mat4 T_SC0 = tf(imu_geometries[0]->extrinsic).inverse();
  Mat4 T_WT0 = T_WS * T_SC0 * T_C0T0;

  // // Set:
  // // 1. Target as origin (with r_WT0 (0, 0, 0))
  // // 2. Calculate the sensor pose offset relative to target origin
  // const Vec3 offset = -1.0 * tf_trans(T_WT0);
  // const Mat3 C_WT0 = tf_rot(T_WT0);
  // const Vec3 r_WT0{0.0, 0.0, 0.0};
  // T_WT0 = tf(C_WT0, r_WT0);
  // T_WS = tf(C_WS, offset);

  // Add target pose
  setTargetPose(T_WT0);

  // Print to screen
  if (verbose) {
    printf("Initial estimates\n");
    printTargetGeometries(stdout);
    printCameraGeometries(stdout);
    printImuGeometries(stdout);
  }

  // Add camera residuals
  addView(ts, T_WS);

  // Update
  initialized = true;
}

void CalibCameraImu::addMeasurement(const timestamp_t ts,
                                    const Vec3 &imu_acc,
                                    const Vec3 &imu_gyr) {
  // Add Imu measurement
  imu_buffer.add(ts, imu_acc, imu_gyr);
  imu_started = true;

  // Check if calibration target buffer is filled
  if (static_cast<int>(camera_buffers.size()) != getNumCameras()) {
    return;
  }

  // Add view
  if (initialized == false) {
    initialize(ts);

  } else {
    Mat4 T_WS;
    if (estimatePose(T_WS) != 0) {
      return;
    }
    addView(ts, T_WS);
  }
}

void CalibCameraImu::addMeasurement(const timestamp_t ts,
                                    const int camera_id,
                                    const CalibTargetPtr &calib_target) {
  // Pre-check
  if (ts != calib_target->getTimestamp()) {
    FATAL("ts != calib_target->getTimestamp()");
  }

  // Do not add vision data before first imu measurement
  if (imu_started == false) {
    return;
  }

  // Add to camera buffer
  const auto target_id = calib_target->getTargetId();
  camera_buffers[camera_id][target_id] = calib_target;
  camera_started = true;

  return;
}

void CalibCameraImu::solve() {
  // Pre-check
  if (camera_data.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries.size() == 0) {
    FATAL("No cameras added?");
  }
  if (imu_geometries.size() == 0) {
    FATAL("No imus added?");
  }
  if (target_configs.size() == 0 || target_geometries.size() == 0) {
    FATAL("No targets added?");
  }

  for (auto &[camera_id, camera] : camera_geometries) {
    problem->SetParameterBlockConstant(camera->intrinsic.data());
    problem->SetParameterBlockConstant(camera->extrinsic.data());
  }
  for (auto &[target_id, target] : target_geometries) {
    problem->SetParameterBlockConstant(target->extrinsic.data());
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 50;
  options.num_threads = 1;
  options.min_trust_region_radius = 1e-50;  // Default: 1e-32
  options.function_tolerance = 1e-20;       // Default: 1e-6
  options.gradient_tolerance = 1e-20;       // Default: 1e-10
  options.parameter_tolerance = 1e-20;      // Default: 1e-8

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl << std::endl;
  printSummary(stdout);
}

} // namespace xyz
