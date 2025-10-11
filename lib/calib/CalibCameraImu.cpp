#include "CalibCameraImu.hpp"
#include "SolvePnp.hpp"

namespace xyz {

CalibCameraImu::CalibCameraImu(const std::string &config_file)
    : CalibData{config_file} {
  prob_options_.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options_.enable_fast_removal = true;
  problem_ = std::make_shared<ceres::Problem>(prob_options_);
}

void CalibCameraImu::addView() {
  // Check if we have IMU measurements
  if (imu_buffer_.getNumMeasurements() == 0) {
    return;
  }

  // Check if calib target was detected and get the best detection
  int best_camera_index = 0;
  CalibTargetPtr best_target = nullptr;
  for (const auto &[camera_index, calib_target] : camera_buffer_) {
    // Check if calibration target is detected
    if (calib_target->detected() == false) {
      continue;
    }

    // Keep track of the best detected calibration target
    if (best_target == nullptr) {
      best_camera_index = camera_index;
      best_target = calib_target;
    } else if (calib_target->getNumDetected() > best_target->getNumDetected()) {
      best_camera_index = camera_index;
      best_target = calib_target;
    }
  }
  if (best_target == nullptr) {
    camera_buffer_.clear();
    return;
  }

  // Get calibration target measurements
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  best_target->getMeasurements(tag_ids,
                               corner_indicies,
                               keypoints,
                               object_points);
  if (keypoints.size() < 10) {
    camera_buffer_.clear();
    return;
  }

  // Estimate relative pose T_CiF
  Mat4 T_CiF;
  SolvePnp pnp{camera_geometries_[best_camera_index]};
  int status = pnp.estimate(keypoints, object_points, T_CiF);
  if (status != 0) {
    camera_buffer_.clear();
    return;
  }

  // Add to calib data if it does not exist
  const timestamp_t ts = best_target->getTimestamp();
  for (const auto &[camera_index, calib_target] : camera_buffer_) {
    if (hasCameraMeasurement(ts, camera_index) == false) {
      addCameraMeasurement(ts, camera_index, calib_target);
    }
  }

  // Add calibration view
  const Mat4 T_FCi = T_CiF.inverse();
  const Mat4 T_BCi = camera_geometries_[best_camera_index]->getTransform();
  const Mat4 T_BS = imu_geometries_[0]->getTransform();
  const Mat4 T_CiS = T_BCi.inverse() * T_BS;
  const Mat4 T_WF;
  const Mat4 T_WS = T_WF * T_FCi * T_CiS;
  const Vec7 pose = tf_vec(T_WS);
  timestamps_.insert(ts);
  // calib_views_[ts] = std::make_shared<CalibView>(problem_,
  //                                                ts,
  //                                                camera_buffer_,
  //                                                camera_geometries_,
  //                                                target_points_,
  //                                                pose);
  camera_buffer_.clear();
}

void CalibCameraImu::addMeasurement(const timestamp_t ts,
                                    const Vec3 &imu_acc,
                                    const Vec3 &imu_gyr) {
  imu_buffer_.add(ts, imu_acc, imu_gyr);
  imu_started_ = true;

  // Check if calibration target buffer is filled
  if (static_cast<int>(camera_buffer_.size()) != getNumCameras()) {
    return;
  }

  // Add view
  addView();
}

void CalibCameraImu::addMeasurement(const int camera_index,
                                    const CalibTargetPtr &calib_target) {
  // Do not add vision data before first imu measurement
  if (imu_started_ == false) {
    return;
  }

  // Add to camera buffer
  camera_buffer_[camera_index] = calib_target;
  camera_started_ = true;

  return;
}

void CalibCameraImu::solve() {
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
  // std::cout << summary.FullReport() << std::endl << std::endl;
  // printSummary(stdout);
}

} // namespace xyz
