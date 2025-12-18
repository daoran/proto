#include "CalibData.hpp"
#include "CameraChain.hpp"
#include "CalibTargetChain.hpp"

namespace xyz {

CalibData::CalibData(const std::string &config_path)
    : config_path_{config_path} {
  // Parse config
  config_t config{config_path_};

  // -- Parse data settings
  parse(config, "data_path", data_path_);

  // -- Parse calibration target settings
  for (int target_id = 0; target_id < 100; target_id++) {
    // Check if key exists
    const std::string prefix = "target" + std::to_string(target_id);
    if (yaml_has_key(config, prefix) == 0) {
      continue;
    }

    // Load calibration target config
    std::string target_type;
    AprilGridConfig target_config;
    parse(config, prefix + ".target_type", target_type);
    parse(config, prefix + ".tag_rows", target_config.tag_rows);
    parse(config, prefix + ".tag_cols", target_config.tag_cols);
    parse(config, prefix + ".tag_size", target_config.tag_size);
    parse(config, prefix + ".tag_spacing", target_config.tag_spacing);
    parse(config, prefix + ".tag_id_offset", target_config.tag_id_offset, true);
    if (target_type != "aprilgrid") {
      FATAL("CalibData currently only supports target type [aprilgrid]!");
    }

    // Add calibration target
    const Vec7 target_extrinsic = tf_vec();
    addTarget(target_config, target_extrinsic);
  }

  // -- Parse camera settings
  for (int camera_id = 0; camera_id < 100; camera_id++) {
    // Check if key exists
    const std::string prefix = "cam" + std::to_string(camera_id);
    if (yaml_has_key(config, prefix) == 0) {
      continue;
    }

    // Load camera data
    loadCameraData(camera_id);

    // Parse
    Vec2i resolution;
    std::string camera_model;
    parse(config, prefix + ".resolution", resolution);
    parse(config, prefix + ".camera_model", camera_model);

    VecX intrinsic;
    if (yaml_has_key(config, prefix + ".intrinsic")) {
      parse(config, prefix + ".intrinsic", intrinsic);
    }

    Vec7 extrinsic;
    if (yaml_has_key(config, prefix + ".extrinsic")) {
      parse(config, prefix + ".extrinsic", extrinsic);
    } else {
      extrinsic << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }

    // Add camera
    addCamera(camera_id, camera_model, resolution, intrinsic, extrinsic);
  }

  // -- Parse IMU settings
  {
    const int imu_id = 0;
    const std::string prefix = "imu" + std::to_string(imu_id);
    if (yaml_has_key(config, prefix) == 0) {
      return;
    }

    ImuParams imu_params;
    parse(config, prefix + ".noise_acc", imu_params.noise_acc);
    parse(config, prefix + ".noise_gyr", imu_params.noise_gyr);
    parse(config, prefix + ".random_walk_acc", imu_params.noise_ba);
    parse(config, prefix + ".random_walk_gyr", imu_params.noise_bg);

    Vec7 extrinsic;
    extrinsic << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    addImu(imu_id, imu_params, extrinsic);

    // Load IMU data
    loadImuData(imu_id);
  }
}

/*******************************************************************************
 * Load methods
 ******************************************************************************/

void CalibData::loadCameraData(const int camera_id) {
  // Setup detector
  AprilGridDetector detector{target_configs_};

  // Form target dirs
  bool cache_exists = false;
  const std::string camera_string = "cam" + std::to_string(camera_id);
  const fs::path &camera_dir = data_path_ / camera_string / "data";

  for (size_t target_id = 0; target_id < target_configs_.size(); ++target_id) {
    // Check if target dir exists
    const std::string target_prefix = "target" + std::to_string(target_id);
    const fs::path &target_dir = data_path_ / target_prefix / camera_string;
    if (fs::exists(target_dir)) {
      cache_exists = true;
      continue;
    }

    // Create target dir
    if (system(("mkdir -p " + target_dir.string()).c_str()) != 0) {
      FATAL("Failed to create dir [%s]", target_dir.c_str());
    }
  }

  // Get image files
  std::vector<std::string> image_paths;
  list_files(camera_dir, image_paths);

  // Detect aprilgrids
  if (cache_exists == false) {
    const std::string desc = "Processing " + camera_string + " data: ";
    for (size_t k = 0; k < image_paths.size(); k++) {
      print_progress((double) k / (double) image_paths.size(), desc);

      const std::string image_fname = image_paths[k];
      const fs::path image_path = camera_dir / image_fname;
      const std::string ts_str = image_fname.substr(0, 19);
      const timestamp_t ts = std::stoull(ts_str);
      if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
        FATAL("Invalid image file: [%s]!", image_path.c_str());
      }

      const auto &image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      for (const auto &target : detector.detect(ts, camera_id, image)) {
        const int target_id = target->getTargetId();
        const fs::path target_prefix = "target" + std::to_string(target_id);
        const fs::path target_dir = data_path_ / target_prefix / camera_string;
        const fs::path target_csv = target_dir / (ts_str + ".csv");
        target->save(target_csv);
      }
    }
    print_progress(1.0);
  }

  // Load aprilgrids
  const std::string desc = "Loading " + camera_string + " data: ";
  for (size_t k = 0; k < image_paths.size(); k++) {
    print_progress((double) k / (double) image_paths.size(), desc);

    const std::string image_fname = image_paths[k];
    const fs::path image_path = camera_dir / image_fname;
    const std::string ts_str = image_fname.substr(0, 19);
    const timestamp_t ts = std::stoull(ts_str);
    const fs::path target_fname = ts_str + ".csv";
    if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
      FATAL("Invalid image file: [%s]!", image_path.c_str());
    }

    for (const auto &[target_id, target_config] : target_configs_) {
      const fs::path target_prefix = "target" + std::to_string(target_id);
      const fs::path target_dir = data_path_ / target_prefix / camera_string;
      const fs::path target_csv = target_dir / (ts_str + ".csv");

      // Load apriltag
      std::shared_ptr<AprilGrid> calib_target = AprilGrid::load(target_csv);

      // Add camera measurement
      addCameraMeasurement(ts, camera_id, calib_target);
    }
  }
  print_progress(1.0);
}

void CalibData::loadImuData(const int imu_id) {
  // Setup
  const std::string imu_string = "imu" + std::to_string(imu_id);
  const fs::path imu_path = data_path_ / imu_string / "data.csv";

  // Open IMU data.csv
  FILE *imu_csv = fopen(imu_path.c_str(), "r");
  if (imu_csv == nullptr) {
    FATAL("IMU data not found [%s]!", imu_path.c_str());
  }
  skip_line(imu_csv); // Skip header

  // Parse csv file
  const std::string desc = "Loading " + imu_string + " data: ";
  const char *scan_format = "%ld,%lf,%lf,%lf,%lf,%lf,%lf";
  const int num_rows = file_rows(imu_path) - 1;

  ImuBuffer imu_buffer;
  for (int i = 0; i < num_rows; i++) {
    print_progress((double) i / (double) num_rows, desc);
    // Parse line
    timestamp_t ts;
    Vec3 imu_acc;
    Vec3 imu_gyr;
    const int retval = fscanf(imu_csv,
                              scan_format,
                              &ts,
                              &imu_gyr.data()[0],
                              &imu_gyr.data()[1],
                              &imu_gyr.data()[2],
                              &imu_acc.data()[0],
                              &imu_acc.data()[1],
                              &imu_acc.data()[2]);
    if (retval != 7) {
      FATAL("Failed to parse data line %d in [%s]\n", i, imu_path.c_str());
    }

    // Add to imu buffer
    imu_buffer.add(ts, imu_acc, imu_gyr);
  }
  print_progress(1.0);

  // Add imu data
  imu_data_[imu_id] = imu_buffer;
}

/*******************************************************************************
 * Camera methods
 ******************************************************************************/

void CalibData::addCamera(const int camera_id,
                          const std::string &camera_model,
                          const Vec2i &resolution,
                          const VecX &intrinsic,
                          const Vec7 &extrinsic) {
  camera_geometries_[camera_id] = std::make_shared<CameraGeometry>(camera_id,
                                                                   camera_model,
                                                                   resolution,
                                                                   intrinsic,
                                                                   extrinsic);
}

void CalibData::addCameraMeasurement(const timestamp_t ts,
                                     const int camera_id,
                                     const CalibTargetPtr &calib_target) {
  const int target_id = calib_target->getTargetId();
  camera_data_[camera_id][ts][target_id] = calib_target;
}

bool CalibData::hasCameraMeasurement(const timestamp_t ts,
                                     const int camera_id,
                                     const int target_id) const {
  if (camera_data_.count(camera_id) == 0) {
    return false;
  } else if (camera_data_.at(camera_id).count(ts) == 0) {
    return false;
  } else if (camera_data_.at(camera_id).at(ts).count(target_id) == 0) {
    return false;
  }
  return true;
}

int CalibData::getNumCameras() const { return camera_geometries_.size(); }

std::map<int, CameraData> &CalibData::getAllCameraData() {
  return camera_data_;
}

CameraData &CalibData::getCameraData(const int camera_id) {
  return camera_data_.at(camera_id);
}

std::map<int, CameraGeometryPtr> &CalibData::getAllCameraGeometries() {
  return camera_geometries_;
}

CameraGeometryPtr &CalibData::getCameraGeometry(const int camera_id) {
  return camera_geometries_.at(camera_id);
}

void CalibData::initializeCameraIntrinsics() {
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
    PoseManifold pose_plus;
    ceres::Problem::Options prob_options;
    prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.enable_fast_removal = true;
    auto problem = std::make_unique<ceres::Problem>(prob_options);

    // -- Add camera to problem
    auto camera_geometry = getCameraGeometry(camera_id);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    problem->AddParameterBlock(intrinsic, intrinsic_size);
    problem->AddParameterBlock(extrinsic, 7);
    problem->SetManifold(extrinsic, &pose_plus);
    problem->SetParameterBlockConstant(extrinsic);

    // -- Add calibration target to problem
    const auto optimal_target_id = find_optimal_target(camera_data);
    const auto target_config = target_configs_[optimal_target_id];
    const auto points = target_config.getObjectPoints();
    const auto target_ext = tf_vec(I(4));
    auto target0 = std::make_shared<CalibTargetGeometry>(0, target_ext, points);
    problem->AddParameterBlock(target0->getExtrinsicPtr(), 7);
    problem->SetManifold(target0->getExtrinsicPtr(), &pose_plus);
    problem->SetParameterBlockConstant(target0->getExtrinsicPtr());

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
      problem->AddParameterBlock(relposes[ts].data(), 7);
      problem->SetManifold(relposes[ts].data(), &pose_plus);

      // Add residual blocks
      for (size_t i = 0; i < point_ids.size(); i++) {
        const int point_id = point_ids[i];
        Vec3 &pt = target0->getPoint(point_id);
        problem->AddParameterBlock(pt.data(), 3);
        problem->SetParameterBlockConstant(pt.data());

        auto resblock = CalibCameraError::create(camera_geometry,
                                                 target0,
                                                 point_id,
                                                 relposes[ts].data(),
                                                 keypoints[i],
                                                 covar);
        resblocks.push_back(resblock);
        problem->AddResidualBlock(resblock.get(),
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
    ceres::Solve(options, problem.get(), &summary);
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
    printf("camera_id: %d\n", camera_id);
    solve_intrinsics(camera_id, camera_data);
  }
}

void CalibData::initializeCameraExtrinsics() {
  // Pre-check
  if (getNumCameras() < 2) {
    return;
  }
  if (camera_data_.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries_.size() == 0) {
    FATAL("No cameras added?");
  }
  if (target_configs_.size() == 0 || target_geometries_.size() == 0) {
    FATAL("No targets added?");
  }

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
  PoseManifold pose_plus;
  ceres::Problem::Options prob_options;
  prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  auto problem = std::make_unique<ceres::Problem>(prob_options);

  // -- Add calibration target geometries
  for (auto &[target_id, target_geometry] : getAllTargetGeometries()) {
    double *extrinsic = target_geometry->getExtrinsicPtr();
    problem->AddParameterBlock(extrinsic, 7);
    problem->SetManifold(extrinsic, &pose_plus);
    if (target_id == 0) {
      problem->SetParameterBlockConstant(extrinsic);
    }
  }

  // -- Add camera geometries
  for (const auto &[camera_id, camera_data] : getAllCameraData()) {
    auto camera_geometry = getCameraGeometry(camera_id);
    const int intrinsic_size = camera_geometry->getIntrinsic().size();
    double *intrinsic = camera_geometry->getIntrinsicPtr();
    double *extrinsic = camera_geometry->getExtrinsicPtr();
    problem->AddParameterBlock(intrinsic, intrinsic_size);
    problem->AddParameterBlock(extrinsic, 7);
    problem->SetManifold(extrinsic, &pose_plus);
    if (camera_id == 0) {
      problem->SetParameterBlockConstant(extrinsic);
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
          problem->AddParameterBlock(relposes[ts].data(), 7);
          problem->SetManifold(relposes[ts].data(), &pose_plus);
        } else {
          T_C0T0 = tf(relposes[ts]);
        }

        // Add residual blocks
        for (size_t i = 0; i < point_ids.size(); i++) {
          const int point_id = point_ids[i];
          Vec3 &pt = target_geometry->getPoint(point_id);
          problem->AddParameterBlock(pt.data(), 3);
          problem->SetParameterBlockConstant(pt.data());

          auto resblock = CalibCameraError::create(camera_geometry,
                                                   target_geometry,
                                                   point_id,
                                                   relposes[ts].data(),
                                                   keypoints[i],
                                                   covar);
          resblocks.push_back(resblock);
          problem->AddResidualBlock(resblock.get(),
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
  ceres::Solve(options, problem.get(), &summary);
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

/*******************************************************************************
 * IMU methods
 ******************************************************************************/

void CalibData::addImu(const int imu_id,
                       const ImuParams &imu_params,
                       const Vec7 &extrinsic) {
  imu_geometries_[imu_id] =
      std::make_shared<ImuGeometry>(imu_id, imu_params, extrinsic);
}

int CalibData::getNumImus() const { return imu_geometries_.size(); }

std::map<int, ImuBuffer> &CalibData::getAllImuData() { return imu_data_; }

ImuBuffer &CalibData::getImuData(const int imu_id) {
  return imu_data_.at(imu_id);
}

std::map<int, ImuGeometryPtr> &CalibData::getAllImuGeometries() {
  return imu_geometries_;
}

ImuGeometryPtr &CalibData::getImuGeometry(const int imu_id) {
  return imu_geometries_.at(imu_id);
}

/*******************************************************************************
 * Calibration target methods
 ******************************************************************************/

void CalibData::addTarget(const AprilGridConfig &config,
                          const Vec7 &extrinsic) {
  const int target_id = config.target_id;
  const auto points = config.getObjectPoints();
  target_configs_[target_id] = config;
  target_geometries_[target_id] =
      std::make_shared<CalibTargetGeometry>(target_id, extrinsic, points);
}

void CalibData::addTargetPoint(const int target_id,
                               const int point_id,
                               const Vec3 &point) {
  target_geometries_[target_id]->addPoint(point_id, point);
}

void CalibData::setTargetPose(const Mat4 &target_pose) {
  target_pose_ = tf_vec(target_pose);
}

Vec7 &CalibData::getTargetPose() { return target_pose_; }

double *CalibData::getTargetPosePtr() { return target_pose_.data(); }

int CalibData::getNumTargets() const { return target_configs_.size(); }

std::map<int, CalibTargetGeometryPtr> &CalibData::getAllTargetGeometries() {
  return target_geometries_;
}

CalibTargetGeometryPtr &CalibData::getTargetGeometry(const int target_id) {
  return target_geometries_.at(target_id);
}

Vec3 &CalibData::getTargetPoint(const int target_id, const int point_id) {
  return target_geometries_.at(target_id)->getPoint(point_id);
}

/*******************************************************************************
 * Pose methods
 ******************************************************************************/

void CalibData::addPose(const timestamp_t ts, const Mat4 &pose) {
  timestamps_.insert(ts);
  poses_[ts] = tf_vec(pose);
}

Vec7 &CalibData::getPose(const timestamp_t ts) { return poses_[ts]; }

double *CalibData::getPosePtr(const timestamp_t ts) {
  return poses_[ts].data();
}

/*******************************************************************************
 * Speed and biases methods
 ******************************************************************************/

void CalibData::addSpeedAndBiases(const timestamp_t ts,
                                  const Vec3 &v_WS,
                                  const Vec3 &bias_acc,
                                  const Vec3 &bias_gyr) {
  Vec9 sb;
  sb << v_WS, bias_acc, bias_gyr;
  speed_and_biases_[ts] = sb;
}

Vec9 &CalibData::getSpeedAndBiases(const timestamp_t ts) {
  return speed_and_biases_[ts];
}

/*******************************************************************************
 * Misc methods
 ******************************************************************************/

void CalibData::printSettings(FILE *fp) const {
  fprintf(fp, "settings:\n");
  fprintf(fp, "  data_path:   \"%s\"\n", data_path_.c_str());
  fprintf(fp, "  config_path: \"%s\"\n", config_path_.c_str());
  fprintf(fp, "\n");
}

void CalibData::printCalibTargetConfigs(FILE *fp) const {
  for (const auto &[target_id, target_config] : target_configs_) {
    const std::string target_prefix = "target" + std::to_string(target_id);
    fprintf(fp, "%s:\n", target_prefix.c_str());
    fprintf(fp, "  target_type:  \"%s\"\n", "aprilgrid");
    fprintf(fp, "  tag_rows:       %d\n", target_config.tag_rows);
    fprintf(fp, "  tag_cols:       %d\n", target_config.tag_cols);
    fprintf(fp, "  tag_size:       %f\n", target_config.tag_size);
    fprintf(fp, "  tag_spacing:    %f\n", target_config.tag_spacing);
    fprintf(fp, "  tag_id_offset:  %d\n", target_config.tag_id_offset);
    fprintf(fp, "\n");
  }
}

void CalibData::printTargetGeometries(FILE *fp, const bool max_digits) const {
  for (const auto &[target_id, target] : target_geometries_) {
    const auto extrinsic = vec2str(target->getExtrinsic(), false, max_digits);
    fprintf(fp, "target%d:\n", target_id);
    fprintf(fp, "  extrinsic:  [%s]\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibData::printCameraGeometries(FILE *fp, const bool max_digits) const {
  for (const auto &[camera_id, camera] : camera_geometries_) {
    const auto model = camera->getCameraModelString();
    const auto resolution = camera->getResolution();
    const auto intrinsic = vec2str(camera->getIntrinsic(), false, max_digits);
    const auto extrinsic = vec2str(camera->getExtrinsic(), false, max_digits);

    fprintf(fp, "camera%d:\n", camera_id);
    fprintf(fp, "  model:     \"%s\"\n", model.c_str());
    fprintf(fp, "  resolution: [%d, %d]\n", resolution.x(), resolution.y());
    fprintf(fp, "  intrinsic:  [%s]\n", intrinsic.c_str());
    fprintf(fp, "  extrinsic:  [%s]\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibData::printImuGeometries(FILE *fp, const bool max_digits) const {
  for (const auto &[imu_id, imu] : imu_geometries_) {
    const auto extrinsic = vec2str(imu->getExtrinsic(), false, max_digits);
    const auto imu_params = imu->getImuParams();

    fprintf(fp, "imu%d:\n", imu_id);
    fprintf(fp, "  noise_acc:   %f\n", imu_params.noise_acc);
    fprintf(fp, "  noise_gyr:   %f\n", imu_params.noise_gyr);
    fprintf(fp, "  random_walk_acc:   %f\n", imu_params.noise_ba);
    fprintf(fp, "  random_walk_gyr:   %f\n", imu_params.noise_bg);
    fprintf(fp, "  extrinsic:  [%s]\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibData::printTargetPoints(FILE *fp) const {
  for (const auto &[target_id, target_geometry] : target_geometries_) {
    fprintf(fp, "# point_id, x, y, z\n");
    fprintf(fp, "target%d_points: [\n", target_id);
    for (const auto &[point_id, point] : target_geometry->getPoints()) {
      fprintf(fp,
              "  %d, %f, %f, %f\n",
              point_id,
              point.x(),
              point.y(),
              point.z());
    }
    fprintf(fp, "]\n");
    fprintf(fp, "\n");
  }
}

void CalibData::printSummary(FILE *fp, const bool max_digits) const {
  printSettings(fp);
  printCalibTargetConfigs(fp);
  printCameraGeometries(fp, max_digits);
}

void CalibData::saveResults(const std::string &save_path) const {
  FILE *fp = fopen(save_path.c_str(), "w");
  if (fp == NULL) {
    FATAL("Failed to open file [%s]", save_path.c_str());
  }

  printSettings(fp);
  printCalibTargetConfigs(fp);
  printCameraGeometries(fp, true);

  fclose(fp);
}

} // namespace xyz
