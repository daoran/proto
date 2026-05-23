#include "CalibProblem.hpp"
#include "CameraChain.hpp"
#include "CalibTargetChain.hpp"

namespace cartesian {

CalibProblem::CalibProblem() {
  // Setup solver
  setupSolver();
}

CalibProblem::CalibProblem(const std::string &config_path_)
    : config_path{config_path_} {
  // Setup solver
  setupSolver();

  // Parse config
  config_t config{config_path};

  // -- Parse data settings
  parse(config, "data_path", data_path);

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
      FATAL("CalibProblem currently only supports target type [aprilgrid]!");
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

    // Load camera data
    loadCameraData(camera_id);
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
 * Setup methods
 ******************************************************************************/

void CalibProblem::setupSolver() {
  prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = std::make_shared<ceres::Problem>(prob_options);
}

/*******************************************************************************
 * Load methods
 ******************************************************************************/

void CalibProblem::loadCameraData(const int camera_id) {
  // Setup detector
  AprilGridDetector detector{target_configs};

  // Form target dirs
  bool cache_exists = false;
  const std::string camera_string = "cam" + std::to_string(camera_id);
  const fs::path &camera_dir = data_path / camera_string / "data";

  for (size_t target_id = 0; target_id < target_configs.size(); ++target_id) {
    // Check if target dir exists
    const std::string target_prefix = "target" + std::to_string(target_id);
    const fs::path &target_dir = data_path / target_prefix / camera_string;
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
  std::vector<fs::path> image_paths;
  for (const auto &path : fs::directory_iterator(camera_dir)) {
    image_paths.push_back(path);
  }
  sort(image_paths.begin(), image_paths.end());

  // Detect aprilgrids
  if (cache_exists == false) {
    const std::string desc = "Processing " + camera_string + " data: ";
    for (size_t k = 0; k < image_paths.size(); k++) {
      if (verbose) {
        print_progress((double) k / (double) image_paths.size(), desc);
      }

      const fs::path image_path = image_paths[k];
      const std::string ts_str = image_path.stem();
      const timestamp_t ts = std::stoull(ts_str);
      if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
        FATAL("Invalid image file: [%s]!", image_path.c_str());
      }

      const auto &image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      for (const auto &target : detector.detect(ts, camera_id, image)) {
        const int target_id = target->get_target_id();
        const fs::path target_prefix = "target" + std::to_string(target_id);
        const fs::path target_dir = data_path / target_prefix / camera_string;
        const fs::path target_csv = target_dir / (ts_str + ".csv");
        target->save(target_csv);
      }
    }
    if (verbose) {
      print_progress(1.0);
    }
  }

  // Load aprilgrids
  const std::string desc = "Loading " + camera_string + " data: ";
  for (size_t k = 0; k < image_paths.size(); k++) {
    if (verbose) {
      print_progress((double) k / (double) image_paths.size(), desc);
    }

    const fs::path image_path = image_paths[k];
    const std::string ts_str = image_path.stem();
    const timestamp_t ts = std::stoull(ts_str);
    const fs::path target_fname = ts_str + ".csv";
    if (std::all_of(ts_str.begin(), ts_str.end(), ::isdigit) == false) {
      FATAL("Invalid image file: [%s]!", image_path.c_str());
    }

    for (const auto &[target_id, target_config] : target_configs) {
      const fs::path target_prefix = "target" + std::to_string(target_id);
      const fs::path target_dir = data_path / target_prefix / camera_string;
      const fs::path target_csv = target_dir / (ts_str + ".csv");

      // Load apriltag
      std::shared_ptr<AprilGrid> calib_target = AprilGrid::load(target_csv);

      // Add camera measurement
      addCameraMeasurement(ts, camera_id, calib_target);
    }
  }
  if (verbose) {
    print_progress(1.0);
  }
}

void CalibProblem::loadImuData(const int imu_id) {
  // Setup
  const std::string imu_string = "imu" + std::to_string(imu_id);
  const fs::path imu_path = data_path / imu_string / "data.csv";

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
    if (verbose) {
      print_progress((double) i / (double) num_rows, desc);
    }

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
  if (verbose) {
    print_progress(1.0);
  }

  // Add imu data
  imu_data[imu_id] = imu_buffer;
}

/*******************************************************************************
 * Camera methods
 ******************************************************************************/

void CalibProblem::addCamera(const int camera_id,
                             const std::string &camera_model,
                             const Vec2i &resolution,
                             const VecX &intrinsic,
                             const Vec7 &extrinsic) {
  // Add camera geometry
  auto camera = std::make_shared<CameraGeometry>(camera_id,
                                                 camera_model,
                                                 resolution,
                                                 intrinsic,
                                                 extrinsic);
  camera_geometries[camera_id] = camera;

  // Add camera intrinsic
  const int intrinsic_size = camera->intrinsic.size();
  problem->AddParameterBlock(camera->intrinsic.data(), intrinsic_size);

  // Add camera extrinsic
  problem->AddParameterBlock(camera->extrinsic.data(), 7);
  problem->SetManifold(camera->extrinsic.data(), &pose_plus);
  if (camera_id == 0) {
    problem->SetParameterBlockConstant(camera->extrinsic.data());
  }
}

void CalibProblem::addCameraMeasurement(const timestamp_t ts,
                                        const int camera_id,
                                        const CalibTargetPtr &calib_target) {
  const int target_id = calib_target->get_target_id();
  camera_data[camera_id][ts][target_id] = calib_target;
}

bool CalibProblem::hasCameraMeasurement(const timestamp_t ts,
                                        const int camera_id,
                                        const int target_id) const {
  if (camera_data.count(camera_id) == 0) {
    return false;
  } else if (camera_data.at(camera_id).count(ts) == 0) {
    return false;
  } else if (camera_data.at(camera_id).at(ts).count(target_id) == 0) {
    return false;
  }
  return true;
}

int CalibProblem::getNumCameras() const { return camera_geometries.size(); }

std::map<int, CameraData> &CalibProblem::getAllCameraData() {
  return camera_data;
}

CameraData &CalibProblem::getCameraData(const int camera_id) {
  return camera_data.at(camera_id);
}

std::map<int, CameraGeometryPtr> &CalibProblem::getAllCameraGeometries() {
  return camera_geometries;
}

CameraGeometryPtr &CalibProblem::getCameraGeometry(const int camera_id) {
  return camera_geometries.at(camera_id);
}

void CalibProblem::initializeCameraIntrinsics() {
  // Pre-check
  if (camera_data.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries.size() == 0) {
    FATAL("No cameras added?");
  }
  if (target_configs.size() == 0 || target_geometries.size() == 0) {
    FATAL("No targets added?");
  }

  // Initialize camera intrinsics
  CalibInit::initializeCameraIntrinsics(getAllCameraData(),
                                        target_configs,
                                        getAllCameraGeometries());
}

void CalibProblem::initializeCameraExtrinsics() {
  // Pre-check
  if (getNumCameras() < 2) {
    return;
  }
  if (camera_data.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries.size() == 0) {
    FATAL("No cameras added?");
  }
  if (target_configs.size() == 0 || target_geometries.size() == 0) {
    FATAL("No targets added?");
  }

  // Initialize camera extrinsics
  CalibInit::initializeCameraExtrinsics(getAllCameraData(),
                                        getAllCameraGeometries(),
                                        getAllTargetGeometries());
}

/*******************************************************************************
 * IMU methods
 ******************************************************************************/

void CalibProblem::addImu(const int imu_id,
                          const ImuParams &imu_params,
                          const Vec7 &extrinsic) {
  // Add IMU geometry
  auto imu = std::make_shared<ImuGeometry>(imu_id, imu_params, extrinsic);
  imu_geometries[imu_id] = imu;

  // Add IMU extrinsic
  problem->AddParameterBlock(imu_geometries[imu_id]->extrinsic.data(), 7);
  problem->SetManifold(imu_geometries[imu_id]->extrinsic.data(), &pose_plus);
}

int CalibProblem::getNumImus() const { return imu_geometries.size(); }

std::map<int, ImuBuffer> &CalibProblem::getAllImuData() { return imu_data; }

ImuBuffer &CalibProblem::getImuData(const int imu_id) {
  return imu_data.at(imu_id);
}

std::map<int, ImuGeometryPtr> &CalibProblem::getAllImuGeometries() {
  return imu_geometries;
}

ImuGeometryPtr &CalibProblem::getImuGeometry(const int imu_id) {
  return imu_geometries.at(imu_id);
}

/*******************************************************************************
 * Calibration target methods
 ******************************************************************************/

void CalibProblem::addTarget(const AprilGridConfig &config,
                             const Vec7 &extrinsic) {
  // Add target config and geometry
  const int target_id = config.target_id;
  const auto pts = config.getObjectPoints();
  auto target =
      std::make_shared<CalibTargetGeometry>(target_id, extrinsic, pts);
  target_configs[target_id] = config;
  target_geometries[target_id] = target;

  // Target extrinsic
  problem->AddParameterBlock(target->extrinsic.data(), 7);
  problem->SetManifold(target->extrinsic.data(), &pose_plus);
  if (target_id == 0) {
    problem->SetParameterBlockConstant(target->extrinsic.data());
  }

  // Target points
  for (auto &[point_id, point] : config.getObjectPoints()) {
    target->points[point_id] = point;
    problem->AddParameterBlock(target->points[point_id].data(), 3);
    problem->SetParameterBlockConstant(target->points[point_id].data());
  }
}

void CalibProblem::setTargetPose(const Mat4 &transform) {
  target_pose = tf_vec(transform);
  problem->AddParameterBlock(target_pose.data(), 7);
  problem->SetManifold(target_pose.data(), &pose_plus);
  // problem->SetParameterBlockConstant(target_pose.data());
}

Vec7 &CalibProblem::getTargetPose() { return target_pose; }

double *CalibProblem::getTargetPosePtr() { return target_pose.data(); }

int CalibProblem::getNumTargets() const { return target_configs.size(); }

std::map<int, CalibTargetGeometryPtr> &CalibProblem::getAllTargetGeometries() {
  return target_geometries;
}

CalibTargetGeometryPtr &CalibProblem::getTargetGeometry(const int target_id) {
  return target_geometries.at(target_id);
}

Vec3 &CalibProblem::getTargetPoint(const int target_id, const int point_id) {
  return target_geometries.at(target_id)->points[point_id];
}

/*******************************************************************************
 * Pose methods
 ******************************************************************************/

void CalibProblem::addPose(const timestamp_t ts, const Mat4 &pose) {
  timestamps.insert(ts);
  poses[ts] = tf_vec(pose);
  problem->AddParameterBlock(poses.at(ts).data(), 7);
  problem->SetManifold(poses.at(ts).data(), &pose_plus);
}

Vec7 &CalibProblem::getPose(const timestamp_t ts) { return poses[ts]; }

double *CalibProblem::getPosePtr(const timestamp_t ts) {
  return poses.at(ts).data();
}

/*******************************************************************************
 * Speed and biases methods
 ******************************************************************************/

void CalibProblem::addSpeedAndBiases(const timestamp_t ts,
                                     const Vec3 &v_WS,
                                     const Vec3 &bias_acc,
                                     const Vec3 &bias_gyr) {
  Vec9 sb;
  sb << v_WS, bias_acc, bias_gyr;
  speed_and_biases[ts] = sb;
  problem->AddParameterBlock(speed_and_biases.at(ts).data(), 9);
}

Vec9 &CalibProblem::getSpeedAndBiases(const timestamp_t ts) {
  return speed_and_biases.at(ts);
}

double *CalibProblem::getSpeedAndBiasesPtr(const timestamp_t ts) {
  return speed_and_biases.at(ts).data();
}

/*******************************************************************************
 * Ceres
 ******************************************************************************/

void CalibProblem::addResidualBlock(ResidualBlock *resblock) {
  problem->AddResidualBlock(resblock, nullptr, resblock->getParamPtrs());
}

/*******************************************************************************
 * Misc methods
 ******************************************************************************/

void CalibProblem::printSettings(FILE *fp) const {
  fprintf(fp, "settings:\n");
  fprintf(fp, "  data_path:   \"%s\"\n", data_path.c_str());
  fprintf(fp, "  config_path: \"%s\"\n", config_path.c_str());
  fprintf(fp, "\n");
}

void CalibProblem::printCalibTargetConfigs(FILE *fp) const {
  for (const auto &[target_id, target_config] : target_configs) {
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

void CalibProblem::printTargetGeometries(FILE *fp,
                                         const bool max_digits) const {
  // Print target0 pose
  fprintf(fp, "target0:\n");
  fprintf(fp, "  pose: %s\n", vec2str(target_pose, true, max_digits).c_str());
  fprintf(fp, "\n");

  // Print target geometries
  for (const auto &[target_id, target] : target_geometries) {
    const auto extrinsic = vec2str(target->extrinsic, true, max_digits);
    fprintf(fp, "target%d:\n", target_id);
    fprintf(fp, "  extrinsic: %s\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibProblem::printCameraGeometries(FILE *fp,
                                         const bool max_digits) const {
  for (const auto &[camera_id, camera] : camera_geometries) {
    const auto model = camera->camera_model->type();
    const auto resolution = camera->resolution;
    const auto intrinsic = vec2str(camera->intrinsic, false, max_digits);
    const auto extrinsic = vec2str(camera->extrinsic, false, max_digits);

    fprintf(fp, "camera%d:\n", camera_id);
    fprintf(fp, "  model:     \"%s\"\n", model.c_str());
    fprintf(fp, "  resolution: [%d, %d]\n", resolution.x(), resolution.y());
    fprintf(fp, "  intrinsic:  [%s]\n", intrinsic.c_str());
    fprintf(fp, "  extrinsic:  [%s]\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibProblem::printImuGeometries(FILE *fp, const bool max_digits) const {
  for (const auto &[imu_id, imu] : imu_geometries) {
    const auto extrinsic = vec2str(imu->extrinsic, false, max_digits);
    const auto imu_params = imu->imu_params;

    fprintf(fp, "imu%d:\n", imu_id);
    fprintf(fp, "  noise_acc:         %.4e\n", imu_params.noise_acc);
    fprintf(fp, "  noise_gyr:         %.4e\n", imu_params.noise_gyr);
    fprintf(fp, "  random_walk_acc:   %.4e\n", imu_params.noise_ba);
    fprintf(fp, "  random_walk_gyr:   %.4e\n", imu_params.noise_bg);
    fprintf(fp, "  extrinsic:  [%s]\n", extrinsic.c_str());
    fprintf(fp, "\n");
  }
}

void CalibProblem::printTargetPoints(FILE *fp) const {
  for (const auto &[target_id, target_geometry] : target_geometries) {
    fprintf(fp, "# point_id, x, y, z\n");
    fprintf(fp, "target%d_points: [\n", target_id);
    for (const auto &[point_id, point] : target_geometry->points) {
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

void CalibProblem::printSummary(FILE *fp, const bool max_digits) const {
  printSettings(fp);
  printCalibTargetConfigs(fp);
  printCameraGeometries(fp, max_digits);
  printImuGeometries(fp, max_digits);
  printTargetGeometries(fp, max_digits);
}

void CalibProblem::saveResults(const std::string &save_path) const {
  FILE *fp = fopen(save_path.c_str(), "w");
  if (fp == NULL) {
    FATAL("Failed to open file [%s]", save_path.c_str());
  }

  printSettings(fp);
  printCalibTargetConfigs(fp);
  printCameraGeometries(fp, true);

  fclose(fp);
}

} // namespace cartesian
