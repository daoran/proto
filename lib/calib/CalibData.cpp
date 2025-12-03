#include "CalibData.hpp"

namespace xyz {

CalibData::CalibData(const std::string &config_path)
    : config_path_{config_path} {
  // Parse config
  config_t config{config_path_};

  // -- Parse data settings
  parse(config, "data_path", data_path_);

  // -- Parse target settings
  for (int target_id = 0; target_id < 10; target_id++) {
    // Check if key exists
    const std::string prefix = "target" + std::to_string(target_id);
    if (yaml_has_key(config, prefix) == 0) {
      continue;
    }

    std::string target_type;
    int tag_rows = 0;
    int tag_cols = 0;
    double tag_size = 0;
    double tag_spacing = 0;
    int tag_id_offset = 0;
    parse(config, prefix + ".target_type", target_type);
    parse(config, prefix + ".tag_rows", tag_rows);
    parse(config, prefix + ".tag_cols", tag_cols);
    parse(config, prefix + ".tag_size", tag_size);
    parse(config, prefix + ".tag_spacing", tag_spacing);
    parse(config, prefix + ".tag_id_offset", tag_id_offset, true);
    if (target_type != "aprilgrid") {
      FATAL("CalibData currently only supports target type [aprilgrid]!");
    }

    target_configs_[target_id] =
        AprilGridConfig{target_id, tag_rows, tag_cols, tag_size, tag_spacing};
  }

  // -- Parse camera settings
  for (int camera_id = 0; camera_id < 10; camera_id++) {
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

void CalibData::loadCameraData(const int camera_id) {
  assert(target_type_ == "aprilgrid");
  assert(tag_rows_ > 0);
  assert(tag_cols_ > 0);
  assert(tag_size_ > 0);
  assert(tag_spacing_ > 0);

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

bool CalibData::hasCameraMeasurement(const timestamp_t ts,
                                     const int camera_id) const {
  if (camera_data_.at(camera_id).count(ts) == 0) {
    return false;
  }
  return true;
}

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

void CalibData::addImu(const int imu_id,
                       const ImuParams &imu_params,
                       const Vec7 &extrinsic) {
  imu_geometries_[imu_id] =
      std::make_shared<ImuGeometry>(imu_id, imu_params, extrinsic);
}

void CalibData::addCameraMeasurement(const timestamp_t ts,
                                     const int camera_id,
                                     const CalibTargetPtr &calib_target) {
  const int target_id = calib_target->getTargetId();
  camera_data_[camera_id][ts][target_id] = calib_target;
}

void CalibData::addTargetPoint(const int target_id,
                               const int point_id,
                               const Vec3 &point) {
  target_geometries_[target_id]->addPoint(point_id, point);
}

int CalibData::getNumCameras() const { return camera_geometries_.size(); }

int CalibData::getNumImus() const { return imu_geometries_.size(); }

int CalibData::getNumTargets() const { return target_configs_.size(); }

std::map<int, CameraData> &CalibData::getAllCameraData() {
  return camera_data_;
}

CameraData &CalibData::getCameraData(const int camera_id) {
  return camera_data_.at(camera_id);
}

std::map<int, ImuBuffer> &CalibData::getAllImuData() { return imu_data_; }

ImuBuffer &CalibData::getImuData(const int imu_id) {
  return imu_data_.at(imu_id);
}

std::map<int, CameraGeometryPtr> &CalibData::getAllCameraGeometries() {
  return camera_geometries_;
}

CameraGeometryPtr &CalibData::getCameraGeometry(const int camera_id) {
  return camera_geometries_.at(camera_id);
}

std::map<int, ImuGeometryPtr> &CalibData::getAllImuGeometries() {
  return imu_geometries_;
}

ImuGeometryPtr &CalibData::getImuGeometry(const int imu_id) {
  return imu_geometries_.at(imu_id);
}

std::map<int, CalibTargetGeometryPtr> &CalibData::getAllTargetGeometries() {
  return target_geometries_;
}

CalibTargetGeometryPtr &CalibData::getTargetGeometry(const int target_id) {
  return target_geometries_.at(target_id);
}

Vec3 &CalibData::getTargetPoint(const int target_id, const int point_id) {
  return target_geometries_.at(target_id)->getPoint(point_id);
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
