#include "EuRoC.hpp"

namespace xyz {

///////////////////////////////////////////////////////////////////////////////
// EurocImu                                                                  //
///////////////////////////////////////////////////////////////////////////////

EurocImu::EurocImu(const std::string &data_dir_) : data_dir{data_dir_} {
  const std::string data_path = data_dir + "/data.csv";
  const std::string sensor_path = data_dir + "/sensor.yaml";

  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    double a_x, a_y, a_z = 0.0;
    int retval = fscanf(fp,
                        "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &w_x,
                        &w_y,
                        &w_z,
                        &a_x,
                        &a_y,
                        &a_z);
    if (retval != 7) {
      FATAL("Failed to parse line in [%s]", data_path.c_str());
    }

    timestamps.push_back(ts);
    w_B.emplace_back(w_x, w_y, w_z);
    a_B.emplace_back(a_x, a_y, a_z);
  }
  fclose(fp);

  // Load calibration data
  config_t config{sensor_path};
  if (config.ok != true) {
    FATAL("Failed to load sensor file [%s]!", sensor_path.c_str());
  }
  parse(config, "sensor_type", sensor_type);
  parse(config, "comment", comment);
  parse(config, "T_BS", T_BS);
  parse(config, "rate_hz", rate_hz);
  parse(config, "gyroscope_noise_density", gyro_noise_density);
  parse(config, "gyroscope_random_walk", gyro_random_walk);
  parse(config, "accelerometer_noise_density", accel_noise_density);
  parse(config, "accelerometer_random_walk", accel_random_walk);

  ok = true;
}

void EurocImu::print() const {
  printf("sensor_type: %s\n", sensor_type.c_str());
  printf("comment: %s\n", comment.c_str());
  print_matrix("T_BS", T_BS);
  printf("rate_hz: %f\n", rate_hz);
  printf("gyroscope_noise_density: %e\n", gyro_noise_density);
  printf("gyroscope_random_walk: %e\n", gyro_random_walk);
  printf("accelerometer_noise_density: %e\n", accel_noise_density);
  printf("accelerometer_random_walk: %e\n", accel_random_walk);
}

///////////////////////////////////////////////////////////////////////////////
// EurocCamera                                                               //
///////////////////////////////////////////////////////////////////////////////

EurocCamera::EurocCamera(const std::string &data_dir_, bool is_calib_data)
    : data_dir{data_dir_} {
  const std::string data_path = data_dir + "/data.csv";
  const std::string sensor_path = data_dir + "/sensor.yaml";

  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    char filename[50] = {0};
    int retval = fscanf(fp, "%" SCNu64 ",%s", &ts, filename);
    if (retval != 2) {
      FATAL("Failed to parse line in [%s]", data_path.c_str());
    }

    // Check if file exists
    const std::string image_file{filename};
    const auto image_path = data_dir + "/data/" + image_file;
    if (file_exists(image_path) == false) {
      FATAL("File [%s] does not exist!", image_path.c_str());
    }

    // Add data
    timestamps.emplace_back(ts);
    image_paths.emplace_back(image_path);
  }
  fclose(fp);

  // Load calibration data
  config_t config{sensor_path};
  if (config.ok == false) {
    FATAL("Failed to load senor file [%s]!", sensor_path.c_str());
  }
  parse(config, "sensor_type", sensor_type);
  parse(config, "comment", comment);
  parse(config, "T_BS", T_BS);
  parse(config, "rate_hz", rate_hz);
  parse(config, "resolution", resolution);
  parse(config, "camera_model", camera_model, is_calib_data);
  parse(config, "intrinsics", intrinsics, is_calib_data);
  parse(config, "distortion_model", distortion_model, is_calib_data);
  parse(config,
        "distortion_coefficients",
        distortion_coefficients,
        is_calib_data);

  ok = true;
}

void EurocCamera::print() const {
  printf("sensor_type: %s\n", sensor_type.c_str());
  printf("comment: %s\n", comment.c_str());
  print_matrix("T_BS:\n", T_BS);
  printf("rate_hz: %f", rate_hz);
  printf("resolution: [%d, %d]\n", resolution.x(), resolution.y());
  printf("camera_model: %s\n", camera_model.c_str());
  printf("intrinsics: %s\n", vec2str(intrinsics).c_str());
  printf("distortion_model: %s\n", distortion_model.c_str());
  printf("distortion_coefficients: %s\n",
         vec2str(distortion_coefficients).c_str());
}

///////////////////////////////////////////////////////////////////////////////
// EurocGroundTruth                                                          //
///////////////////////////////////////////////////////////////////////////////

EurocGroundTruth::EurocGroundTruth(const std::string &data_dir_)
    : data_dir{data_dir_} {
  // Open file for loading
  const std::string data_path = data_dir + "/data.csv";
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  std::string str_format;
  str_format += "%" SCNu64 ",";     // Timestamp
  str_format += "%lf,%lf,%lf,";     // Position
  str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
  str_format += "%lf,%lf,%lf,";     // Velocity
  str_format += "%lf,%lf,%lf,";     // Gyro bias
  str_format += "%lf,%lf,%lf";      // Accel bias

  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double p_x, p_y, p_z = 0.0;
    double q_x, q_y, q_z, q_w = 0.0;
    double v_x, v_y, v_z = 0.0;
    double b_w_x, b_w_y, b_w_z = 0.0;
    double b_a_x, b_a_y, b_a_z = 0.0;
    int retval = fscanf(fp,
                        str_format.c_str(),
                        &ts,
                        &p_x,
                        &p_y,
                        &p_z,
                        &q_x,
                        &q_y,
                        &q_z,
                        &q_w,
                        &v_x,
                        &v_y,
                        &v_z,
                        &b_w_x,
                        &b_w_y,
                        &b_w_z,
                        &b_a_x,
                        &b_a_y,
                        &b_a_z);
    if (retval != 17) {
      FATAL("Failed to parse line in [%s]", data_path.c_str());
    }

    timestamps.push_back(ts);
    p_RS_R.emplace_back(p_x, p_y, p_z);
    q_RS.emplace_back(q_w, q_x, q_y, q_z);
    v_RS_R.emplace_back(v_x, v_y, v_z);
    b_w_RS_S.emplace_back(b_w_x, b_w_y, b_w_z);
    b_a_RS_S.emplace_back(b_a_x, b_a_y, b_a_z);
  }
  fclose(fp);

  ok = true;
}

///////////////////////////////////////////////////////////////////////////////
// EurocData                                                                 //
///////////////////////////////////////////////////////////////////////////////

EurocData::EurocData(const std::string &data_path)
    : data_path{strip_end(data_path, "/")} {
  // Load IMU data
  imu_data = EurocImu{data_path + "/mav0/imu0"};
  for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
    const timestamp_t ts = imu_data.timestamps[i];
    const Vec3 acc = imu_data.a_B[i];
    const Vec3 gyr = imu_data.w_B[i];
    timeline.add(ts, acc, gyr);
  }

  // Load camera data
  // -- Load cam0 data
  const auto cam0_path = data_path + "/mav0/cam0";
  cam0_data = EurocCamera{cam0_path};
  for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
    const timestamp_t ts = cam0_data.timestamps[i];
    const auto image_path = cam0_data.image_paths[i];
    timeline.add(ts, 0, image_path);
  }
  // -- Load cam1 data
  const auto cam1_path = data_path + "/mav0/cam1";
  cam1_data = EurocCamera{cam0_path};
  for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
    const timestamp_t ts = cam1_data.timestamps[i];
    const auto image_path = cam1_data.image_paths[i];
    timeline.add(ts, 1, image_path);
  }
  // -- Set camera image size
  cv::Mat image = cv::imread(cam0_data.image_paths[0]);
  image_size = cv::Size(image.size());

  // Load ground truth
  const auto gt_path = data_path + "/mav0/state_groundtruth_estimate0";
  ground_truth = EurocGroundTruth{gt_path};

  // Process timestamps
  ts_start = min_timestamp();
  ts_end = max_timestamp();
  ts_now = ts_start;

  // Get timestamps and calculate relative time
  auto it = timeline.timestamps.begin();
  auto it_end = timeline.timestamps.end();
  while (it != it_end) {
    const timestamp_t ts = *it;
    time[ts] = ((double) ts - ts_start) * 1e-9;

    // Advance to next non-duplicate entry.
    do {
      ++it;
    } while (ts == *it);
  }

  ok = true;
}

void EurocData::reset() {
  ts_start = min_timestamp();
  ts_end = max_timestamp();
  ts_now = ts_start;
  time_index = 0;
  imu_index = 0;
  frame_index = 0;
}

timestamp_t EurocData::min_timestamp() const {
  const timestamp_t cam0_ts0 = cam0_data.timestamps.front();
  const timestamp_t imu_ts0 = imu_data.timestamps.front();

  timestamps_t first_ts{cam0_ts0, imu_ts0};
  auto ts0 = std::min_element(first_ts.begin(), first_ts.end());
  const size_t first_ts_index = std::distance(first_ts.begin(), ts0);

  return first_ts[first_ts_index];
}

timestamp_t EurocData::max_timestamp() const {
  const timestamp_t cam0_last_ts = cam0_data.timestamps.back();
  const timestamp_t imu_last_ts = imu_data.timestamps.back();

  timestamps_t last_ts{cam0_last_ts, imu_last_ts};
  auto last_result = std::max_element(last_ts.begin(), last_ts.end());
  const timestamp_t last_ts_index = std::distance(last_ts.begin(), last_result);
  const timestamp_t max_ts = last_ts[last_ts_index];

  return max_ts;
}

EurocTarget::EurocTarget(const std::string &target_file)
    : file_path{target_file} {
  config_t config{target_file};
  if (config.ok != true) {
    FATAL("Failed to load target file [%s]!", target_file.c_str());
  }
  parse(config, "target_type", type);
  parse(config, "tagRows", tag_rows);
  parse(config, "tagCols", tag_cols);
  parse(config, "tagSize", tag_size);
  parse(config, "tagSpacing", tag_spacing);

  ok = true;
}

void EurocTarget::print() const {
  printf("target_type: %s\n", type.c_str());
  printf("tag_rows: %d\n", tag_rows);
  printf("tag_cols: %d\n", tag_cols);
  printf("tag_size: %f\n", tag_size);
  printf("tag_spacing: %f\n", tag_spacing);
}

///////////////////////////////////////////////////////////////////////////////
// EurocCalib                                                                //
///////////////////////////////////////////////////////////////////////////////

EurocCalib::EurocCalib(const std::string &data_path)
    : data_path{strip_end(data_path, "/")} {
  // Load IMU data
  const std::string imu_data_dir = data_path + "/mav0/imu0";
  imu_data = EurocImu{imu_data_dir};

  // Load cam0 data
  const std::string cam0_dir = data_path + "/mav0/cam0";
  cam0_data = EurocCamera{cam0_dir, true};

  // Load cam1 data
  const std::string cam1_dir = data_path + "/mav0/cam1";
  cam1_data = EurocCamera{cam1_dir, true};

  // Check if cam0 has same amount of images as cam1
  const size_t cam0_nb_images = cam0_data.image_paths.size();
  const size_t cam1_nb_images = cam1_data.image_paths.size();
  if (cam0_nb_images != cam1_nb_images) {
    if (cam0_nb_images > cam1_nb_images) {
      cam0_data.timestamps.pop_back();
      cam0_data.image_paths.pop_back();
    } else if (cam0_nb_images < cam1_nb_images) {
      cam1_data.timestamps.pop_back();
      cam1_data.image_paths.pop_back();
    }
  }

  // Get image size
  const cv::Mat image = cv::imread(cam0_data.image_paths[0]);
  image_size = cv::Size(image.size());

  // Load calibration target data
  const std::string target_path = data_path + "/april_6x6.yaml";
  calib_target = EurocTarget{target_path};

  ok = true;
}

Timeline EurocCalib::timeline() {
  // Create timeline
  Timeline timeline;

  // -- Add cam0 events
  for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
    const auto ts = cam0_data.timestamps[i];
    const auto img_path = cam0_data.image_paths[i];
    timeline.add(ts, 0, img_path);
  }
  // -- Add cam1 events
  for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
    const auto ts = cam1_data.timestamps[i];
    const auto img_path = cam1_data.image_paths[i];
    timeline.add(ts, 1, img_path);
  }
  // -- Add imu events
  for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
    const timestamp_t ts = imu_data.timestamps[i];
    const auto acc = imu_data.a_B[i];
    const auto gyr = imu_data.w_B[i];
    timeline.add(ts, acc, gyr);
  }

  return timeline;
}

} // namespace xyz
