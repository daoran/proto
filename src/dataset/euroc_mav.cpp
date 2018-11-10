#include "prototype/dataset/euroc_mav.hpp"

namespace prototype {
namespace euroc {

/*****************************************************************************
 * calib_target_t
 ****************************************************************************/

calib_target_t::calib_target_t() {}

calib_target_t::calib_target_t(const std::string &file_path)
  : file_path{file_path} {
  if (calib_target_load(*this,  file_path) == 0) {
    ok = true;
  }
}

calib_target_t::~calib_target_t() {}

int calib_target_load(calib_target_t &target,
                      const std::string &target_file) {
  config_t config{target_file};
  if (config.ok != true) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  parse(config, "target_type", target.type);
  parse(config, "tagRows", target.tag_rows);
  parse(config, "tagCols", target.tag_cols);
  parse(config, "tagSize", target.tag_size);
  parse(config, "tagSpacing", target.tag_spacing);

  return 0;
}

std::ostream &operator<<(std::ostream &os, const calib_target_t &target) {
  os << "target_type: " << target.type << std::endl;
  os << "tag_rows: " << target.tag_rows << std::endl;
  os << "tag_cols: " << target.tag_cols << std::endl;
  os << "tag_size: " << target.tag_size << std::endl;
  os << "tag_spacing: " << target.tag_spacing << std::endl;
  return os;
}



/*****************************************************************************
 * imu_data_t
 ****************************************************************************/

imu_data_t::imu_data_t() {}

imu_data_t::imu_data_t(const std::string &data_dir_) : data_dir{data_dir_} {}

imu_data_t::~imu_data_t() {}

int imu_data_load(imu_data_t &data, const std::string &data_dir) {
  data.data_dir = data_dir;
  const std::string imu_data_path = data.data_dir + "/data.csv";
  const std::string imu_calib_path = data.data_dir + "/sensor.yaml";

  // Load IMU data
  matx_t csv_data;
  if (csv2mat(imu_data_path, true, csv_data) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_path.c_str());
    return -1;
  }

  const long t0 = csv_data(0, 0);
  for (long i = 0; i < csv_data.rows(); i++) {
    const long ts = csv_data(i, 0);
    data.timestamps.push_back(ts);
    data.time.push_back((ts - t0) * 1e-9);
    data.w_B.emplace_back(csv_data(i, 1), csv_data(i, 2), csv_data(i, 3));
    data.a_B.emplace_back(csv_data(i, 4), csv_data(i, 5), csv_data(i, 6));
  }

  // Load calibration data
  config_t config{imu_calib_path};
  if (config.ok != false) {
    LOG_ERROR("Failed to load sensor file [%s]!", imu_calib_path.c_str());
    return -1;
  }
  parse(config, "sensor_type", data.sensor_type);
  parse(config, "comment", data.comment);
  parse(config, "T_BS", data.T_BS);
  parse(config, "rate_hz", data.rate_hz);
  parse(config, "gyroscope_noise_density", data.gyro_noise_density);
  parse(config, "gyroscope_random_walk", data.gyro_random_walk);
  parse(config, "accelerometer_noise_density", data.accel_noise_density);
  parse(config, "accelerometer_random_walk", data.accel_random_walk);

  return 0;
}

std::ostream &operator<<(std::ostream &os, const imu_data_t &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "gyroscope_noise_density: " << data.gyro_noise_density << std::endl;
  os << "gyroscope_random_walk: " << data.gyro_random_walk << std::endl;
  os << "accelerometer_noise_density: " << data.accel_noise_density << std::endl;
  os << "accelerometer_random_walk: " << data.accel_random_walk << std::endl;
  // clang-format on

  return os;
}



/*****************************************************************************
 * camera_data_t
 ****************************************************************************/

camera_data_t::camera_data_t() {}

camera_data_t::camera_data_t(const std::string &data_dir_) : data_dir{data_dir_} {}

camera_data_t::~camera_data_t() {}

int camera_data_load(camera_data_t &cd, const std::string &data_dir) {
  cd.data_dir = data_dir;
  const std::string cam_data_path = cd.data_dir + "/data.csv";
  const std::string cam_calib_path = cd.data_dir + "/sensor.yaml";

  // Load camera data
  matx_t data;
  if (csv2mat(cam_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load camera data [%s]!", cam_data_path.c_str());
    return -1;
  }

  const long t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const std::string image_file = std::to_string((long) data(i, 0)) + ".png";
    const std::string image_path = cd.data_dir + "/data/" + image_file;
    const long ts = data(i, 0);

    if (file_exists(image_path) == false) {
      LOG_ERROR("File [%s] does not exist!", image_path.c_str());
      return -1;
    }

    cd.timestamps.emplace_back(ts);
    cd.time.emplace_back((ts - t0) * 1e-9);
    cd.image_paths.emplace_back(image_path);
  }

  // Load calibration data
  config_t config{cam_calib_path};
  if (config.ok == false) {
    LOG_ERROR("Failed to load senor file [%s]!", cam_calib_path.c_str());
    return -1;
  }
  parse(config, "sensor_type", cd.sensor_type);
  parse(config, "comment", cd.comment);
  parse(config, "T_BS", cd.T_BS);
  parse(config, "rate_hz", cd.rate_hz);
  parse(config, "resolution", cd.resolution);
  parse(config, "camera_model", cd.camera_model);
  parse(config, "intrinsics", cd.intrinsics);
  parse(config, "distortion_model", cd.distortion_model);
  parse(config, "distortion_coefficients", cd.distortion_coefficients);

  return 0;
}

std::ostream &operator<<(std::ostream &os, const camera_data_t &cd) {
  // clang-format off
  os << "sensor_type: " << cd.sensor_type << std::endl;
  os << "comment: " << cd.comment << std::endl;
  os << "T_BS:\n" << cd.T_BS << std::endl;
  os << "rate_hz: " << cd.rate_hz << std::endl;
  os << "resolution: " << cd.resolution.transpose() << std::endl;
  os << "camera_model: " << cd.camera_model << std::endl;
  os << "intrinsics: " << cd.intrinsics.transpose() << std::endl;
  os << "distortion_model: " << cd.distortion_model << std::endl;
  os << "distortion_coefficients: " << cd.distortion_coefficients.transpose() << std::endl;
  // clang-format on

  return os;
}



/*****************************************************************************
 * ground_truth_t
 ****************************************************************************/

ground_truth_t::ground_truth_t() {}

ground_truth_t::ground_truth_t(const std::string data_dir_) : data_dir{data_dir_} {}

ground_truth_t::~ground_truth_t() {}

int ground_truth_load(ground_truth_t &gtd, const std::string &data_dir) {
  // Load ground truth data
  gtd.data_dir = data_dir;
  const std::string gnd_data_path = gtd.data_dir + "/data.csv";
  matx_t data;
  if (csv2mat(gnd_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load ground truth data [%s]!", gnd_data_path.c_str());
    return -1;
  }

  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const long ts = data(i, 0);
    gtd.timestamps.push_back(ts);
    gtd.time.push_back(((double) ts - t0) * 1e-9);
    gtd.p_RS_R.emplace_back(data(i, 1), data(i, 2), data(i, 3));
    gtd.q_RS.emplace_back(data(i, 5), data(i, 6), data(i, 7), data(i, 4));
    gtd.v_RS_R.emplace_back(data(i, 8), data(i, 9), data(i, 10));
    gtd.b_w_RS_S.emplace_back(data(i, 11), data(i, 12), data(i, 13));
    gtd.b_a_RS_S.emplace_back(data(i, 14), data(i, 15), data(i, 16));
  }

  return 0;
}



/*****************************************************************************
 * dataset_event_t
 ****************************************************************************/

dataset_event_t::dataset_event_t() {}

dataset_event_t::dataset_event_t(const vec3_t &a_m, const vec3_t &w_m)
    : type{IMU_EVENT}, a_m{a_m}, w_m{w_m} {}

dataset_event_t::dataset_event_t(const int camera_index,
                                 const std::string &image_path)
    : type{CAMERA_EVENT}, camera_index{camera_index}, image_path{image_path} {}

dataset_event_t::~dataset_event_t() {}



/*****************************************************************************
 * mav_dataset_t
 ****************************************************************************/

mav_dataset_t::mav_dataset_t() {}

mav_dataset_t::mav_dataset_t(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {}

mav_dataset_t::~mav_dataset_t() {}

int mav_dataset_load(mav_dataset_t &ds) {
  // Load IMU data
  const std::string imu_data_dir = ds.data_path + "/imu0";
  if (imu_data_load(ds.imu_data, imu_data_dir) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", ds.imu_data.data_dir.c_str());
    return -1;
  }
  for (size_t i = 0; i < ds.imu_data.timestamps.size(); i++) {
    const long ts = ds.imu_data.timestamps[i];
    const vec3_t a_B = ds.imu_data.a_B[i];
    const vec3_t w_B = ds.imu_data.w_B[i];
    const auto imu_event = dataset_event_t{a_B, w_B};
    ds.timeline.insert({ts, imu_event});
  }

  // Load camera data
  const std::string cam0_data_dir = ds.data_path + "/cam0";
  if (camera_data_load(ds.cam0_data, cam0_data_dir) != 0) {
    LOG_ERROR("Failed to load cam0 data [%s]!", ds.cam0_data.data_dir.c_str());
    return -1;
  }
  const std::string cam1_data_dir = ds.data_path + "/cam1";
  if (camera_data_load(ds.cam1_data, cam1_data_dir) != 0) {
    LOG_ERROR("Failed to load cam1 data [%s]!", ds.cam1_data.data_dir.c_str());
    return -1;
  }
  for (size_t i = 0; i < ds.cam0_data.timestamps.size(); i++) {
    const long ts = ds.cam0_data.timestamps[i];
    const auto cam0_event = dataset_event_t(0, ds.cam0_data.image_paths[i]);
    ds.timeline.insert({ts, cam0_event});
  }
  for (size_t i = 0; i < ds.cam1_data.timestamps.size(); i++) {
    const long ts = ds.cam1_data.timestamps[i];
    const auto cam1_event = dataset_event_t(1, ds.cam1_data.image_paths[i]);
    ds.timeline.insert({ts, cam1_event});
  }
  cv::Mat image = cv::imread(ds.cam0_data.image_paths[0]);
  ds.image_size = cv::Size(image.size());

  // Load ground truth
  const std::string gnd_path = ds.data_path + "/state_groundtruth_estimate0";
  if (ground_truth_load(ds.ground_truth, gnd_path) != 0) {
    LOG_ERROR("Failed to load ground truth!");
    return -1;
  }

  // Process timestamps
  ds.ts_start = mav_dataset_min_timestamp(ds);
  ds.ts_end = mav_dataset_max_timestamp(ds);
  ds.ts_now = ds.ts_start;

  // Get timestamps and calculate relative time
  auto it = ds.timeline.begin();
  auto it_end = ds.timeline.end();
  while (it != it_end) {
    const long ts = it->first;
    ds.timestamps.push_back(ts);
    ds.time[ts] = ((double) ts - ds.ts_start) * 1e-9;

    // Advance to next non-duplicate entry.
    do {
      ++it;
    } while (ts == it->first);
  }

  ds.ok = true;
  return 0;
}

void mav_dataset_reset(mav_dataset_t &ds) {
  ds.ts_start = mav_dataset_min_timestamp(ds);
  ds.ts_end = mav_dataset_max_timestamp(ds);
  ds.ts_now = ds.ts_start;
  ds.time_index = 0;
  ds.imu_index = 0;
  ds.frame_index = 0;
  // ds.feature_tracks.clear();
}

long mav_dataset_min_timestamp(const mav_dataset_t &ds) {
  const long cam0_first_ts = ds.cam0_data.timestamps.front();
  const long imu_first_ts = ds.imu_data.timestamps.front();

  std::vector<long> first_ts{cam0_first_ts, imu_first_ts};
  auto first_result = std::min_element(first_ts.begin(), first_ts.end());
  const long first_ts_index = std::distance(first_ts.begin(), first_result);
  const long min_ts = first_ts[first_ts_index];

  return min_ts;
}

long mav_dataset_max_timestamp(const mav_dataset_t &ds) {
  const long cam0_last_ts = ds.cam0_data.timestamps.back();
  const long imu_last_ts = ds.imu_data.timestamps.back();

  std::vector<long> last_ts{cam0_last_ts, imu_last_ts};
  auto last_result = std::max_element(last_ts.begin(), last_ts.end());
  const long last_ts_index = std::distance(last_ts.begin(), last_result);
  const long max_ts = last_ts[last_ts_index];

  return max_ts;
}



/*****************************************************************************
 * calib_data_t
 ****************************************************************************/

calib_data_t::calib_data_t() {}

calib_data_t::calib_data_t(const std::string &data_path)
    : data_path{strip_end(data_path, "/")} {}

calib_data_t::~calib_data_t() {}

int calib_data_load(calib_data_t &data, const std::string &data_path) {
  // Load IMU data
  const std::string imu_data_dir = data_path + "/mav0/imu0";
  if (imu_data_load(data.imu_data, imu_data_dir) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_dir.c_str());
    return -1;
  }

  // Load cam0 data
  const std::string cam0_dir = data.data_path + "/mav0/cam0";
  if (camera_data_load(data.cam0_data, cam0_dir) != 0) {
    LOG_ERROR("Failed to load cam0 data [%s]!", cam0_dir.c_str());
    return -1;
  }

  // Load cam1 data
  const std::string cam1_dir = data.data_path + "/mav0/cam1";
  if (camera_data_load(data.cam1_data, cam1_dir) != 0) {
    LOG_ERROR("Failed to load cam1 data [%s]!", cam1_dir.c_str());
    return -1;
  }

  // Check if cam0 has same amount of images as cam1
  const size_t cam0_nb_images = data.cam0_data.image_paths.size();
  const size_t cam1_nb_images = data.cam1_data.image_paths.size();
  if (cam0_nb_images != cam1_nb_images) {
    if (cam0_nb_images > cam1_nb_images) {
      data.cam0_data.timestamps.pop_back();
      data.cam0_data.time.pop_back();
      data.cam0_data.image_paths.pop_back();
    } else if (cam0_nb_images < cam1_nb_images) {
      data.cam1_data.timestamps.pop_back();
      data.cam1_data.time.pop_back();
      data.cam1_data.image_paths.pop_back();
    }
  }

  // Get image size
  const cv::Mat image = cv::imread(data.cam0_data.image_paths[0]);
  data.image_size = cv::Size(image.size());

  // Load calibration target data
  const std::string target_path = data.data_path + "/april_6x6.yaml";
  if (calib_target_load(data.calib_target, target_path) != 0) {
    LOG_ERROR("Failed to load target [%s]!", target_path.c_str());
    return -1;
  }

  return 0;
}

}  // namespace euroc
}  // namespace prototype
