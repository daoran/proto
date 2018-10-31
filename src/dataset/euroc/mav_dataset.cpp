#include "prototype/dataset/euroc/mav_dataset.hpp"

namespace prototype {

std::ostream &operator<<(std::ostream &os, const dataset_event_t &data) {
  if (data.type == IMU_EVENT) {
    os << "event_type: imu" << std::endl;
    os << "a_m: " << data.a_m.transpose() << std::endl;
    os << "w_m: " << data.w_m.transpose() << std::endl;
  } else if (data.type == CAMERA_EVENT) {
    os << "event_type: camera" << std::endl;
    os << "camera_index: " << data.camera_index << std::endl;
    os << "image_path: " << data.image_path << std::endl;
  }

  return os;
}

int mav_dataset_load(mav_dataset_t &ds) {
  // Load IMU data
  ds.imu_data = imu_data_t{ds.data_path + "/imu0"};
  if (imu_data_load(ds.imu_data) != 0) {
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
  ds.cam0_data = camera_data_t{ds.data_path + "/cam0"};
  if (camera_data_load(ds.cam0_data) != 0) {
    LOG_ERROR("Failed to load cam0 data [%s]!", ds.cam0_data.data_dir.c_str());
    return -1;
  }
  ds.cam1_data = camera_data_t{ds.data_path + "/cam1"};
  if (camera_data_load(ds.cam1_data) != 0) {
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
  ds.ground_truth =
      ground_truth_t{ds.data_path + "/state_groundtruth_estimate0"};
  if (ground_truth_load(ds.ground_truth) != 0) {
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

// int mav_dataset_step() {
//   bool imu_event = false;
//   bool cam0_event = false;
//   bool cam1_event = false;
//
//   vec3_t a_m{0.0, 0.0, 0.0};
//   vec3_t w_m{0.0, 0.0, 0.0};
//   std::string cam0_image_path;
//   std::string cam1_image_path;
//
//   // Loop through events at a specific timestamp
//   // and get the imu or camera data
//   auto it = this->timeline.lower_bound(this->ts_now);
//   auto it_end = this->timeline.upper_bound(this->ts_now);
//   while (it != it_end) {
//     data_event_t event = it->second;
//     if (event.type == IMU_EVENT) {
//       imu_event = true;
//       a_m = event.a_m;
//       w_m = event.w_m;
//     } else if (event.camera_index == 0) {
//       cam0_event = true;
//       cam0_image_path = event.image_path;
//     } else if (event.camera_index == 1) {
//       cam1_event = true;
//       cam1_image_path = event.image_path;
//     }
//
//     it++;
//   }
//
//   // Trigger imu callback
//   if (imu_event && this->imu_cb != nullptr) {
//     // clang-format off
//     mat3_t R_body_imu;
//     R_body_imu << 0.0, 0.0, 1.0,
//                   0.0, -1.0, 0.0,
//                   1.0, 0.0, 0.0;
//     // clang-format on
//     a_m = R_body_imu * a_m;
//     w_m = R_body_imu * w_m;
//
//     if (this->imu_cb(a_m, w_m, this->ts_now) != 0) {
//       LOG_ERROR("IMU callback failed! Stopping MAVDataset!");
//       return -2;
//     }
//     this->imu_index++;
//   }
//
//   // Trigger camera and measurement callback
//   if (cam0_event && cam1_event) {
//     // Camera callback
//     if (this->mono_camera_cb != nullptr) {
//       const cv::Mat frame = cv::imread(cam0_image_path);
//       if (this->mono_camera_cb(frame) != 0) {
//         LOG_ERROR("Mono camera callback failed! Stopping MAVDataset!");
//         return -3;
//       }
//     } else if (this->stereo_camera_cb != nullptr) {
//       const cv::Mat frame0 = cv::imread(cam0_image_path);
//       const cv::Mat frame1 = cv::imread(cam1_image_path);
//       if (this->stereo_camera_cb(frame0, frame1) != 0) {
//         LOG_ERROR("Stereo camera callback failed! Stopping MAVDataset!");
//         return -3;
//       }
//     }
//
//     // Measurement callback
//     if (this->get_tracks_cb != nullptr) {
//       FeatureTracks tracks = this->get_tracks_cb();
//       this->feature_tracks.insert(std::end(this->feature_tracks),
//                                   std::begin(tracks),
//                                   std::end(tracks));
//
//       if (this->mea_cb != nullptr) {
//         if (this->mea_cb(tracks) != 0) {
//           LOG_ERROR("Measurement callback failed! Stopping MAVDataset!");
//           return -2;
//         }
//       }
//     }
//
//     this->frame_index++;
//   }
//
//   // Trigger record estimate callback
//   if (this->record_cb != nullptr && this->get_state != nullptr) {
//     const vecx_t state = this->get_state();
//     this->record_cb(this->time[this->ts_now],
//                     state.segment(0, 3),
//                     state.segment(3, 3),
//                     state.segment(6, 3));
//   }
//
//   // Update timestamp
//   this->time_index++;
//   this->ts_now = this->timestamps[this->time_index];
//
//   return 0;
// }
//
// int mav_dataset_run() {
//   this->reset();
//
//   for (size_t i = 0; i < this->timestamps.size(); i++) {
//     // for (size_t i = 0; i < 600; i++) {
//     const int retval = this->step();
//     if (retval != 0) {
//       return retval;
//     }
//   }
//   return 0;
// }

} //  namespace prototype
