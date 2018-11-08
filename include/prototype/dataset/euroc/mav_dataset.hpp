#ifndef PROTOTYPE_VISION_DATASET_EUROC_MAV_DATASET_HPP
#define PROTOTYPE_VISION_DATASET_EUROC_MAV_DATASET_HPP

#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "prototype/core.hpp"
#include "prototype/dataset/euroc/camera_data.hpp"
#include "prototype/dataset/euroc/ground_truth.hpp"
#include "prototype/dataset/euroc/imu_data.hpp"

namespace prototype {

#define BIND_IMU_CALLBACK(CLASS_METHOD, INSTANCE)                              \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2,                                             \
            std::placeholders::_3);

#define BIND_MONO_CAMERA_CALLBACK(CLASS_METHOD, INSTANCE)                      \
  std::bind(&CLASS_METHOD, &INSTANCE, std::placeholders::_1);

#define BIND_STEREO_CAMERA_CALLBACK(CLASS_METHOD, INSTANCE)                    \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2);

#define BIND_GET_TRACKS_CALLBACK(CLASS_METHOD, INSTANCE)                       \
  std::bind(&CLASS_METHOD, &INSTANCE);

#define BIND_MEASUREMENT_CALLBACK(CLASS_METHOD, INSTANCE)                      \
  std::bind(&CLASS_METHOD, &INSTANCE, std::placeholders::_1);

#define BIND_GET_STATE_CALLBACK(CLASS_METHOD, INSTANCE)                        \
  std::bind(&CLASS_METHOD, &INSTANCE);

#define BIND_RECORD_CALLBACK(CLASS_METHOD, INSTANCE)                           \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2,                                             \
            std::placeholders::_3,                                             \
            std::placeholders::_4);

/**
 * Dataset Event Type
 */
#define NOT_SET 0
#define IMU_EVENT 1
#define CAMERA_EVENT 2

/**
 * Dataset Event
 */
struct dataset_event_t {
  // Event Type
  int type;

  // IMU data
  vec3_t a_m;
  vec3_t w_m;

  // Camera data
  int camera_index;
  std::string image_path;

  dataset_event_t() {}
  dataset_event_t(const vec3_t &a_m, const vec3_t &w_m)
      : type{IMU_EVENT}, a_m{a_m}, w_m{w_m} {}
  dataset_event_t(const int camera_index, const std::string &image_path)
      : type{CAMERA_EVENT}, camera_index{camera_index}, image_path{image_path} {
  }
};

/**
 * dataset_event_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const dataset_event_t &data);

/**
 * EuRoC MAV Dataset
 */
struct mav_dataset_t {
  bool ok = false;
  std::string data_path;

  imu_data_t imu_data;
  camera_data_t cam0_data;
  camera_data_t cam1_data;
  ground_truth_t ground_truth;
  cv::Size image_size;

  long ts_start = 0;
  long ts_end = 0;
  long ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::vector<long> timestamps;
  std::map<long, double> time;
  std::multimap<long, dataset_event_t> timeline;
  // FeatureTracks feature_tracks;

  // clang-format off
  // std::function<vecx_t()> get_state;
  // std::function<int(const cv::Mat &frame)> mono_camera_cb;
  // std::function<int(const cv::Mat &frame0, const cv::Mat &frame1)> stereo_camera_cb;
  // std::function<std::vector<FeatureTrack>()> get_tracks_cb;
  // std::function<int(const vec3_t &a_m, const vec3_t &w_m, const long ts)> imu_cb;
  // std::function<int(const FeatureTracks &tracks)> mea_cb;
  // std::function<int(const double time, const vec3_t &p_G, const vec3_t &v_G, const vec3_t &rpy_G)> record_cb;
  // clang-format on

  mav_dataset_t() {}
  mav_dataset_t(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {}
};

/**
 * Load dataset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
int mav_dataset_load(mav_dataset_t &ds);

/**
 * Reset
 *
 * @param[in,out] ds Dataset
 * @returns 0 for success, -1 for failure
 */
void mav_dataset_reset(mav_dataset_t &ds);

/**
 * Return min timestamp
 *
 * @param[in] ds Dataset
 * @returns Minimum timestamp
 */
long mav_dataset_min_timestamp(const mav_dataset_t &ds);

/**
 * Return max timestamp
 * @returns Maximum timestamp
 */
long mav_dataset_max_timestamp(const mav_dataset_t &ds);

// /**
//  * Step
//  *
//  * @returns
//  * - 0 for success
//  * - -2 for imu callback failure
//  * - -3 for camera callback failure
//  */
// int step();
//
// /**
//  * Run
//  *
//  * @returns
//  * - 0 for success
//  * - -2 for imu callback failure
//  * - -3 for camera callback failure
//  */
// int run();

} //  namespace prototype
#endif // PROTOTYPE_VISION_DATASET_EUROC_MAV_DATASET_HPP
