/**
 * @file
 * @defgroup kitti kitti
 * @ingroup dataset
 */
#ifndef PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP
#define PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "prototype/core.hpp"
#include "prototype/dataset/kitti/raw/parse.hpp"

namespace prototype {
/**
 * @addtogroup kitti
 * @{
 */

/**
 * OXTS entry
 */
struct oxts_entry_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string file_path;

  vec3_t gps = zeros(3, 1);
  vec3_t rpy = zeros(3, 1);
  vec3_t v_G = zeros(3, 1);
  vec3_t v_B = zeros(3, 1);
  vec3_t a_G = zeros(3, 1);
  vec3_t a_B = zeros(3, 1);
  vec3_t w_G = zeros(3, 1);
  vec3_t w_B = zeros(3, 1);
  double pos_accuracy = 0.0;
  double vel_accuracy = 0.0;

  oxts_entry_t() {}
  oxts_entry_t(const std::string &file_path_) : file_path{file_path_} {}
};

/**
 * Load OXTS entry
 *
 * @param[in,out] entry OXTS entry
 * @returns 0 for success, -1 for failure
 */
int oxts_entry_load(oxts_entry_t &entry);

/**
 * OXTS
 */
struct oxts_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string oxts_dir;

  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> gps;
  std::vector<vec3_t> rpy;
  std::vector<vec3_t> p_G;
  std::vector<vec3_t> v_G;
  std::vector<vec3_t> v_B;
  std::vector<vec3_t> a_G;
  std::vector<vec3_t> a_B;
  std::vector<vec3_t> w_G;
  std::vector<vec3_t> w_B;
  std::vector<double> pos_accuracy;
  std::vector<double> vel_accuracy;

  oxts_t() {}
  oxts_t(const std::string &oxts_dir_) : oxts_dir{oxts_dir_} {}
};

/**
 * load OXTS
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_entries(oxts_t &oxts);

/**
 * Load timestamps
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_timestamps(oxts_t &oxts);

/**
 * Load OXTS
 *
 * @param[in,out] oxts OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load(oxts_t &oxts);

/** @} group kitti */
} //  namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP
