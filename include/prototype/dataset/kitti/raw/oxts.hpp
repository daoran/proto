/**
 * @file
 * @defgroup kitti kitti
 * @ingroup dataset
 */
#ifndef PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP
#define PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP

#include <algorithm>
#include <vector>
#include <string>
#include <chrono>

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
};

/**
  * Load OXTS entry
  *
  * @param file_path File path to a single OXTS data
  * @returns 0 for success, -1 for failure
  */
int oxts_entry_load(oxts_entry_t &entry, const std::string &file_path);

/**
 * OXTS
 */
struct oxts_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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


};

/**
 * load OXTS
 *
 * @param oxts OXTS dataset
 * @param oxts_dir Path to OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_entries(oxts_t &oxts, const std::string &oxts_dir);

/**
 * Load timestamps
 *
 * @param oxts OXTS dataset
 * @param oxts_dir Path to OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load_timestamps(oxts_t &oxts, const std::string &oxts_dir);

/**
 * Load OXTS
 *
 * @param oxts OXTS dataset
 * @param oxts_dir Path to OXTS data
 * @returns 0 for success, -1 for failure
 */
int oxts_load(oxts_t &oxts, const std::string &oxts_dir);

/** @} group kitti */
} //  namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_RAW_OXTS_HPP
