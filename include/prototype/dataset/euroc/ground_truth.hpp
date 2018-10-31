/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef PROTOTYPE_VISION_DATASET_EUROC_GROUND_TRUTH_HPP
#define PROTOTYPE_VISION_DATASET_EUROC_GROUND_TRUTH_HPP

#include "prototype/core.hpp"
#include <vector>

namespace prototype {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * Ground truth data
 */
struct ground_truth_t {
  std::string data_dir;

  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<vec3_t> p_RS_R;
  std::vector<vec4_t> q_RS;
  std::vector<vec3_t> v_RS_R;
  std::vector<vec3_t> b_w_RS_S;
  std::vector<vec3_t> b_a_RS_S;

  ground_truth_t() {}
  ground_truth_t(const std::string data_dir_) : data_dir{data_dir_} {}
};

/**
 * Load ground truth data
 *
 * @param[int,out] data Ground truth data
 * @returns 0 for success, -1 for failure
 */
int ground_truth_load(ground_truth_t &data);

/** @} group euroc */
} //  namespace prototype
#endif // PROTOTYPE_VISION_DATASET_EUROC_GROUND_TRUTH_HPP
