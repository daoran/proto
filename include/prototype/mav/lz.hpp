#ifndef PROTOTYPE_MAV_LZ_HPP
#define PROTOTYPE_MAV_LZ_HPP

#include <map>

// Order matters with the AprilTags lib. The detector has to be first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include "prototype/core/core.hpp"
#include "prototype/vision/camera/pinhole.hpp"

namespace proto {

/**
 * Landing zone
 */
struct lz_t {
  bool detected = false;
  mat4_t T_BC = I(4);
  mat4_t T_CZ = I(4);

  lz_t() {}
  ~lz_t() {}
  lz_t(const bool detected_, const mat4_t &T_BC_, const mat4_t &T_CZ_)
    : detected{detected_}, T_BC{T_BC_}, T_CZ{T_CZ_} {}
};

/**
 * Landing zone detector
 */
struct lz_detector_t {
  bool ok = false;

  AprilTags::TagDetector *det = nullptr;
  std::map<int, double> targets;

  lz_detector_t();
  lz_detector_t(const std::vector<int> &tag_ids,
                const std::vector<double> &tag_sizes);
  lz_detector_t(const std::string &config,
                const std::string &prefix="");
  ~lz_detector_t();
};

/**
 * Print landing zone
 */
void lz_print(const lz_t &lz);

/**
 * Configure landing zone
 */
int lz_detector_configure(lz_detector_t &lz,
                          const std::vector<int> &tag_ids,
                          const std::vector<double> &tag_sizes);

/**
 * Configure landing zone
 */
int lz_detector_configure(lz_detector_t &lz,
                          const std::string &config_file,
                          const std::string &prefix="");

/**
 * Detect landing zone
 */
int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       mat4_t &T_CZ);

/**
 * Detect landing zone
 */
int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       const mat4_t &T_BC,
                       lz_t &lz);

/**
 * Calculate landing zone corners in pixels.
 */
int lz_detector_calc_corners(const lz_detector_t &lz,
										         const pinhole_t &pinhole,
										         const cv::Mat &image,
                             const mat4_t &T_CZ,
										         const int tag_id,
                             const double padding,
                             vec2_t &top_left,
                             vec2_t &btm_right);

} //  namespace proto
#endif // PROTOTYPE_MAV_LZ_HPP
