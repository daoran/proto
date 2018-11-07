/**
 * @file
 * @defgroup calib calib
 */
#ifndef PROTOTYPE_CALIB_APRILGRID_HPP
#define PROTOTYPE_CALIB_APRILGRID_HPP

// Order matters with the AprilTags lib. The detector has to be first.
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

#include "prototype/core.hpp"

namespace prototype {

/**
 * Comparator to sort detected AprilTags by id
 */
static bool sort_apriltag_by_id(const AprilTags::TagDetection &a,
                                const AprilTags::TagDetection &b) {
  return (a.id < b.id);
}

/**
 * AprilGrid detection
 */
struct aprilgrid_t {
  AprilTags::TagDetector detector =
      AprilTags::TagDetector(AprilTags::tagCodes36h11);

  bool configured = false;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  bool detected = false;
  int nb_detections = 0;
  long timestamp = 0;
  std::vector<int> ids;
  std::vector<vec2_t> keypoints;

  bool estimated = false;
  std::vector<vec3_t> points_CF;
  mat4_t T_CF = I(4);
  vec3_t rvec_CF = zeros(3, 1);
  vec3_t tvec_CF = zeros(3, 1);

  aprilgrid_t();
  aprilgrid_t(const long timestamp,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing);
  ~aprilgrid_t();
};

/**
 * AprilGrid detections
 */
typedef std::vector<aprilgrid_t> aprilgrids_t;

/**
 * aprilgrid_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const aprilgrid_t &april_grid);

/**
 * Add measurement
 *
 * @param[in,out] grid AprilGrid
 * @param[in] id Tag id
 * @param[in] keypoints Keypoints
 */
void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints);

/**
 * Get measurements based on tag id
 *
 * @param[in] grid AprilGrid
 * @param[in] id Tag id
 * @param[out] keypoints Keypoints
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  std::vector<vec2_t> &keypoints);

/**
 * Get measurements based on tag id
 *
 * @param[in] grid AprilGrid
 * @param[in] id Tag id
 * @param[out] keypoints Keypoints
 * @param[out] points_CF Positions from camera to fiducial target
 * @param[out] T_CF Transform between fiducial target and camera
 *
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  std::vector<vec2_t> &keypoints,
                  std::vector<vec3_t> &points);

/**
 * Get the tag's grid index using the tag id
 *
 * Note: The origin of the grid is bottom left corner rather than top left.
 *
 * @param[in] id Tag id
 * @param[out] i i-th row
 * @param[out] j j-th column
 *
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_grid_index(const aprilgrid_t &grid, const int id, int &i, int &j);

/**
 * Calculate object point
 *
 * @param[in] grid AprilGrid
 * @param[in] tag_id Tag id
 * @param[in] corner_id Corner id
 * @param[out] object_point Object point
 *
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_object_point(const aprilgrid_t &grid,
                           const int tag_id,
                           const int corner_id,
                           vec3_t &object_point);

/**
 * Calculate object points
 *
 * @param[in] grid AprilGrid
 * @param[in] tag_id Tag id
 * @param[out] object_points Object points
 *
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_object_points(const aprilgrid_t &grid,
                            const int tag_id,
                            std::vector<vec3_t> &object_points);

/**
 * Calculate relative position between AprilGrid and camera using solvepnp
 *
 * @param[in] cam_K Camera intrinsics matrix K
 * @param[in] cam_D Camera distortion vector D
 *
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_calc_relative_pose(aprilgrid_t &grid,
                                 const mat3_t &cam_K,
                                 const vec4_t &cam_D);

/**
 * Show detection
 *
 * @param[in] grid AprilGrid
 * @param[in] image Input image
 * @param[in] tags Detected AprilTags
 */
void aprilgrid_imshow(const aprilgrid_t &grid,
                      const std::string &title,
                      const cv::Mat &image);

/**
 * Save AprilGrid detection
 *
 * @param[in] save_path Path to save detection
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_save(const aprilgrid_t &grid, const std::string &save_path);

/**
 * Load AprilGrid detection
 *
 * @param[in,out] grid AprilGrid
 * @param[in] data_path Path to data file
 * @returns 0 or -1 for success or failure
 */
int aprilgrid_load(aprilgrid_t &grid, const std::string &data_path);

/**
 * Configure AprilGrid detector
 *
 * @param[in,out] grid AprilGrid
 * @param[in] config_file Path to config file
 * @returns 0 or 1 for success or failure
 */
int aprilgrid_configure(aprilgrid_t &grid,
                        const std::string &config_file);

/**
 * Filter tags detected
 *
 * @param[in] image Image
 * @param[in,out] tags Detected AprilTags
 */
void aprilgrid_filter_tags(const cv::Mat &image,
                           std::vector<AprilTags::TagDetection> &tags);

/**
 * Detect AprilGrid
 *
 * @param[in,out] grid AprilGrid
 * @param[in] image Input image
 * @returns Number of AprilTags detected
 */
int aprilgrid_detect(aprilgrid_t &detector, const cv::Mat &image);

/**
 * Detect AprilGrid
 *
 * @param[in,out] grid AprilGrid
 * @param[in] image Input image
 * @param[in] cam_K Camera intrinsics matrix K
 * @param[in] cam_D Camera distortion vector D
 *
 * @returns Number of AprilTags detected
 */
int aprilgrid_detect(aprilgrid_t &grid,
                     const cv::Mat &image,
                     const mat3_t &cam_K,
                     const vec4_t &cam_D);

} // namespace prototype
#endif // PROTOTYPE_CALIB_APRILGRID_HPP
