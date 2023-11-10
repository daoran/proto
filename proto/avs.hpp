#ifndef AVS_HPP
#define AVS_HPP

#include <map>
#include <vector>

extern "C" {
#include "proto.h"
#include "euroc.h"
}

#include <opencv2/opencv.hpp>

///////////
// UTILS //
///////////

/**
 * Convert cv::Mat type to string
 */
std::string type2str(const int type);

/**
 * Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps);

/**
 * Sort Keypoints
 */
void sort_keypoints(std::vector<cv::KeyPoint> &kps);

/**
 * Given a set of keypoints `kps` make sure they are atleast `min_dist` pixels
 * away from each other, if they are not remove them.
 */
std::vector<cv::KeyPoint> spread_keypoints(
    const cv::Mat &image,
    const std::vector<cv::KeyPoint> &kps,
    const int min_dist = 10,
    const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
    const bool debug = false);

/**
 * Returns number of inliers, outliers and total from inlier vector.
 */
void inlier_stats(const std::vector<uchar> inliers,
                  size_t &num_inliers,
                  size_t &num_outliers,
                  size_t &num_total,
                  float &inlier_ratio);

/**
 * Track keypoints `pts_i` from image `img_i` to image `img_j` using optical
 * flow. Returns a tuple of `(pts_i, pts_j, inliers)` points in image i, j and a
 * vector of inliers.
 */
void optflow_track(const cv::Mat &img_i,
                   const cv::Mat &img_j,
                   const std::vector<cv::KeyPoint> &kps_i,
                   std::vector<cv::KeyPoint> &kps_j,
                   std::vector<uchar> &inliers,
                   const int patch_size = 50,
                   const int max_iter = 50,
                   const int max_level = 3,
                   const real_t epsilon = 0.001,
                   const bool debug = false);

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
void ransac(const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            const undistort_func_t cam0_undist,
            const undistort_func_t cam1_undist,
            const real_t cam0_params[8],
            const real_t cam1_params[8],
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.75,
            const double confidence = 0.99);

/**
 * Filter features by triangulating them via a stereo-pair and see if the
 * reprojection error is reasonable.
 */
void reproj_filter(const project_func_t cam_i_proj_func,
                   const int cam_i_res[2],
                   const real_t *cam_i_params,
                   const real_t *cam_j_params,
                   const real_t T_C0Ci[4 * 4],
                   const real_t T_C0Cj[4 * 4],
                   const std::vector<cv::KeyPoint> &kps_i,
                   const std::vector<cv::KeyPoint> &kps_j,
                   std::vector<cv::Point3d> &points,
                   std::vector<bool> &inliers,
                   const real_t reproj_threshold = 0.5);

//////////////////
// FEATURE GRID //
//////////////////

/**
 * Feature Grid
 *
 * The idea is to take all the feature positions and put them into grid cells
 * across the full image space. This is so that one could keep track of how
 * many feautures are being tracked in each individual grid cell and act
 * accordingly.
 *
 * o-----> x
 * | ---------------------
 * | |  0 |  1 |  2 |  3 |
 * V ---------------------
 * y |  4 |  5 |  6 |  7 |
 *   ---------------------
 *   |  8 |  9 | 10 | 11 |
 *   ---------------------
 *   | 12 | 13 | 14 | 15 |
 *   ---------------------
 *
 *   grid_x = ceil((max(1, pixel_x) / img_w) * grid_cols) - 1.0
 *   grid_y = ceil((max(1, pixel_y) / img_h) * grid_rows) - 1.0
 *   cell_id = int(grid_x + (grid_y * grid_cols))
 *
 */
struct FeatureGrid {
  int image_width;
  int image_height;
  int grid_rows;
  int grid_cols;
  std::vector<int> cells;
  std::vector<std::pair<int, int>> keypoints;

  FeatureGrid(const int image_width,
              const int image_height,
              const int grid_rows = 3,
              const int grid_cols = 4);
  virtual ~FeatureGrid() = default;

  /** Add keypoint */
  void add(const int pixel_x, const int pixel_y);

  /** Return cell index based on point `pt` */
  int cellIndex(const int pixel_x, const int pixel_y) const;

  /** Return cell count */
  int count(const int cell_idx) const;

  /** Debug */
  void debug(const bool imshow = true) const;
};

///////////////////
// GRID DETECTOR //
///////////////////

/**
 * Grid detector
 */
struct GridDetector {
  // cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  cv::Ptr<cv::Feature2D> detector = cv::FastFeatureDetector::create();
  bool optflow_mode = true;
  int max_keypoints = 200;
  int grid_rows = 3;
  int grid_cols = 4;

  GridDetector() = default;
  virtual ~GridDetector() = default;

  /** Detect new keypoints **/
  void detect(
      const cv::Mat &image,
      std::vector<cv::KeyPoint> &kps_new,
      cv::Mat &des_new,
      const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
      bool debug = false) const;
  void detect(
      const cv::Mat &image,
      std::vector<cv::KeyPoint> &kps_new,
      const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
      bool debug = false) const;

  /** Visualize **/
  void visualize(const cv::Mat &image,
                 const FeatureGrid &grid,
                 const std::vector<cv::KeyPoint> &kps_new,
                 const std::vector<cv::KeyPoint> &kps_prev) const;
};

/////////////////////
// FEATURE-TRACKER //
/////////////////////

struct FeatureInfo {
  size_t feature_id = 0;
  std::vector<timestamp_t> timestamps;
  std::map<int, std::vector<cv::KeyPoint>> keypoints;

  FeatureInfo() = default;
  FeatureInfo(const size_t feature_id_,
              const timestamp_t ts_,
              const std::map<int, cv::KeyPoint> &keypoints_)
      : feature_id{feature_id_} {
    timestamps.push_back(ts_);
    for (const auto &[cam_idx, kp] : keypoints_) {
      keypoints[cam_idx].push_back(kp);
    }
  }
  virtual ~FeatureInfo() = default;

  /** Return Feature ID **/
  size_t featureId() const {
    return feature_id;
  }

  /** Return Timestamp **/
  timestamp_t timestamp(const size_t idx) const {
    return timestamps.at(idx);
  }

  /** Return First Timestamp **/
  timestamp_t first_timestamp() const {
    return timestamps.front();
  }

  /** Return Last Timestamp **/
  timestamp_t last_timestamp() const {
    return timestamps.back();
  }

  /** Return Camera Keypoints **/
  std::vector<cv::KeyPoint> get_keypoints(const int cam_idx) const {
    return keypoints.at(cam_idx);
  }

  /** Return Last Camera Keypoints **/
  cv::KeyPoint last_keypoint(const int cam_idx) const {
    return keypoints.at(cam_idx).back();
  }

  /** Update feature with new measurement **/
  void update(const timestamp_t ts, const int cam_idx, const cv::KeyPoint &kp) {
    timestamps.push_back(ts);
    keypoints[cam_idx].push_back(kp);
  }
};

struct FeatureTracker {
  // Flags and counters
  bool init = false;
  size_t feature_counter = 0;

  // Settings
  real_t reproj_threshold = 5.0;

  // Feature detector and matcher
  GridDetector detector;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  // Variables
  std::map<int, camera_params_t> cam_params_map;
  std::map<int, extrinsic_t> cam_exts_map;
  std::vector<std::pair<int, int>> overlaps;

  std::map<int, cv::Mat> prev_imgs;
  std::map<size_t, FeatureInfo> features;

  FeatureTracker();
  virtual ~FeatureTracker() = default;

  /** Add camera **/
  void add_camera(const camera_params_t &cam_params,
                  const extrinsic_t &cam_ext);

  /** Add overlap **/
  void add_overlap(const std::pair<int, int> &overlap);

  /** Detect overlapping keypoints **/
  void detect_overlap(
      const std::map<int, cv::Mat> &mcam_imgs,
      std::map<int, std::vector<cv::KeyPoint>> &mcam_kps_overlap) const;

  /** Track **/
  int track(const timestamp_t ts,
            const std::map<int, cv::Mat> &mcam_imgs,
            const bool debug = false);

  /** Visualize **/
  void visualize();
};

#endif // AVS_HPP
