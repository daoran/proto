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
class FeatureGrid {
public:
  int image_width_;
  int image_height_;
  int grid_rows_;
  int grid_cols_;
  std::vector<int> cells_;
  std::vector<std::pair<int, int>> keypoints_;

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
class GridDetector {
public:
  // cv::Ptr<cv::Feature2D> detector_ = cv::ORB::create();
  // bool optflow_mode_ = false;
  cv::Ptr<cv::Feature2D> detector_ = cv::FastFeatureDetector::create();
  bool optflow_mode_ = true;
  int max_keypoints_ = 200;
  int grid_rows_ = 3;
  int grid_cols_ = 4;

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

  /** Debug **/
  void _debug(const cv::Mat &image,
              const FeatureGrid &grid,
              const std::vector<cv::KeyPoint> &kps_new,
              const std::vector<cv::KeyPoint> &kps_prev) const;
};

/////////////
// TRACKER //
/////////////

class KeyFrame {
public:
  std::map<int, cv::Mat> images_;
  std::map<int, std::vector<cv::KeyPoint>> keypoints_ol_;
  std::map<int, std::vector<cv::KeyPoint>> keypoints_nol_;

  KeyFrame(const std::map<int, cv::Mat> images,
           const std::map<int, std::vector<cv::KeyPoint>> &keypoints_ol,
           const std::map<int, std::vector<cv::KeyPoint>> &keypoints_nol);
  virtual ~KeyFrame() = default;

  /** Debug **/
  void debug() const;
};

class Tracker {
public:
  // Counters
  size_t keypoint_counter_ = 0;

  // Settings
  real_t reproj_threshold_ = 5.0;

  // Feature detector and matcher
  GridDetector detector_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;

  // Variables
  std::map<int, camera_params_t> cam_params_;
  std::map<int, extrinsic_t> cam_exts_;
  std::set<std::pair<int, int>> overlaps_;
  std::unique_ptr<KeyFrame> kf_;
  std::vector<std::unique_ptr<KeyFrame>> old_kfs_;

  Tracker();
  virtual ~Tracker() = default;

  /** Add camera **/
  void addCamera(const camera_params_t &cam_params, const extrinsic_t &cam_ext);

  /** Add overlap **/
  void addOverlap(const std::pair<int, int> &overlap);

  /** Detect overlapping keypoints **/
  void _detectOverlap(
      const std::map<int, cv::Mat> &mcam_imgs,
      std::map<int, std::vector<cv::KeyPoint>> &mcam_kps_overlap) const;

  /** Detect non-overlapping keypoints **/
  void _detectNonOverlap(
      const std::map<int, cv::Mat> &mcam_imgs,
      const std::map<int, std::vector<cv::KeyPoint>> &mcam_kps_overlap,
      std::map<int, std::vector<cv::KeyPoint>> &mcam_kps_nonoverlap) const;

  /** Create New Keyframe **/
  KeyFrame _newKeyFrame(const std::map<int, cv::Mat> &mcam_imgs,
                        const bool debug = false) const;

  /** Track **/
  int track(const std::map<int, cv::Mat> &mcam_imgs, const bool debug = false);
};

#endif // AVS_HPP
