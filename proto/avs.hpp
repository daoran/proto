#ifndef AVS_HPP
#define AVS_HPP

#include <map>
#include <vector>

extern "C" {
#include "proto.h"
#include "euroc.h"
} // extern C

#include <opencv2/opencv.hpp>

///////////
// UTILS //
///////////

namespace cv {

/**
 * Convert cv::Mat type to string
 */
std::string type2str(const int type);

/**
 * Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps);

} // namespace cv

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
std::vector<uchar> ransac(const std::vector<cv::KeyPoint> &kps0,
                          const std::vector<cv::KeyPoint> &kps1,
                          const undistort_func_t cam0_undist,
                          const undistort_func_t cam1_undist,
                          const real_t cam0_params[8],
                          const real_t cam1_params[8],
                          const double reproj_threshold = 0.75,
                          const double confidence = 0.99);

//////////////////
// FEATURE GRID //
//////////////////

/**
 * Feature Grid
 *
 * The idea is to take all the feature positions and put them into grid cells
 * across the full image space. This is so that one could keep track of how many
 * feautures are being tracked in each individual grid cell and act accordingly.
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
 * Grid detector
 */
class GridDetector {
public:
  cv::Ptr<cv::Feature2D> detector_ = cv::ORB::create();
  int max_keypoints_ = 200;
  int grid_rows_ = 3;
  int grid_cols_ = 4;

  GridDetector() = default;
  virtual ~GridDetector() = default;

  /** Detect new keypoints **/
  void detect(const cv::Mat &image,
              const std::vector<cv::KeyPoint> &prev_kps,
              std::vector<cv::KeyPoint> &kps,
              cv::Mat &des,
              bool debug = false);
};

///////////////
// FRONT-END //
///////////////

class KeyFrame {
public:
  std::map<int, std::vector<cv::KeyPoint>> keypoints;
  std::map<int, cv::Mat> descriptors;
  pose_t pose;

  KeyFrame();
  virtual ~KeyFrame() = default;

  /** Return camera keypoints **/
  void data(const int cam_idx,
            std::vector<cv::KeyPoint> &kps,
            cv::Mat &des) const;
};

class FrontEnd {
public:
  GridDetector detector_;
  cv::Ptr<cv::DescriptorMatcher> matcher_ =
      cv::DescriptorMatcher::create("BruteForce-Hamming");

  std::vector<cv::KeyPoint> kps0_;
  std::vector<cv::KeyPoint> kps1_;
  cv::Mat des0_;
  cv::Mat des1_;

  std::map<int, camera_params_t> cam_params_;
  std::map<int, extrinsic_t> cam_exts_;

  FrontEnd() = default;
  virtual ~FrontEnd() = default;

  /** Add camera **/
  void addCamera(const camera_params_t &cam_params, const extrinsic_t &cam_ext);

  /** Reprojection Filter **/
  void _reprojFilter(const int idx_i,
                     const int idx_j,
                     const std::vector<cv::KeyPoint> &kps_i,
                     const std::vector<cv::KeyPoint> &kps_j);

  /**
   * Detect new keypoints
   */
  void detect(const cv::Mat &img0,
              const cv::Mat &img1,
              const bool debug = false);
};

#endif // AVS_HPP
