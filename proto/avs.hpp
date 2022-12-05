#ifndef AVS_HPP
#define AVS_HPP

extern "C" {
#include "proto.h"
#include "euroc.h"
} // extern C

#include <vector>
#include <opencv2/opencv.hpp>

///////////
// UTILS //
///////////

/** Macro that adds the ability to switch between C / C++ style mallocs */
#ifdef __cplusplus

#ifndef MALLOC
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#endif

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
#endif

#endif

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
 */
typedef struct feature_grid_t {
  int image_width;
  int image_height;
  int grid_rows;
  int grid_cols;
  std::vector<int> cells;
  std::vector<std::pair<int, int>> keypoints;
} feature_grid_t;

/**
 * Setup feature grid
 */
void feature_grid_setup(feature_grid_t &grid,
                        const int image_width,
                        const int image_height,
                        const int grid_rows = 3,
                        const int grid_cols = 4);

/**
 * Add keypoint to feature grid
 */
void feature_grid_add(feature_grid_t &grid,
                      const int pixel_x,
                      const int pixel_y);

/**
 * Return cell index based on point `pt`
 */
int feature_grid_cell_index(const feature_grid_t &grid,
                            const int pixel_x,
                            const int pixel_y);

/**
 * Return cell count
 */
int feature_grid_count(const feature_grid_t &grid, const int cell_idx);

/**
 * Debug
 */
void feature_grid_debug(const feature_grid_t &grid, const bool imshow = true);

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
std::vector<int>
spread_keypoints(const cv::Mat &image,
                 const std::vector<cv::KeyPoint> &keypoints,
                 const int min_dist,
                 const std::vector<cv::KeyPoint> prev_keypoints,
                 const bool debug = false);

typedef struct grid_detector_t {
  cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  int max_keypoints = 200;
  int grid_rows = 3;
  int grid_cols = 4;
} grid_detector_t;

/**
 * Grid detect
 */
void grid_detector_detect(const grid_detector_t &detector,
                          const cv::Mat &image,
                          const std::vector<cv::KeyPoint> &prev_kps,
                          std::vector<cv::KeyPoint> &kps,
                          cv::Mat &des);

///////////////
// FRONT-END //
///////////////

class FrontEnd {
public:
  cv::Ptr<cv::FeatureDetector> detector_ = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor_ = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher_ =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::KeyPoint> kps0_;
  std::vector<cv::KeyPoint> kps1_;
  cv::Mat des0_;
  cv::Mat des1_;

  FrontEnd() = default;
  ~FrontEnd() = default;

  void detect(const cv::Mat &img0,
              const cv::Mat &img1,
              const bool debug = false);
};

#endif // AVS_HPP
