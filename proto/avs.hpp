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
struct feature_grid_t {
  int image_width;
  int image_height;
  std::vector<cv::KeyPoint> keypoints;
  int grid_rows;
  int grid_cols;
  std::vector<int> cells;
};

/**
 * Setup feature grid
 */
void feature_grid_setup(
    feature_grid_t *grid,
    const int image_width,
    const int image_height,
    const std::vector<cv::KeyPoint> &keypoints = std::vector<cv::KeyPoint>(),
    const int grid_rows = 3,
    const int grid_cols = 4);

/**
 * Return cell index based on point `pt`
 */
int feature_grid_cell_index(const feature_grid_t *grid, const cv::KeyPoint &kp);

/**
 * Return cell count
 */
int feature_grid_count(const feature_grid_t *grid, const int cell_idx);

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

/**
 * Grid detect
 */
void grid_detect(const cv::Ptr<cv::Feature2D> &detector,
                 const cv::Mat &image,
                 const int max_keypoints = 1000,
                 const int grid_rows = 3,
                 const int grid_cols = 4,
                 const std::vector<cv::KeyPoint> &prev_keypoints =
                     std::vector<cv::KeyPoint>());

// def grid_detect(detector, image, **kwargs):
//   """
//   Detect features uniformly using a grid system.
//   """
//   optflow_mode = kwargs.get('optflow_mode', False)
//   max_keypoints = kwargs.get('max_keypoints', 240)
//   grid_rows = kwargs.get('grid_rows', 3)
//   grid_cols = kwargs.get('grid_cols', 4)
//   prev_kps = kwargs.get('prev_kps', [])
//   if prev_kps is None:
//     prev_kps = []

//   # Calculate number of grid cells and max corners per cell
//   image_height, image_width = image.shape
//   dx = int(math.ceil(float(image_width) / float(grid_cols)))
//   dy = int(math.ceil(float(image_height) / float(grid_rows)))
//   nb_cells = grid_rows * grid_cols
//   max_per_cell = math.floor(max_keypoints / nb_cells)

//   # Detect corners in each grid cell
//   feature_grid = FeatureGrid(grid_rows, grid_cols, image.shape, prev_kps)
//   des_all = []
//   kps_all = []

//   cell_idx = 0
//   for y in range(0, image_height, dy):
//     for x in range(0, image_width, dx):
//       # Make sure roi width and height are not out of bounds
//       w = image_width - x if (x + dx > image_width) else dx
//       h = image_height - y if (y + dy > image_height) else dy

//       # Detect corners in grid cell
//       cs, ce, rs, re = (x, x + w, y, y + h)
//       roi_image = image[rs:re, cs:ce]

//       kps = None
//       des = None
//       if optflow_mode:
//         detector.setNonmaxSuppression(1)
//         kps = detector.detect(roi_image)
//         kps = sort_keypoints(kps)

//       else:
//         kps = detector.detect(roi_image, None)
//         kps, des = detector.compute(roi_image, kps)

//       # Offset keypoints
//       cell_vacancy = max_per_cell - feature_grid.count(cell_idx)
//       if cell_vacancy <= 0:
//         continue

//       limit = min(len(kps), cell_vacancy)
//       for i in range(limit):
//         kp = kps[i]
//         kp.pt = (kp.pt[0] + x, kp.pt[1] + y)
//         kps_all.append(kp)
//         des_all.append(des[i, :] if optflow_mode is False else None)

//       # Update cell_idx
//       cell_idx += 1

//   # Space out the keypoints
//   if optflow_mode:
//     kps_all = spread_keypoints(image, kps_all, 20, prev_kps=prev_kps)

//   # Debug
//   if kwargs.get('debug', False):
//     # Setup
//     viz = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
//     kps_grid = FeatureGrid(grid_rows, grid_cols, image.shape, kps_all)

//     # Visualization properties
//     red = (0, 0, 255)
//     yellow = (0, 255, 255)
//     linetype = cv2.LINE_AA
//     font = cv2.FONT_HERSHEY_SIMPLEX

//     # -- Draw horizontal lines
//     for x in range(0, image_width, dx):
//       cv2.line(viz, (x, 0), (x, image_height), red, 1, linetype)

//     # -- Draw vertical lines
//     for y in range(0, image_height, dy):
//       cv2.line(viz, (0, y), (image_width, y), red, 1, linetype)

//     # -- Draw bin numbers
//     cell_idx = 0
//     for y in range(0, image_height, dy):
//       for x in range(0, image_width, dx):
//         text = str(kps_grid.count(cell_idx))
//         origin = (x + 10, y + 20)
//         viz = cv2.putText(viz, text, origin, font, 0.5, red, 1, linetype)

//         # text = str(feature_grid.count(cell_idx))
//         # origin = (x + 10, y + 20)
//         # viz = cv2.putText(viz, text, origin, font, 0.5, yellow, 1,
//         linetype)

//         cell_idx += 1

//     # -- Draw keypoints
//     viz = draw_keypoints(viz, kps_all, color=red)
//     viz = draw_keypoints(viz, prev_kps, color=yellow)
//     cv2.imshow("viz", viz)
//     cv2.waitKey(0)

//   # Return
//   if optflow_mode:
//     return kps_all

//   return kps_all, np.array(des_all)

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
