#ifndef AVS_HPP
#define AVS_HPP

#include <map>
#include <vector>
#include <algorithm>
#include <ceres/ceres.h>

extern "C" {
#include "proto.h"
#include "euroc.h"
}

#include <opencv2/opencv.hpp>

/******************************************************************************
 * COMPUTER-VISION
 *****************************************************************************/

///////////
// UTILS //
///////////

/**
 * Extend a std::vector with another
 */
template <typename T>
void extend_vector(std::vector<T> &destination, const std::vector<T> &source) {
  destination.insert(destination.end(), source.begin(), source.end());
}

/**
 * Returns mean
 */
real_t mean(const std::vector<real_t> &x) {
  real_t sum = 0.0;
  for (const auto i : x) {
    sum += i;
  }

  const real_t N = x.size();
  return sum / N;
}

/**
 * Returns median
 */
real_t median(const std::vector<real_t> &v) {
  // sort values
  std::vector<real_t> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const real_t a = v_copy[v_copy.size() / 2];
    const real_t b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

/**
 * Returns RMSE
 */
real_t rmse(const std::vector<real_t> &residuals) {
  real_t sse = 0.0;
  for (const auto r : residuals) {
    sse += r * r;
  }

  real_t n = residuals.size();
  real_t mse = sse / n;
  return sqrt(mse);
}

/**
 * Returns variance
 */
real_t var(const std::vector<real_t> &x) {
  const double mu = mean(x);
  const double N = x.size();

  double sum = 0.0;
  for (const auto x_i : x) {
    sum += pow(x_i - mu, 2);
  }

  return sum / (N - 1.0);
}

/**
 * Convert cv::Mat type to string
 */
std::string type2str(const int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

/**
 * Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps) {
  std::vector<cv::Point2f> pts;
  for (const auto &kp : kps) {
    pts.push_back(kp.pt);
  }
  return pts;
}

/**
 * Print Keypoint
 */
void print_keypoint(const cv::KeyPoint &kp) {
  printf("angle: %f\n", kp.angle);
  printf("class_id: %d\n", kp.class_id);
  printf("octave: %d\n", kp.octave);
  printf("pt: [%.2f, %.2f]\n", kp.pt.x, kp.pt.y);
  printf("response: %f\n", kp.response);
  printf("size: %f\n", kp.size);
}

/**
 * Sort Keypoints
 */
void sort_keypoints(std::vector<cv::KeyPoint> &kps) {
  std::sort(kps.begin(), kps.end(), [](cv::KeyPoint a, cv::KeyPoint b) {
    return a.response > b.response;
  });
}

/**
 * Given a set of keypoints `kps` make sure they are atleast `min_dist` pixels
 * away from each other, if they are not remove them.
 */
std::vector<cv::KeyPoint> spread_keypoints(
    const cv::Mat &image,
    const std::vector<cv::KeyPoint> &kps,
    const int min_dist = 10,
    const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
    const bool debug = false) {
  // Pre-check
  std::vector<cv::KeyPoint> kps_filtered;
  if (kps.size() == 0) {
    return kps_filtered;
  }

  // Setup
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  cv::Mat A = cv::Mat::zeros(img_h, img_w, CV_8UC1);

  // Loop through previous keypoints
  for (const auto kp : kps_prev) {
    // Fill the area of the matrix where the next keypoint cannot be around
    const int px = kp.pt.x;
    const int py = kp.pt.y;
    const int rs = std::max(py - min_dist, 0);
    const int re = std::min(py + min_dist + 1, img_h - 1);
    const int cs = std::max(px - min_dist, 0);
    const int ce = std::min(px + min_dist + 1, img_w - 1);

    for (size_t i = rs; i < re; i++) {
      for (size_t j = cs; j < ce; j++) {
        A.at<uint8_t>(i, j) = 255;
      }
    }
  }

  // Loop through keypoints
  for (const auto kp : kps) {
    // Check if point is ok to be added to results
    const int px = kp.pt.x;
    const int py = kp.pt.y;
    if (A.at<uint8_t>(py, px) > 0) {
      continue;
    }
    kps_filtered.push_back(kp);

    // Fill the area of the matrix where the next keypoint cannot be around
    const int rs = std::max(py - min_dist, 0);
    const int re = std::min(py + min_dist + 1, img_h - 1);
    const int cs = std::max(px - min_dist, 0);
    const int ce = std::min(px + min_dist + 1, img_w - 1);

    for (size_t i = rs; i < re; i++) {
      for (size_t j = cs; j < ce; j++) {
        A.at<uint8_t>(i, j) = 255;
      }
    }
  }

  // Debug
  if (debug) {
    cv::imshow("Debug", A);
    cv::waitKey(0);
  }

  return kps_filtered;
}

/**
 * Returns number of inliers, outliers and total from inlier vector.
 */
void inlier_stats(const std::vector<uchar> inliers,
                  size_t &num_inliers,
                  size_t &num_outliers,
                  size_t &num_total,
                  float &inlier_ratio) {
  num_inliers = 0;
  num_outliers = 0;
  num_total = inliers.size();
  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i]) {
      num_inliers++;
    } else {
      num_outliers++;
    }
  }

  inlier_ratio = (float) num_inliers / (float) num_total;
}

/**
 * Filter Outliers
 */
void filter_outliers(std::vector<cv::KeyPoint> &kps_i,
                     std::vector<cv::KeyPoint> &kps_j,
                     const std::vector<uchar> &inliers) {
  std::vector<cv::KeyPoint> a;
  std::vector<cv::KeyPoint> b;

  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i]) {
      a.push_back(kps_i[i]);
      b.push_back(kps_j[i]);
    }
  }
  kps_i = a;
  kps_j = b;
}

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
                   const bool debug = false) {
  // Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
  std::vector<cv::Point2f> pts_i = kps2pts(kps_i);
  std::vector<cv::Point2f> pts_j = kps2pts(kps_j);

  // Track features from img i to img j
  const auto win_size = cv::Size(patch_size, patch_size);
  const auto crit_type = cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS;
  const auto crit = cv::TermCriteria(crit_type, max_iter, epsilon);
  const auto errs = cv::noArray();
  std::vector<uchar> optflow_inliers;
  cv::calcOpticalFlowPyrLK(img_i,
                           img_j,
                           pts_i,
                           pts_j,
                           optflow_inliers,
                           errs,
                           win_size,
                           max_level,
                           crit);

  // Make sure keypoints are within image dimensions
  std::vector<bool> bound_inliers;
  const int img_h = img_i.rows;
  const int img_w = img_i.cols;
  for (const auto &p : pts_j) {
    const auto x_ok = p.x >= 0 && p.x <= img_w;
    const auto y_ok = p.y >= 0 && p.y <= img_h;
    bound_inliers.push_back((x_ok && y_ok));
  }

  // Write out final inliers
  assert(pts_i.size() == optflow_inliers.size());
  assert(pts_i.size() == bound_inliers.size());

  kps_j.clear();
  inliers.clear();
  for (size_t i = 0; i < pts_i.size(); i++) {
    kps_j.emplace_back(pts_j[i],
                       kps_i[i].size,
                       kps_i[i].angle,
                       kps_i[i].response,
                       kps_i[i].octave,
                       kps_i[i].class_id);
    inliers.push_back(optflow_inliers[i] && bound_inliers[i]);
  }

  // Debug
  bool quit = false;
  while (debug && quit == false) {
    // Draw properties
    const int radius = 2;
    const cv::Scalar red{0, 0, 255};

    // Setup
    cv::Mat viz_i;
    cv::Mat viz_j;
    cv::cvtColor(img_i, viz_i, cv::COLOR_GRAY2RGB);
    cv::cvtColor(img_j, viz_j, cv::COLOR_GRAY2RGB);

    // Draw KeyPoints
    for (size_t n = 0; n < kps_i.size(); n++) {
      if (inliers[n] == 0) {
        continue;
      }

      auto pt_i = kps_i[n].pt;
      auto pt_j = kps_j[n].pt;
      cv::circle(viz_i, pt_i, radius, red, -1);
      cv::circle(viz_j, pt_j, radius, red, -1);
    }

    // Stitch images horizontally
    cv::Mat viz;
    cv::hconcat(viz_i, viz_j, viz);

    // Draw lines
    for (size_t n = 0; n < kps_i.size(); n++) {
      auto pt_i = kps_i[n].pt;
      auto pt_j = kps_j[n].pt;
      pt_j.x += viz_i.cols;
      const auto line = cv::LINE_AA;
      cv::line(viz, pt_i, pt_j, red, 1, line);
    }

    // Show
    cv::imshow("Optical-Flow Tracking", viz);
    if (cv::waitKey(0) == 'q') {
      quit = true;
    }
  }
}

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
            const double reproj_threshold = 0.5,
            const double confidence = 0.99) {
  assert(kps0.size() > 0 && kps1.size() > 0);
  assert(kps0.size() == kps1.size());
  assert(cam0_undist && cam1_undist);
  assert(cam0_params && cam1_params);

  // Undistort image points
  std::vector<cv::Point2f> pts0 = kps2pts(kps0);
  std::vector<cv::Point2f> pts1 = kps2pts(kps1);
  for (size_t i = 0; i < pts0.size(); i++) {
    const real_t z0_in[2] = {pts0[i].x, pts0[i].y};
    const real_t z1_in[2] = {pts1[i].x, pts1[i].y};

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    cam0_undist(cam0_params, z0_in, z0);
    cam1_undist(cam1_params, z1_in, z1);

    pts0[i].x = z0[0];
    pts0[i].y = z0[1];

    pts1[i].x = z1[0];
    pts1[i].y = z1[1];
  }

  // Perform ransac
  const int method = cv::FM_RANSAC;
  cv::findFundamentalMat(pts0,
                         pts1,
                         method,
                         reproj_threshold,
                         confidence,
                         inliers);
}

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
void ransac(const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            const undistort_func_t undist_func,
            const real_t cam_params[8],
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.75,
            const double confidence = 0.99) {
  ransac(kps0,
         kps1,
         undist_func,
         undist_func,
         cam_params,
         cam_params,
         inliers,
         reproj_threshold,
         confidence);
}

/**
 * Check parallax
 */
void check_parallax(const camera_params_t &cam0_params,
                    const camera_params_t &cam1_params,
                    const extrinsic_t &cam0_ext,
                    const extrinsic_t &cam1_ext,
                    const std::vector<cv::KeyPoint> &kps0,
                    const std::vector<cv::KeyPoint> &kps1,
                    const real_t parallax_threshold,
                    std::vector<uchar> &inliers) {
  assert(parallax_threshold > 0);
  assert(kps0.size() == kps1.size());

  // Form projection matrices P_i and P_j
  const real_t *params0 = cam0_params.data;
  const real_t *params1 = cam1_params.data;

  real_t I4[4 * 4] = {0};
  eye(I4, 4, 4);

  POSE2TF(cam0_ext.data, T_BC0);
  POSE2TF(cam1_ext.data, T_BC1);
  TF_INV(T_BC0, T_C0B);
  TF_CHAIN(T_C0C1, 2, T_C0B, T_BC1);
  TF_INV(T_C0C1, T_C1C0);

  real_t P0[4 * 4] = {0};
  real_t P1[4 * 4] = {0};
  pinhole_projection_matrix(params0, I4, P0);
  pinhole_projection_matrix(params1, T_C0C1, P1);

  // Check parallax
  inliers.clear();
  const size_t N = kps0.size();
  for (size_t i = 0; i < N; i++) {
    // Undistort
    const real_t z0_in[2] = {kps0[i].pt.x, kps0[i].pt.y};
    const real_t z1_in[2] = {kps1[i].pt.x, kps1[i].pt.y};
    real_t z0[2] = {0};
    real_t z1[2] = {0};
    cam0_params.undistort_func(params0, z0_in, z0);
    cam1_params.undistort_func(params1, z1_in, z1);

    // Triangulate
    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_C0);
    tf_point(T_C1C0, p_C0, p_C1);
    if (p_C0[2] < 0 && p_C1[2] < 0) {
      inliers.push_back(false);
      continue;
    }

    // Check parallax
    DOT(p_C0, 1, 3, p_C1, 3, 1, numerator);
    const double denominator = vec3_norm(p_C0) * vec3_norm(p_C1);
    const double parallax = rad2deg(acos(numerator[0] / denominator));
    if (parallax < parallax_threshold) {
      inliers.push_back(false);
    } else {
      inliers.push_back(true);
    }
  }
}

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
                   const real_t reproj_threshold = 0.5) {
  // Form camera i and camera j extrinsics T_CiCj
  TF_INV(T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiCj, 2, T_CiC0, T_C0Cj);

  // Form Projection matrices P_i and P_j
  TF_IDENTITY(T_eye);
  real_t P_i[3 * 4] = {0};
  real_t P_j[3 * 4] = {0};
  pinhole_projection_matrix(cam_i_params, T_eye, P_i);
  pinhole_projection_matrix(cam_j_params, T_CiCj, P_j);

  // Check features
  for (size_t n = 0; n < kps_i.size(); n++) {
    // Triangulate feature
    const real_t z_i[2] = {kps_i[n].pt.x, kps_i[n].pt.y};
    const real_t z_j[2] = {kps_j[n].pt.x, kps_j[n].pt.y};

    real_t uz_i[2] = {0};
    real_t uz_j[2] = {0};
    pinhole_radtan4_undistort(cam_i_params, z_i, uz_i);
    pinhole_radtan4_undistort(cam_j_params, z_j, uz_j);

    real_t p_Ci[3] = {0};
    linear_triangulation(P_i, P_j, uz_i, uz_j, p_Ci);
    points.emplace_back(p_Ci[0], p_Ci[1], p_Ci[2]);

    // Check feature depth
    if (p_Ci[2] < 0.0) {
      inliers.push_back(false);
      continue;
    }

    // Reproject feature into camera i
    real_t z_i_hat[2] = {0};
    cam_i_proj_func(cam_i_params, p_Ci, z_i_hat);

    // Check image point bounds
    const bool x_ok = (z_i_hat[0] < cam_i_res[0] && z_i_hat[0] > 0);
    const bool y_ok = (z_i_hat[1] < cam_i_res[1] && z_i_hat[1] > 0);
    if (!x_ok || !y_ok) {
      inliers.push_back(false);
      continue;
    }

    // Check reprojection error
    const real_t r[2] = {z_i[0] - z_i_hat[0], z_i[1] - z_i_hat[1]};
    const real_t reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
    if (reproj_error > reproj_threshold) {
      inliers.push_back(false);
      continue;
    }

    // Passed all tests
    inliers.push_back(true);
  }
}

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
              const int grid_cols = 4)
      : image_width{image_width},
        image_height{image_height}, grid_rows{grid_rows}, grid_cols{grid_cols} {
    for (int i = 0; i < (grid_rows * grid_cols); i++) {
      cells.push_back(0);
    }
  }

  virtual ~FeatureGrid() = default;

  /** Add keypoint */
  void add(const int pixel_x, const int pixel_y) {
    assert(pixel_x >= 0 && pixel_x <= image_width);
    assert(pixel_y >= 0 && pixel_y <= image_height);
    const int cell_idx = cellIndex(pixel_x, pixel_y);
    cells[cell_idx] += 1;
    keypoints.emplace_back(pixel_x, pixel_y);
  }

  /** Return cell index based on point `pt` */
  int cellIndex(const int pixel_x, const int pixel_y) const {
    const float img_w = image_width;
    const float img_h = image_height;
    float grid_x = ceil((std::max(1, pixel_x) / img_w) * grid_cols) - 1.0;
    float grid_y = ceil((std::max(1, pixel_y) / img_h) * grid_rows) - 1.0;
    const int cell_id = int(grid_x + (grid_y * grid_cols));
    return cell_id;
  }

  /** Return cell count */
  int count(const int cell_idx) const { return cells[cell_idx]; }

  /** Debug */
  void debug(const bool imshow = true) const {
    const int w = image_width;
    const int h = image_height;
    cv::Mat viz = cv::Mat::zeros(h, w, CV_32F);

    for (const auto kp : keypoints) {
      const auto x = kp.first;
      const auto y = kp.second;

      const cv::Point p(x, y);
      const int radius = 2;
      const cv::Scalar color{255, 255, 255};
      cv::circle(viz, p, radius, color, -1);
    }

    if (imshow) {
      cv::imshow("Figure 1", viz);
      cv::waitKey(0);
    }
  }
};

///////////////////
// GRID DETECTOR //
///////////////////

/**
 * Grid detector
 */
struct GridDetector {
  int max_keypoints = 200;
  int grid_rows = 3;
  int grid_cols = 4;

  // cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  // cv::Ptr<cv::Feature2D> detector = cv::FastFeatureDetector::create();
  cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();

  double quality_level = 0.01;
  double min_distance = 10;
  cv::Mat mask;
  int block_size = 3;
  bool use_harris = false;
  double k = 0.04;

  GridDetector() = default;
  virtual ~GridDetector() = default;

  /** Detect new keypoints **/
  void detect(
      const cv::Mat &image,
      std::vector<cv::KeyPoint> &kps_new,
      const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
      bool debug = false) const {
    // Asserts
    assert(image.channels() == 1);

    // Calculate number of grid cells and max corners per cell
    const int img_w = image.size().width;
    const int img_h = image.size().height;
    const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
    const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
    const int num_cells = grid_rows * grid_cols;
    const int max_per_cell = floor(max_keypoints / num_cells);

    // Create feature grid of previous keypoints
    FeatureGrid grid{img_w, img_h};
    for (const auto &kp : kps_prev) {
      const int pixel_x = kp.pt.x;
      const int pixel_y = kp.pt.y;
      grid.add(pixel_x, pixel_y);
    }

    // Detect corners in each grid cell
    int cell_idx = 0;
    for (int y = 0; y < img_h; y += dy) {
      for (int x = 0; x < img_w; x += dx) {
        // Make sure roi width and height are not out of bounds
        const int w = (x + dx > img_w) ? img_w - x : dx;
        const int h = (y + dy > img_h) ? img_h - y : dy;

        // Offset keypoints
        const int vacancy = max_per_cell - grid.count(cell_idx);
        if (vacancy <= 0) {
          continue;
        }

        // Detect corners in grid cell
        cv::Rect roi(x, y, w, h);
        std::vector<cv::KeyPoint> kps_roi;

        detector->setMaxFeatures(vacancy);
        detector->setQualityLevel(quality_level);
        detector->setMinDistance(min_distance);
        detector->setBlockSize(block_size);
        detector->setHarrisDetector(use_harris);
        detector->setK(k);
        detector->detect(image(roi), kps_roi);

        std::vector<cv::Point2f> corners;
        for (const auto kp : kps_roi) {
          corners.emplace_back(kp.pt.x, kp.pt.y);
        }

        cv::Size win_size{5, 5};
        cv::Size mask{-1, -1};
        cv::TermCriteria criteria{cv::TermCriteria::EPS +
                                      cv::TermCriteria::COUNT,
                                  40,
                                  0.001};
        cv::cornerSubPix(image(roi), corners, win_size, mask, criteria);

        for (size_t i = 0; i < corners.size(); i++) {
          const auto &corner = corners[i];
          kps_roi[i].pt.x = corner.x;
          kps_roi[i].pt.y = corner.y;
        }
        sort_keypoints(kps_roi);

        for (int i = 0; i < std::min((int) kps_roi.size(), vacancy); i++) {
          cv::KeyPoint kp = kps_roi[i];
          kp.pt.x += x;
          kp.pt.y += y;

          kp.pt.x = (kp.pt.x > 0) ? kp.pt.x : 0;
          kp.pt.x = (kp.pt.x < image.cols) ? kp.pt.x : image.cols - 1;
          kp.pt.y = (kp.pt.y > 0) ? kp.pt.y : 0;
          kp.pt.y = (kp.pt.y < image.rows) ? kp.pt.y : image.rows - 1;

          kps_new.push_back(kp);
          grid.add(kp.pt.x, kp.pt.y);
        }

        // Update cell_idx
        cell_idx += 1;
      }
    }
    kps_new = spread_keypoints(image, kps_new, min_distance, kps_prev);

    // Debug
    if (debug) {
      visualize(image, grid, kps_new, kps_prev);
    }
  }

  /** Visualize **/
  void visualize(const cv::Mat &image,
                 const FeatureGrid &grid,
                 const std::vector<cv::KeyPoint> &kps_new,
                 const std::vector<cv::KeyPoint> &kps_prev) const {
    // Visualization properties
    const auto red = cv::Scalar{0, 0, 255};
    const auto yellow = cv::Scalar{0, 255, 255};
    const auto line = cv::LINE_AA;
    const auto font = cv::FONT_HERSHEY_SIMPLEX;

    // Setup
    const int img_w = image.size().width;
    const int img_h = image.size().height;
    const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
    const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
    cv::Mat viz;
    cv::cvtColor(image, viz, cv::COLOR_GRAY2RGB);

    // Draw horizontal lines
    for (int x = 0; x < img_w; x += dx) {
      cv::line(viz, cv::Point2f(x, 0), cv::Point2f(x, img_w), red, 1, line);
    }

    // Draw vertical lines
    for (int y = 0; y < img_h; y += dy) {
      cv::line(viz, cv::Point2f(0, y), cv::Point2f(img_w, y), red, 1, line);
    }

    // Draw bin numbers
    int cell_idx = 0;
    for (int y = 0; y < img_h; y += dy) {
      for (int x = 0; x < img_w; x += dx) {
        auto text = std::to_string(grid.count(cell_idx));
        auto origin = cv::Point2f{x + 10.0f, y + 20.0f};
        cv::putText(viz, text, origin, font, 0.5, red, 1, line);
        cell_idx += 1;
      }
    }

    // Draw keypoints
    cv::drawKeypoints(viz, kps_new, viz, red);
    cv::drawKeypoints(viz, kps_prev, viz, yellow);

    // Imshow
    cv::imshow("viz", viz);
    cv::waitKey(0);
  }
};

/******************************************************************************
 * STATE-ESTIMATION
 *****************************************************************************/

/* Feature */
struct Feature {
  size_t feature_id = 0;
  std::map<int, camera_params_t> &cam_ints;
  std::map<int, extrinsic_t> &cam_exts;

  std::vector<timestamp_t> timestamps;
  std::map<timestamp_t, std::map<int, cv::KeyPoint>> keypoints;
  int min_length = 5;
  int max_length = 20;

  bool initialized = false;
  timestamp_t initialize_timestamp = 0;
  double data[3] = {0};

  Feature(const size_t feature_id_,
          std::map<int, camera_params_t> &cam_ints_,
          std::map<int, extrinsic_t> &cam_exts_)
      : feature_id{feature_id_}, cam_ints{cam_ints_}, cam_exts{cam_exts_} {}
  virtual ~Feature() = default;

  /** Return First Timestamp **/
  timestamp_t first_timestamp() const { return timestamps.front(); }

  /** Return Last Timestamp **/
  timestamp_t last_timestamp() const { return timestamps.back(); }

  /** Return length */
  size_t length() const { return timestamps.size(); }

  /** Return Camera Keypoints **/
  std::map<int, cv::KeyPoint> get_keypoints() const {
    const auto last_ts = timestamps.back();
    return keypoints.at(last_ts);
  }

  /** Update feature with new measurement **/
  void update(const timestamp_t ts, const int cam_idx, const cv::KeyPoint &kp) {
    timestamps.push_back(ts);
    keypoints[ts][cam_idx] = kp;
  }

  /** Initialize */
  bool initialize(const timestamp_t ts, const real_t T_WB[4 * 4]) {
    if (initialized) {
      return true;
    }

    // Do we have data?
    if (keypoints.count(ts) == 0) {
      return false;
    }

    // Is the feature tracked long enough?
    if (length() < min_length) {
      return false;
    }

    // Two or more measurements?
    if (keypoints[ts].size() < 2) {
      return false;
    }

    // Triangulate
    // -- Form projection matrices P0 and P1
    const real_t *params0 = cam_ints[0].data;
    const real_t *params1 = cam_ints[1].data;

    real_t I4[4 * 4] = {0};
    eye(I4, 4, 4);

    POSE2TF(cam_exts[0].data, T_BC0);
    POSE2TF(cam_exts[1].data, T_BC1);
    TF_INV(T_BC0, T_C0B);
    TF_CHAIN(T_C0C1, 2, T_C0B, T_BC1);

    real_t P0[4 * 4] = {0};
    real_t P1[4 * 4] = {0};
    pinhole_projection_matrix(params0, I4, P0);
    pinhole_projection_matrix(params1, T_C0C1, P1);

    // -- Undistort image points z0 and z1
    const auto kp0 = keypoints[ts][0];
    const auto kp1 = keypoints[ts][1];
    const real_t z0_in[2] = {kp0.pt.x, kp0.pt.y};
    const real_t z1_in[2] = {kp1.pt.x, kp1.pt.y};
    real_t z0[2] = {0};
    real_t z1[2] = {0};
    cam_ints[0].undistort_func(params0, z0_in, z0);
    cam_ints[1].undistort_func(params1, z1_in, z1);

    // -- Triangulate
    real_t p_C0[3] = {0};
    real_t p_W[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_C0);
    if (p_C0[2] <= 0) {
      return false;
    }
    TF_CHAIN(T_WC0, 2, T_WB, T_BC0);
    tf_point(T_WC0, p_C0, p_W);

    // Update
    initialized = true;
    initialize_timestamp = ts;
    data[0] = p_W[0];
    data[1] = p_W[1];
    data[2] = p_W[2];

    return true;
  }
};

/**
 * Pose local parameterization
 */
struct PoseLocalParameterization : public ceres::LocalParameterization {
  /** Plus */
  virtual bool Plus(const double *x,
                    const double *dx,
                    double *x_plus_dx) const {
    x_plus_dx[0] = x[0] + dx[0];
    x_plus_dx[1] = x[1] + dx[1];
    x_plus_dx[2] = x[2] + dx[2];

    double dq[4] = {0};
    quat_delta(dx + 3, dq);
    quat_mul(x + 3, dq, x_plus_dx + 3);
    quat_normalize(x_plus_dx + 3);

    return true;
  }

  /** Compute Jacobian */
  virtual bool ComputeJacobian(const double *x, double *J) const {
    // clang-format off
    J[0]  = 1; J[1]  = 0; J[2]  = 0;  J[3] = 0; J[4]  = 0; J[5]  = 0;
    J[6]  = 0; J[7]  = 1; J[8]  = 0;  J[9] = 0; J[10] = 0; J[11] = 0;
    J[12] = 0; J[13] = 0; J[14] = 1; J[15] = 0; J[16] = 0; J[17] = 0;
    J[18] = 0; J[19] = 0; J[20] = 0; J[21] = 1; J[22] = 0; J[23] = 0;
    J[24] = 0; J[25] = 0; J[26] = 0; J[27] = 0; J[28] = 1; J[29] = 0;
    J[30] = 0; J[31] = 0; J[32] = 0; J[33] = 0; J[34] = 0; J[35] = 1;
    J[36] = 0; J[37] = 0; J[38] = 0; J[39] = 0; J[40] = 0; J[41] = 0;
    // clang-format on
    return true;
  }

  /** Return global size */
  virtual int GlobalSize() const { return 7; }

  /** Return local size */
  virtual int LocalSize() const { return 6; }

  /** Form delta quaternion `dq` from a small rotation vector `dalpha`. */
  static void quat_delta(const double dalpha[3], double dq[4]) {
    assert(dalpha != NULL);
    assert(dq != NULL);

    const double half_norm = 0.5 * vec_norm(dalpha, 3);
    const double k = sinc(half_norm) * 0.5;
    const double vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
    double scalar = cos(half_norm);

    dq[0] = scalar;
    dq[1] = vector[0];
    dq[2] = vector[1];
    dq[3] = vector[2];
  }

  /** Quaternion left-multiply `p` with `q`, results are outputted to `r`. */
  static void quat_lmul(const double p[4], const double q[4], double r[4]) {
    assert(p != NULL);
    assert(q != NULL);
    assert(r != NULL);

    const double pw = p[0];
    const double px = p[1];
    const double py = p[2];
    const double pz = p[3];

    r[0] = pw * q[0] - px * q[1] - py * q[2] - pz * q[3];
    r[1] = px * q[0] + pw * q[1] - pz * q[2] + py * q[3];
    r[2] = py * q[0] + pz * q[1] + pw * q[2] - px * q[3];
    r[3] = pz * q[0] - py * q[1] + px * q[2] + pw * q[3];
  }

  /** Quaternion multiply `p` with `q`, results are outputted to `r`. */
  static void quat_mul(const double p[4], const double q[4], double r[4]) {
    assert(p != NULL);
    assert(q != NULL);
    assert(r != NULL);
    quat_lmul(p, q, r);
  }

  /** Return Quaternion norm */
  static double quat_norm(const double q[4]) {
    return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  }

  /** Normalize Quaternion */
  static void quat_normalize(double q[4]) {
    const double n = quat_norm(q);
    q[0] = q[0] / n;
    q[1] = q[1] / n;
    q[2] = q[2] / n;
    q[3] = q[3] / n;
  }
};

struct VisionFactor : ceres::CostFunction {
  timestamp_t ts;
  int cam_id = -1;
  size_t feature_id = 0;
  real_t z[2] = {0};
  real_t sqrt_info[2 * 2] = {0};

  /** Constructor **/
  VisionFactor() = delete;

  /** Constructor **/
  VisionFactor(const timestamp_t ts_,
               const int cam_id_,
               const size_t feature_id_,
               const real_t z_[2])
      : ts{ts_}, cam_id{cam_id_}, feature_id{feature_id_} {
    z[0] = z_[0];
    z[1] = z_[1];
    sqrt_info[0] = 1;
    sqrt_info[1] = 0;
    sqrt_info[2] = 0;
    sqrt_info[3] = 1;

    set_num_residuals(2);
    auto block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(7); // Body pose
    block_sizes->push_back(7); // Camera extrinsic
    block_sizes->push_back(3); // Feature position
    block_sizes->push_back(8); // Camera intrinsic
  }

  /** Destructor **/
  virtual ~VisionFactor() = default;

  /** Evaluate **/
  bool Evaluate(double const *const *params, double *res, double **jacs) const {
    // Map parameters
    TF(params[0], T_WB);
    TF(params[1], T_BCi);
    const double *p_W = params[2];
    const double *cam = params[3];

    // Transform feature from world to camera frame
    TF_INV(T_BCi, T_CiB);
    TF_INV(T_WB, T_BW);
    TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
    TF_POINT(T_CiW, p_W, p_Ci);

    // Project to image plane
    bool valid = true;
    real_t z_hat[2] = {0};
    const project_func_t proj_func = pinhole_radtan4_project;
    proj_func(cam, p_Ci, z_hat);

    bool x_ok = z_hat[0] > 0 && z_hat[0] < 752;
    bool y_ok = z_hat[1] > 0 && z_hat[1] < 480;
    if (p_Ci[2] < 0.01) {
      valid = false;
    } else if (!x_ok || !y_ok) {
      valid = false;
    }

    // Calculate residuals
    // -- Residual
    double r[2] = {0};
    r[0] = z[0] - z_hat[0];
    r[1] = z[1] - z_hat[1];
    // -- Weighted residual
    dot(sqrt_info, 2, 2, r, 2, 1, res);

    // Calculate jacobians
    // -- Form: -1 * sqrt_info
    real_t neg_sqrt_info[2 * 2] = {0};
    mat_copy(sqrt_info, 2, 2, neg_sqrt_info);
    neg_sqrt_info[0] *= -1.0;
    neg_sqrt_info[3] *= -1.0;
    // -- Form: Jh_ = -1 * sqrt_info * Jh
    real_t Jh[2 * 3] = {0};
    real_t Jh_[2 * 3] = {0};
    pinhole_radtan4_project_jacobian(cam, p_Ci, Jh);
    dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_);
    // -- Form: J_cam_params
    real_t J_cam_params[2 * 8] = {0};
    pinhole_radtan4_params_jacobian(cam, p_Ci, J_cam_params);
    // -- Form minimal Jacobians
    double mJ0[2 * 6] = {0};
    double mJ1[2 * 6] = {0};
    double mJ2[2 * 3] = {0};
    double mJ3[2 * 8] = {0};
    if (valid) {
      camera_factor_pose_jacobian(Jh_, T_WB, T_BCi, p_W, mJ0);
      camera_factor_extrinsic_jacobian(Jh_, T_BCi, p_Ci, mJ1);
      camera_factor_feature_jacobian(Jh_, T_WB, T_BCi, mJ2);
      camera_factor_camera_jacobian(neg_sqrt_info, J_cam_params, mJ3);
    }
    // -- Form global Jacobians
    if (jacs) {
      if (jacs[0]) {
        memset(jacs[0], 0, sizeof(double) * 2 * 7);
        mat_block_set(jacs[0], 7, 0, 1, 0, 5, mJ0);
      }

      if (jacs[1]) {
        memset(jacs[1], 0, sizeof(double) * 2 * 7);
        mat_block_set(jacs[1], 7, 0, 1, 0, 5, mJ1);
      }

      if (jacs[2]) {
        memset(jacs[2], 0, sizeof(double) * 2 * 3);
        mat_copy(mJ2, 2, 3, jacs[2]);
      }

      if (jacs[3]) {
        memset(jacs[3], 0, sizeof(double) * 2 * 8);
        mat_copy(mJ3, 2, 3, jacs[3]);
      }
    }

    return true;
  }

  /**
   * Pose jacobian
   */
  static void camera_factor_pose_jacobian(const real_t Jh_w[2 * 3],
                                          const real_t T_WB[3 * 3],
                                          const real_t T_BC[3 * 3],
                                          const real_t p_W[3],
                                          real_t J[2 * 6]) {
    assert(Jh_w != NULL);
    assert(T_BC != NULL);
    assert(T_WB != NULL);
    assert(p_W != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J_pos = Jh_w * C_CB * -C_BW;
    // J_rot = Jh_w * C_CB * C_BW * hat(p_W - r_WB) * -C_WB;
    // J = [J_pos, J_rot];

    // Setup
    real_t C_BW[3 * 3] = {0};
    real_t C_CB[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};

    TF_ROT(T_WB, C_WB);
    TF_ROT(T_BC, C_BC);
    mat_transpose(C_WB, 3, 3, C_BW);
    mat_transpose(C_BC, 3, 3, C_CB);
    dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

    // Form: -C_BW
    real_t neg_C_BW[3 * 3] = {0};
    mat_copy(C_BW, 3, 3, neg_C_BW);
    mat_scale(neg_C_BW, 3, 3, -1.0);

    // Form: -C_CW
    real_t neg_C_CW[3 * 3] = {0};
    dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

    // Form: -C_WB
    real_t neg_C_WB[3 * 3] = {0};
    mat_copy(C_WB, 3, 3, neg_C_WB);
    mat_scale(neg_C_WB, 3, 3, -1.0);

    // Form: C_CB * -C_BW * hat(p_W - r_WB) * -C_WB
    real_t p[3] = {0};
    real_t S[3 * 3] = {0};
    TF_TRANS(T_WB, r_WB);
    vec_sub(p_W, r_WB, p, 3);
    hat(p, S);

    real_t A[3 * 3] = {0};
    real_t B[3 * 3] = {0};
    dot(neg_C_CW, 3, 3, S, 3, 3, A);
    dot(A, 3, 3, neg_C_WB, 3, 3, B);

    // Form: J_pos = Jh_w * C_CB * -C_BW;
    real_t J_pos[2 * 3] = {0};
    dot(Jh_w, 2, 3, neg_C_CW, 3, 3, J_pos);

    J[0] = J_pos[0];
    J[1] = J_pos[1];
    J[2] = J_pos[2];

    J[6] = J_pos[3];
    J[7] = J_pos[4];
    J[8] = J_pos[5];

    // Form: J_rot = Jh_w * C_CB * -C_BW * hat(p_W - r_WB) * -C_WB;
    real_t J_rot[2 * 3] = {0};
    dot(Jh_w, 2, 3, B, 3, 3, J_rot);

    J[3] = J_rot[0];
    J[4] = J_rot[1];
    J[5] = J_rot[2];

    J[9] = J_rot[3];
    J[10] = J_rot[4];
    J[11] = J_rot[5];
  }

  /**
   * Body-camera extrinsic jacobian
   */
  static void camera_factor_extrinsic_jacobian(const real_t Jh_w[2 * 3],
                                               const real_t T_BC[4 * 4],
                                               const real_t p_C[3],
                                               real_t J[2 * 6]) {
    assert(Jh_w != NULL);
    assert(T_BC != NULL);
    assert(p_C != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J_pos = Jh_w * -C_CB;
    // J_rot = Jh_w * C_CB * hat(C_BC * p_C);

    // Setup
    real_t C_BC[3 * 3] = {0};
    real_t C_CB[3 * 3] = {0};
    real_t C_BW[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};

    tf_rot_get(T_BC, C_BC);
    mat_transpose(C_BC, 3, 3, C_CB);
    dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

    // Form: -C_CB
    real_t neg_C_CB[3 * 3] = {0};
    mat_copy(C_CB, 3, 3, neg_C_CB);
    mat_scale(neg_C_CB, 3, 3, -1.0);

    // Form: -C_BC
    real_t neg_C_BC[3 * 3] = {0};
    mat_copy(C_BC, 3, 3, neg_C_BC);
    mat_scale(neg_C_BC, 3, 3, -1.0);

    // Form: -C_CB * hat(C_BC * p_C) * -C_BC
    real_t p[3] = {0};
    real_t S[3 * 3] = {0};
    dot(C_BC, 3, 3, p_C, 3, 1, p);
    hat(p, S);

    real_t A[3 * 3] = {0};
    real_t B[3 * 3] = {0};
    dot(neg_C_CB, 3, 3, S, 3, 3, A);
    dot(A, 3, 3, neg_C_BC, 3, 3, B);

    // Form: J_rot = Jh_w * -C_CB;
    real_t J_pos[2 * 3] = {0};
    dot(Jh_w, 2, 3, neg_C_CB, 3, 3, J_pos);

    J[0] = J_pos[0];
    J[1] = J_pos[1];
    J[2] = J_pos[2];

    J[6] = J_pos[3];
    J[7] = J_pos[4];
    J[8] = J_pos[5];

    // Form: J_rot = Jh_w * -C_CB * hat(C_BC * p_C) * -C_BC;
    real_t J_rot[2 * 3] = {0};
    dot(Jh_w, 2, 3, B, 3, 3, J_rot);

    J[3] = J_rot[0];
    J[4] = J_rot[1];
    J[5] = J_rot[2];

    J[9] = J_rot[3];
    J[10] = J_rot[4];
    J[11] = J_rot[5];
  }

  /**
   * Camera parameters jacobian
   */
  static void camera_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                            const real_t J_cam_params[2 * 8],
                                            real_t J[2 * 8]) {
    assert(neg_sqrt_info != NULL);
    assert(J_cam_params != NULL);
    assert(J != NULL);

    // J = -1 * sqrt_info * J_cam_params;
    dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
  }

  /**
   * Feature jacobian
   */
  static void camera_factor_feature_jacobian(const real_t Jh_w[2 * 3],
                                             const real_t T_WB[4 * 4],
                                             const real_t T_BC[4 * 4],
                                             real_t J[2 * 3]) {
    if (J == NULL) {
      return;
    }
    assert(Jh_w != NULL);
    assert(T_WB != NULL);
    assert(T_BC != NULL);
    assert(J != NULL);

    // Jh_w = -1 * sqrt_info * Jh;
    // J = Jh_w * C_CW;

    // Setup
    real_t T_WC[4 * 4] = {0};
    real_t C_WC[3 * 3] = {0};
    real_t C_CW[3 * 3] = {0};
    dot(T_WB, 4, 4, T_BC, 4, 4, T_WC);
    tf_rot_get(T_WC, C_WC);
    mat_transpose(C_WC, 3, 3, C_CW);

    // Form: J = -1 * sqrt_info * Jh * C_CW;
    dot(Jh_w, 2, 3, C_CW, 3, 3, J);
  }
};

/** Estimate Relative Pose **/
real_t estimate_pose(std::map<int, camera_params_t> cam_ints,
                     std::map<int, extrinsic_t> cam_exts,
                     std::map<size_t, std::shared_ptr<Feature>> &features,
                     pose_t &pose_km1,
                     pose_t &pose_k,
                     const int max_iter = 10,
                     const int max_num_threads = 2,
                     const bool verbose = true,
                     const bool fix_features = true,
                     const bool fix_camera_intrinsics = true,
                     const bool fix_camera_extrinsics = true) {
  // Setup
  ceres::Problem::Options prob_options;
  ceres::Problem problem{prob_options};
  PoseLocalParameterization *pose_param = new PoseLocalParameterization();
  ceres::LossFunction *loss_fn = new ceres::CauchyLoss(1.0);

  // Add camera intrinsics and extrinsics to problem
  for (auto &[cam_id, cam_int] : cam_ints) {
    problem.AddParameterBlock(cam_int.data, 8);
    if (fix_camera_intrinsics) {
      problem.SetParameterBlockConstant(cam_int.data);
    }
  }
  for (auto &[cam_id, cam_ext] : cam_exts) {
    problem.AddParameterBlock(cam_ext.data, 7);
    problem.SetParameterization(cam_ext.data, pose_param);
    if (fix_camera_extrinsics) {
      problem.SetParameterBlockConstant(cam_ext.data);
    }
  }

  // Add poses at km1 and k, fix pose at km1
  problem.AddParameterBlock(pose_km1.data, 7);
  problem.AddParameterBlock(pose_k.data, 7);
  problem.SetParameterization(pose_km1.data, pose_param);
  problem.SetParameterization(pose_k.data, pose_param);
  problem.SetParameterBlockConstant(pose_km1.data);

  // Add vision factors
  std::vector<VisionFactor *> factors;
  for (auto &[fid, feature] : features) {
    // Pre-check
    if (feature->initialized == false) {
      continue;
    }

    // Add feature to problem
    problem.AddParameterBlock(feature->data, 3);
    if (fix_features) {
      problem.SetParameterBlockConstant(feature->data);
    }

    // Add camera factors at pose_k
    for (const auto &[cam_idx, kp] : feature->keypoints[pose_k.ts]) {
      // Form factor
      const real_t z[2] = {kp.pt.x, kp.pt.y};
      auto res_fn = new VisionFactor{pose_k.ts, cam_idx, fid, z};
      factors.push_back(res_fn);

      // Add factor to problem
      std::vector<double *> param_blocks;
      param_blocks.push_back(pose_k.data);
      param_blocks.push_back(cam_exts.at(cam_idx).data);
      param_blocks.push_back(feature->data);
      param_blocks.push_back(cam_ints.at(cam_idx).data);
      problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
    }

    // Add camera factors at pose_km1
    for (const auto &[cam_idx, kp] : feature->keypoints[pose_km1.ts]) {
      // Form factor
      const real_t z[2] = {kp.pt.x, kp.pt.y};
      auto res_fn = new VisionFactor{pose_km1.ts, cam_idx, fid, z};
      factors.push_back(res_fn);

      // Add factor to problem
      std::vector<double *> param_blocks;
      param_blocks.push_back(pose_km1.data);
      param_blocks.push_back(cam_exts.at(cam_idx).data);
      param_blocks.push_back(feature->data);
      param_blocks.push_back(cam_ints.at(cam_idx).data);
      problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
    }
  }

  // Evaluate reprojection errors
  std::vector<double> reproj_errors;
  std::map<size_t, std::vector<double>> feature_errors;
  auto eval_residuals = [&]() {
    for (auto &factor : factors) {
      double r[2] = {0.0, 0.0};

      std::vector<double *> params;
      if (factor->ts == pose_km1.ts) {
        params.push_back(pose_km1.data);
      } else {
        params.push_back(pose_k.data);
      }
      params.push_back(cam_exts.at(factor->cam_id).data);
      params.push_back(features[factor->feature_id]->data);
      params.push_back(cam_ints.at(factor->cam_id).data);
      factor->Evaluate(params.data(), r, nullptr);

      const auto reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
      reproj_errors.push_back(reproj_error);
      feature_errors[factor->feature_id].push_back(reproj_error);
    }
  };

  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

  // eval_residuals();
  // printf("Before\n");
  // printf("-----------------------\n");
  // printf("mean reproj error: %f\n", mean(reproj_errors));
  // printf("median reproj error: %f\n", median(reproj_errors));
  // printf("rmse reproj error: %f\n", rmse(reproj_errors));
  // printf("var reproj error: %f\n", var(reproj_errors));

  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl << std::endl;
  }

  eval_residuals();
  printf("Estimate pose\n");
  printf("-----------------------\n");
  printf("mean reproj error: %f\n", mean(reproj_errors));
  printf("median reproj error: %f\n", median(reproj_errors));
  printf("rmse reproj error: %f\n", rmse(reproj_errors));
  printf("var reproj error: %f\n", var(reproj_errors));

  const auto threshold = 3.0 * std::sqrt(var(reproj_errors));
  std::vector<size_t> outliers;
  for (auto &[fid, feature] : features) {
    if (mean(feature_errors[fid]) >= 5.0) {
      outliers.push_back(fid);
    }
  }
  printf("removed %ld outliers out of %ld\n", outliers.size(), features.size());
  for (const auto fid : outliers) {
    features.erase(fid);
  }

  printf("\n");
  printf("\n");

  return rmse(reproj_errors);
}

/** Two State Implicit Filter **/
struct TSIF {
  // Fronend
  GridDetector detector;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();

  // Settings
  int max_keypoints = 1000;
  bool enable_clahe = true;
  int parallax_threshold = 1.0;
  int max_length = 30;
  int min_length = 5;

  // Calibrations
  std::map<int, camera_params_t> cam_ints;
  std::map<int, extrinsic_t> cam_exts;

  // Features
  size_t next_feature_id = 0;
  std::map<size_t, std::shared_ptr<Feature>> features;
  std::map<size_t, std::shared_ptr<Feature>> old_features;

  // Data
  bool initialized = false;
  timestamp_t prev_ts = -1;
  cv::Mat prev_frame0;
  cv::Mat prev_frame1;
  size_t frame_index = 0;
  std::vector<pose_t> pose_hist;

  // Poses
  pose_t pose_init;
  pose_t pose_km1;
  pose_t pose_k;

  /** Constructor **/
  TSIF(std::map<int, camera_params_t> &cam_ints_,
       std::map<int, extrinsic_t> &cam_exts_)
      : cam_ints{cam_ints_}, cam_exts{cam_exts_} {
    real_t pose0[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    pose_setup(&pose_init, 0, pose0);
    pose_setup(&pose_km1, 0, pose0);
    pose_setup(&pose_k, 0, pose0);
  }

  /** Destructor **/
  virtual ~TSIF() = default;

  /** Set initial pose **/
  void set_initial_pose(const real_t T_WB[4 * 4]) {
    TF_VECTOR(T_WB, pose_vec);
    pose_setup(&pose_init, 0, pose_vec);
  }

  /** Add Feature **/
  void _add_feature(const timestamp_t ts,
                    const cv::KeyPoint &kp0,
                    const cv::KeyPoint &kp1) {
    const auto fid = next_feature_id;
    features[fid] = std::make_shared<Feature>(fid, cam_ints, cam_exts);
    features[fid]->update(ts, 0, kp0);
    features[fid]->update(ts, 1, kp1);
    next_feature_id += 1;
  }

  /** Update Feature **/
  void _update_feature(const size_t fid,
                       const timestamp_t ts,
                       const cv::KeyPoint &kp0,
                       const cv::KeyPoint &kp1) {
    features[fid]->update(ts, 0, kp0);
    features[fid]->update(ts, 1, kp1);

    if (features[fid]->length() >= features[fid]->max_length) {
      _remove_feature(fid);
    }
  }

  /** Remove Feature **/
  void _remove_feature(const size_t fid) {
    old_features[fid] = features[fid];
    features.erase(fid);
  }

  /** Get keypoints **/
  void _get_keypoints(std::vector<size_t> &feature_ids,
                      std::vector<cv::KeyPoint> &kps0,
                      std::vector<cv::KeyPoint> &kps1) {
    for (const auto &[fid, feature] : features) {
      auto keypoints = feature->get_keypoints();
      feature_ids.push_back(fid);
      kps0.push_back(keypoints[0]);
      kps1.push_back(keypoints[1]);
    }
  }

  void _initialize_features(const timestamp_t ts) {
    const int max_iter = 5;
    const int max_num_threads = 2;
    const bool verbose = false;
    const bool fix_features = false;
    const bool fix_camera_intrinsics = true;
    const bool fix_camera_extrinsics = true;

    // Initialize features
    POSE2TF(pose_k.data, T_WB);
    std::set<size_t> new_fids;
    for (auto &[fid, feature] : features) {
      if (feature->initialized) {
        continue;
      }
      if (feature->initialize(ts, T_WB) == true) {
        new_fids.insert(feature->feature_id);
      }
    }

    // Refine initialized features
    // -- Setup
    ceres::Problem::Options prob_options;
    ceres::Problem problem{prob_options};
    PoseLocalParameterization *pose_param = new PoseLocalParameterization();
    ceres::LossFunction *loss_fn = nullptr;

    // -- Add camera intrinsics and extrinsics to problem
    for (auto &[cam_id, cam_int] : cam_ints) {
      problem.AddParameterBlock(cam_int.data, 8);
      if (fix_camera_intrinsics) {
        problem.SetParameterBlockConstant(cam_int.data);
      }
    }
    for (auto &[cam_id, cam_ext] : cam_exts) {
      problem.AddParameterBlock(cam_ext.data, 7);
      problem.SetParameterization(cam_ext.data, pose_param);
      if (fix_camera_extrinsics) {
        problem.SetParameterBlockConstant(cam_ext.data);
      }
    }

    // -- Add poses at km1 and k, fix pose at km1
    problem.AddParameterBlock(pose_k.data, 7);
    problem.SetParameterization(pose_k.data, pose_param);
    problem.SetParameterBlockConstant(pose_k.data);

    // -- Add vision factors
    std::map<size_t, VisionFactor *> factors;
    for (auto &[fid, feature] : features) {
      // Pre-check
      if (feature->initialized == false) {
        continue;
      }

      // Add feature to problem
      problem.AddParameterBlock(feature->data, 3);

      // Add camera factors at pose_k
      for (const auto &[cam_idx, kp] : feature->keypoints[pose_k.ts]) {
        // Form factor
        const real_t z[2] = {kp.pt.x, kp.pt.y};
        auto res_fn = new VisionFactor{pose_k.ts, cam_idx, fid, z};
        factors[fid] = res_fn;

        // Add factor to problem
        std::vector<double *> param_blocks;
        param_blocks.push_back(pose_k.data);
        param_blocks.push_back(cam_exts.at(cam_idx).data);
        param_blocks.push_back(feature->data);
        param_blocks.push_back(cam_ints.at(cam_idx).data);
        problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
      }
    }

    // Solve
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = verbose;
    options.max_num_iterations = max_iter;
    options.num_threads = max_num_threads;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (verbose) {
      std::cout << summary.FullReport() << std::endl << std::endl;
    }

    // Evaluate reprojection errors
    std::vector<double> reproj_errors;
    std::map<size_t, double> feature_errors;
    auto eval_residuals = [&]() {
      for (auto &[fid, factor] : factors) {
        double r[2] = {0.0, 0.0};

        std::vector<double *> params;
        params.push_back(pose_k.data);
        params.push_back(cam_exts.at(factor->cam_id).data);
        params.push_back(features[factor->feature_id]->data);
        params.push_back(cam_ints.at(factor->cam_id).data);
        factor->Evaluate(params.data(), r, nullptr);

        const auto reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
        reproj_errors.push_back(reproj_error);
        feature_errors[factor->feature_id] = reproj_error;
      }
    };
    eval_residuals();

    printf("\n");
    printf("Initialize Features:\n");
    printf("--------------------\n");
    printf("mean reproj error: %f\n", mean(reproj_errors));
    printf("median reproj error: %f\n", median(reproj_errors));
    printf("rmse reproj error: %f\n", rmse(reproj_errors));
    printf("var reproj error: %f\n", var(reproj_errors));
    printf("\n");

    const auto threshold = 3.0 * std::sqrt(var(reproj_errors));
    std::vector<size_t> outliers;
    for (auto &[fid, feature] : features) {
      if (feature_errors[fid] >= threshold) {
        outliers.push_back(fid);
      }
    }
    for (const auto fid : outliers) {
      features.erase(fid);
    }
  }

  void _initialize(const timestamp_t ts) {
    // Pre-checks
    if (initialized) {
      return;
    } else if (frame_index < min_length) {
      return;
    }

    // Initialize features
    pose_setup(&pose_k, ts, pose_init.data);
    pose_setup(&pose_km1, ts, pose_init.data);
    _initialize_features(ts);
    initialized = true;
  }

  /** Detect Features **/
  void detect(const timestamp_t ts,
              const cv::Mat &frame0,
              const cv::Mat &frame1) {
    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0;
    std::vector<cv::KeyPoint> kps1;
    _get_keypoints(feature_ids, kps0, kps1);

    // Detect new
    std::vector<cv::KeyPoint> kps0_new;
    std::vector<cv::KeyPoint> kps1_new;
    detector.detect(frame0, kps0_new, kps0);
    if (kps0_new.size() < 10) {
      return;
    }

    // Track in space
    std::vector<uchar> optflow_inliers;
    optflow_track(frame0, frame1, kps0_new, kps1_new, optflow_inliers);
    filter_outliers(kps0_new, kps1_new, optflow_inliers);

    // Ransac in space
    std::vector<uchar> ransac_inliers;
    ransac(kps0_new,
           kps1_new,
           cam_ints[0].undistort_func,
           cam_ints[1].undistort_func,
           cam_ints[0].data,
           cam_ints[1].data,
           ransac_inliers);
    filter_outliers(kps0_new, kps1_new, ransac_inliers);

    // Check parallax
    std::vector<uchar> parallax_inliers;
    const real_t parallax_threshold = 1.0;
    check_parallax(cam_ints[0],
                   cam_ints[1],
                   cam_exts[0],
                   cam_exts[1],
                   kps0_new,
                   kps1_new,
                   parallax_threshold,
                   parallax_inliers);
    filter_outliers(kps0_new, kps1_new, parallax_inliers);

    // Add new features
    for (size_t i = 0; i < kps0_new.size(); i++) {
      _add_feature(ts, kps0_new[i], kps1_new[i]);
    }
  }

  /** Track features **/
  void track(const timestamp_t ts,
             const cv::Mat &frame0,
             const cv::Mat &frame1) {
    // Pre-check
    if (prev_frame0.empty() || prev_frame1.empty()) {
      return;
    }

    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0_km1;
    std::vector<cv::KeyPoint> kps1_km1;
    _get_keypoints(feature_ids, kps0_km1, kps1_km1);

    // Track in time and space
    std::vector<uchar> in0;
    std::vector<uchar> in1;
    std::vector<cv::KeyPoint> kps0_k;
    std::vector<cv::KeyPoint> kps1_k;
    optflow_track(prev_frame0, frame0, kps0_km1, kps0_k, in0);
    optflow_track(prev_frame1, frame1, kps1_km1, kps1_k, in1);

    // Ransac in time
    std::vector<uchar> rs0;
    std::vector<uchar> rs1;
    auto cam0_undistort_func = cam_ints[0].undistort_func;
    auto cam1_undistort_func = cam_ints[1].undistort_func;
    auto cam0_params = cam_ints[0].data;
    auto cam1_params = cam_ints[1].data;
    ransac(kps0_km1, kps0_k, cam0_undistort_func, cam0_params, rs0);
    ransac(kps1_km1, kps1_k, cam1_undistort_func, cam1_params, rs1);

    std::vector<uchar> in01;
    optflow_track(frame0, frame1, kps0_k, kps1_k, in01);

    // Update inliers and remove outliers
    for (size_t i = 0; i < in0.size(); i++) {
      const auto fid = feature_ids[i];
      if (in0[i] && in1[i] && in01[i] && rs0[i] && rs1[i]) {
        _update_feature(fid, ts, kps0_k[i], kps1_k[i]);
      } else {
        _remove_feature(fid);
      }
    }

    // Initialize or track
    if (initialized == false && frame_index >= min_length) {
      _initialize(ts);

    } else if (initialized) {
      // Estimate current pose
      pose_setup(&pose_k, ts, pose_km1.data);

      const int max_iter = 10;
      const int max_num_threads = 2;
      const bool verbose = false;
      estimate_pose(cam_ints,
                    cam_exts,
                    features,
                    pose_km1,
                    pose_k,
                    max_iter,
                    max_num_threads,
                    verbose);

      // Initialize new features
      _initialize_features(ts);

      // estimate_pose(cam_ints,
      //               cam_exts,
      //               features,
      //               pose_km1,
      //               pose_k,
      //               max_iter,
      //               max_num_threads,
      //               verbose,
      //               false);
    }
  }

  /** Visualize **/
  void _visualize(const cv::Mat &frame0, const cv::Mat &frame1) {
    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0_k;
    std::vector<cv::KeyPoint> kps1_k;
    _get_keypoints(feature_ids, kps0_k, kps1_k);

    // Visualize
    const cv::Scalar red{0, 0, 255};
    cv::Mat viz;
    cv::cvtColor(frame0, viz, cv::COLOR_GRAY2RGB);
    cv::drawKeypoints(viz, kps0_k, viz, red);
    cv::imshow("viz", viz);
  }

  /** Update **/
  void update(const timestamp_t ts,
              const cv::Mat &frame0,
              const cv::Mat &frame1) {
    // Apply CLAHE
    if (enable_clahe) {
      clahe->apply(frame0, frame0);
      clahe->apply(frame1, frame1);
    }

    // Detect and track
    detect(ts, frame0, frame1);
    track(ts, frame0, frame1);
    _visualize(frame0, frame1);
    if (initialized) {
      pose_hist.emplace_back(pose_k);
    }

    // Update
    prev_ts = ts;
    prev_frame0 = frame0.clone();
    prev_frame1 = frame1.clone();
    frame_index += 1;
  }
};

#endif // AVS_HPP

//////////////////////////////////////////////////////////////////////////////
//                              UNIT-TESTS                                  //
//////////////////////////////////////////////////////////////////////////////

#ifdef AVS_UNITTESTS

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

int test_feature_grid() {
  int image_width = 640;
  int image_height = 480;
  int grid_rows = 3;
  int grid_cols = 4;
  FeatureGrid grid{image_width, image_height, grid_rows, grid_cols};

  const float dx = (image_width / grid_cols);
  const float dy = (image_height / grid_rows);
  for (int i = 0; i < grid_rows; i++) {
    for (int j = 0; j < grid_cols; j++) {
      const int pixel_x = dx * j + dx / 2;
      const int pixel_y = dy * i + dy / 2;
      grid.add(pixel_x, pixel_y);
    }
  }
  grid.debug(true);

  return 0;
}

int test_spread_keypoints() {
  // Load image
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.png";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  const cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  std::vector<cv::KeyPoint> kps;
  detector->detect(img, kps);

  // Spread keypoints
  const int min_dist = 5;
  const std::vector<cv::KeyPoint> kps_prev;
  const bool debug = false;
  const auto kps_new = spread_keypoints(img, kps, min_dist, kps_prev, debug);

  // Assert keypoints are a Euclidean distance away from each other
  for (const auto kp : kps_new) {
    for (const auto kp_ref : kps_new) {
      const int px = kp.pt.x;
      const int py = kp.pt.y;
      const int px_ref = kp_ref.pt.x;
      const int py_ref = kp_ref.pt.y;

      const int dx = std::abs(px_ref - px);
      const int dy = std::abs(py_ref - py);
      if (dx == 0 && dy == 0) {
        continue;
      }

      const int dist = sqrt(dx * dx + dy * dy);
      TEST_ASSERT(dist >= min_dist);
    }
  }

  return 0;
}

int test_grid_detect() {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> kps_prev;
  std::vector<cv::KeyPoint> kps_new;
  GridDetector detector;
  detector.detect(img, kps_new, kps_prev, true);
  cv::waitKey(0);

  return 0;
}

int test_optflow_track() {
  const auto img_i_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img_j_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  const auto img_i = cv::imread(img_i_path, cv::IMREAD_GRAYSCALE);
  const auto img_j = cv::imread(img_j_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  std::vector<cv::KeyPoint> kps_new;
  GridDetector det;
  det.detect(img_i, kps_new);

  // Optical-flow track
  std::vector<cv::KeyPoint> kps_i = kps_new;
  std::vector<cv::KeyPoint> kps_j = kps_new;
  std::vector<uchar> inliers;
  optflow_track(img_i, img_j, kps_i, kps_j, inliers);

  return 0;
}

int test_reproj_filter() {
  const auto img_i_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img_j_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  auto img_i = cv::imread(img_i_path, cv::IMREAD_GRAYSCALE);
  auto img_j = cv::imread(img_j_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  std::vector<cv::KeyPoint> kps_new;
  GridDetector det;
  det.detect(img_i, kps_new);

  // Optical-flow track
  std::vector<cv::KeyPoint> kps_i = kps_new;
  std::vector<cv::KeyPoint> kps_j = kps_new;
  std::vector<uchar> optflow_inliers;
  optflow_track(img_i, img_j, kps_i, kps_j, optflow_inliers);

  // clang-format off
  const int cam_res[2] = {752, 480};
  real_t cam0_int[8] = {
    458.654, 457.296, 367.215, 248.375, -0.28340811,
    0.07395907, 0.00019359, 1.76187114e-05
  };
  real_t cam1_int[8] = {
    457.587, 456.134, 379.999, 255.238,
    -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05
  };
  const real_t T_SC0[4 * 4] = {
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0
  };
  const real_t T_SC1[4 * 4] = {
    0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
    0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
    -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
    0.0, 0.0, 0.0, 1.0
  };
  TF_INV(T_SC0, T_C0S);
  TF_IDENTITY(T_C0C0);
  TF_CHAIN(T_C0C1, 2, T_C0S, T_SC1);
  // clang-format on

  TIC(reproj_filter_time);
  std::vector<cv::Point3d> points;
  std::vector<bool> reproj_inliers;
  reproj_filter(pinhole_radtan4_project,
                cam_res,
                cam0_int,
                cam1_int,
                T_C0C0,
                T_C0C1,
                kps_i,
                kps_j,
                points,
                reproj_inliers);
  PRINT_TOC("Reproj Filter", reproj_filter_time);

  // Filter keypoints
  std::map<int, std::vector<cv::KeyPoint>> keypoints;
  keypoints[0] = std::vector<cv::KeyPoint>();
  keypoints[1] = std::vector<cv::KeyPoint>();
  for (size_t n = 0; n < kps_i.size(); n++) {
    if (optflow_inliers[n] && reproj_inliers[n]) {
      keypoints[0].push_back(kps_i[n]);
      keypoints[1].push_back(kps_j[n]);
    }
  }

  // Visualize
  const auto red = cv::Scalar{0, 0, 255};
  cv::Mat img0_viz;
  cv::Mat img1_viz;
  cv::Mat viz;
  cv::cvtColor(img_i, img0_viz, cv::COLOR_GRAY2RGB);
  cv::cvtColor(img_j, img1_viz, cv::COLOR_GRAY2RGB);
  cv::drawKeypoints(img0_viz, keypoints[0], img0_viz, red);
  cv::drawKeypoints(img1_viz, keypoints[1], img1_viz, red);
  cv::hconcat(img0_viz, img1_viz, viz);
  cv::imshow("viz", viz);
  cv::waitKey(0);

  return 0;
}

class EuRoCParams {
public:
  camera_params_t cam0_params;
  camera_params_t cam1_params;
  extrinsic_t cam0_ext;
  extrinsic_t cam1_ext;

  EuRoCParams() {
    // clang-format off
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    real_t cam0_data[8] = {
      458.654, 457.296, 367.215, 248.375,
      -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05
    };
    real_t cam1_data[8] = {
      457.587, 456.134, 379.999, 255.238,
      -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05
    };
    real_t cam0_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    real_t cam1_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const real_t T_SC0[4 * 4] = {
      0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
      0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
      -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
      0.0, 0.0, 0.0, 1.0
    };
    const real_t T_SC1[4 * 4] = {
      0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
      0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
      -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
      0.0, 0.0, 0.0, 1.0
    };
    tf_vector(T_SC0, cam0_ext_data);
    tf_vector(T_SC1, cam1_ext_data);

    camera_params_setup(&cam0_params, 0, cam_res, proj_model, dist_model, cam0_data);
    camera_params_setup(&cam1_params, 1, cam_res, proj_model, dist_model, cam1_data);
    extrinsic_setup(&cam0_ext, cam0_ext_data);
    extrinsic_setup(&cam1_ext, cam1_ext_data);
    // clang-format on
  }
};

int test_tracking() {
  // Setup
  const char *data_path = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_path);
  euroc_timeline_t *timeline = data->timeline;

  const cv::Scalar red{0, 0, 255};
  EuRoCParams euroc;
  GridDetector detector;
  cv::Mat frame0_km1;
  std::vector<cv::KeyPoint> kps0_km1;

  int imshow_wait = 1;
  for (size_t k = 0; k < timeline->num_timestamps; k++) {
    const timestamp_t ts = timeline->timestamps[k];
    const euroc_event_t *event = &timeline->events[k];

    if (event->has_cam0 && event->has_cam1) {
      struct timespec t_start = tic();
      const auto frame0_k = cv::imread(event->cam0_image, cv::IMREAD_GRAYSCALE);
      assert(frame0_k.empty() == false);

      // Track
      std::vector<cv::KeyPoint> kps0_k;
      if (k > 0) {
        std::vector<uchar> inliers;
        optflow_track(frame0_km1, frame0_k, kps0_km1, kps0_k, inliers);
        filter_outliers(kps0_km1, kps0_k, inliers);

        ransac(kps0_km1,
               kps0_k,
               euroc.cam0_params.undistort_func,
               euroc.cam0_params.undistort_func,
               euroc.cam0_params.data,
               euroc.cam0_params.data,
               inliers);
        filter_outliers(kps0_km1, kps0_k, inliers);
      }

      // Detect
      std::vector<cv::KeyPoint> kps_new;
      detector.detect(frame0_k, kps_new, kps0_km1);
      if (kps_new.size()) {
        extend_vector(kps0_k, kps_new);
      }

      // Visualize
      cv::Mat viz;
      cv::cvtColor(frame0_k, viz, cv::COLOR_GRAY2RGB);
      cv::drawKeypoints(viz, kps0_k, viz, red);
      cv::imshow("viz", viz);

      // Update
      frame0_km1 = frame0_k.clone();
      kps0_km1 = kps0_k;
      printf("track elasped: %f [s]\n", toc(&t_start));

      char key = cv::waitKey(imshow_wait);
      if (key == 'q') {
        k = timeline->num_timestamps;
      } else if (key == 's') {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 1) {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 0) {
        imshow_wait = 1;
      }
    }
  }

  // Clean up
  euroc_data_free(data);

  return 0;
}

int test_tsif() {
  // Setup
  const char *data_path = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_path);
  euroc_timeline_t *timeline = data->timeline;
  EuRoCParams euroc;

  std::map<int, camera_params_t> cam_ints;
  std::map<int, extrinsic_t> cam_exts;
  cam_ints[0] = euroc.cam0_params;
  cam_ints[1] = euroc.cam1_params;
  cam_exts[0] = euroc.cam0_ext;
  cam_exts[1] = euroc.cam1_ext;
  TSIF tsif{cam_ints, cam_exts};

  // clang-format off
  const int start_index = 2500;
  const timestamp_t start_ts = data->ground_truth->timestamps[start_index];
  const real_t *r_WB = data->ground_truth->p_RS_R[start_index];
  const real_t *q_WB = data->ground_truth->q_RS[start_index];
  TF_QR(q_WB, r_WB, T_WB);
  // clang-format on
  tsif.set_initial_pose(T_WB);

  FILE *gnuplot = gnuplot_init();
  gnuplot_send(gnuplot, "set title 'Plot XY'");

  auto plot = [&](const timestamp_t ts) {
    if (tsif.initialized == false) {
      return;
    }

    // Plot estimate
    // clang-format off
    std::vector<double> xvals;
    std::vector<double> yvals;
    int n = tsif.pose_hist.size();
    for (int i = 0; i < n; i++) {
      xvals.push_back(tsif.pose_hist[i].data[0]);
      yvals.push_back(tsif.pose_hist[i].data[1]);
    }
    gnuplot_send_xy(gnuplot, "$DATA1", xvals.data(), yvals.data(), n);
    fflush(gnuplot);
    // clang-format on

    // Plot ground-truth
    // clang-format off
    std::vector<double> xvals_gnd;
    std::vector<double> yvals_gnd;
    for (int i = 0; i < data->ground_truth->num_timestamps; i++) {
      if (data->ground_truth->timestamps[i] > ts) {
        break;
      }

      const real_t *r_WB = data->ground_truth->p_RS_R[i];
      xvals_gnd.push_back(r_WB[0]);
      yvals_gnd.push_back(r_WB[1]);
    }
    gnuplot_send_xy(gnuplot, "$DATA2", xvals_gnd.data(), yvals_gnd.data(), xvals_gnd.size());
    gnuplot_send(gnuplot, "plot $DATA1 with lines title 'Est', $DATA2 with lines title 'Gnd'");
    fflush(gnuplot);
    // clang-format on
  };

  int imshow_wait = 0;
  for (size_t k = 0; k < timeline->num_timestamps; k++) {
    const timestamp_t ts = timeline->timestamps[k];
    const euroc_event_t *event = &timeline->events[k];
    if (ts < start_ts) {
      continue;
    }

    if (event->has_cam0 && event->has_cam1) {
      const cv::Mat img0 = cv::imread(event->cam0_image, cv::IMREAD_GRAYSCALE);
      const cv::Mat img1 = cv::imread(event->cam1_image, cv::IMREAD_GRAYSCALE);
      assert(img0.empty() == false);
      assert(img1.empty() == false);

      struct timespec t_start = tic();
      tsif.update(ts, img0, img1);
      plot(ts);
      printf("track elasped: %f [s]\n", toc(&t_start));

      char key = cv::waitKey(imshow_wait);
      if (key == 'q') {
        k = timeline->num_timestamps;
      } else if (key == 's') {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 1) {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 0) {
        imshow_wait = 1;
      }
    }
  }

  // Clean up
  euroc_data_free(data);

  return 0;
}

void run_unittests() {
  // TEST(test_feature_grid);
  // TEST(test_spread_keypoints);
  // TEST(test_grid_detect);
  // TEST(test_optflow_track);
  // TEST(test_reproj_filter);
  // TEST(test_tracking);
  TEST(test_tsif);
}

#endif // AVS_UNITTESTS
