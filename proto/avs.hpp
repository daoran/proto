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

typedef std::map<int, std::vector<cv::KeyPoint>> TrackerKeypoints;

///////////
// UTILS //
///////////

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
std::vector<cv::KeyPoint>
filter_outliers(const std::vector<cv::KeyPoint> &keypoints,
                const std::vector<uchar> inliers) {
  std::vector<cv::KeyPoint> kps_inliers;
  for (size_t i = 0; i < keypoints.size(); i++) {
    if (inliers[i]) {
      kps_inliers.push_back(keypoints[i]);
    }
  }
  return kps_inliers;
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
            const double reproj_threshold = 0.75,
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
  int count(const int cell_idx) const {
    return cells[cell_idx];
  }

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

        // Detect corners in grid cell
        cv::Rect roi(x, y, w, h);
        std::vector<cv::KeyPoint> kps_roi;
        detector->detect(image(roi), kps_roi);
        sort_keypoints(kps_roi);
        kps_roi = spread_keypoints(image(roi), kps_roi, 10, kps_prev);

        // Extract feature descriptors
        cv::Mat des_roi;
        if (optflow_mode == false) {
          detector->compute(image(roi), kps_roi, des_roi);
        }

        // Offset keypoints
        const int vacancy = max_per_cell - grid.count(cell_idx);
        if (vacancy <= 0) {
          continue;
        }

        for (int i = 0; i < std::min((int) kps_roi.size(), vacancy); i++) {
          cv::KeyPoint kp = kps_roi[i];
          kp.pt.x += x;
          kp.pt.y += y;

          kps_new.push_back(kp);
          if (optflow_mode == false) {
            if (des_new.empty()) {
              des_new = des_roi.row(i);
            } else {
              cv::vconcat(des_roi.row(i), des_new, des_new);
            }
          }

          grid.add(kp.pt.x, kp.pt.y);
        }

        // Update cell_idx
        cell_idx += 1;
      }
    }

    // Debug
    if (debug) {
      visualize(image, grid, kps_new, kps_prev);
    }
  }

  void detect(
      const cv::Mat &image,
      std::vector<cv::KeyPoint> &kps_new,
      const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
      bool debug = false) const {
    assert(optflow_mode == true);
    cv::Mat des_new;
    detect(image, kps_new, des_new, kps_prev, debug);
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

//////////////////
// FEATURE-INFO //
//////////////////

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

/******************************************************************************
 * STATE-ESTIMATION
 *****************************************************************************/

/* Pose */
struct Pose {
  timestamp_t ts = 0;
  real_t data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  Pose() = default;

  Pose(const timestamp_t ts_, const real_t T_WB[4 * 4]) : ts{ts_} {
    TF_VECTOR(T_WB, pose_vector);
    for (int i = 0; i < 7; i++) {
      data[i] = pose_vector[i];
    }
  }

  virtual ~Pose() = default;
};

/* Camera Intrinsic */
struct CameraIntrinsic {
  int id = -1;
  int resolution[2] = {0};
  std::string proj_model;
  std::string dist_model;
  real_t data[8] = {0};

  CameraIntrinsic() = default;

  CameraIntrinsic(const int id_,
                  const int resolution_[2],
                  const std::string &proj_model_,
                  const std::string &dist_model_,
                  const real_t data_[8])
      : id{id_}, proj_model{proj_model_}, dist_model{dist_model_} {
    for (int i = 0; i < 8; i++) {
      data[i] = data_[i];
    }
  }

  virtual ~CameraIntrinsic() = default;
};

/* Camera Extrinsic */
struct CameraExtrinsic {
  int id = -1;
  real_t data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  CameraExtrinsic() = default;

  CameraExtrinsic(const int id_, const real_t data_[7]) : id{id_} {
    for (int i = 0; i < 7; i++) {
      data[i] = data_[i];
    }
  }

  virtual ~CameraExtrinsic() = default;
};

/* Feature */
struct Feature {
  size_t id = 0;
  real_t data[3] = {0};

  Feature() = default;

  Feature(const size_t id_, const cv::Point3d &point_) : id{id_} {
    data[0] = point_.x;
    data[1] = point_.y;
    data[2] = point_.z;
  }

  Feature(const size_t id_, const real_t point_[3]) : id{id_} {
    data[0] = point_[0];
    data[1] = point_[1];
    data[2] = point_[2];
  }

  virtual ~Feature() = default;
};

/**
 * Pose local parameterization
 */
struct PoseLocalParameterization : public ceres::LocalParameterization {
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

  virtual int GlobalSize() const {
    return 7;
  }

  virtual int LocalSize() const {
    return 6;
  }

  /**
   * Form delta quaternion `dq` from a small rotation vector `dalpha`.
   */
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

  /**
   * Quaternion left-multiply `p` with `q`, results are outputted to `r`.
   */
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

  /**
   * Quaternion multiply `p` with `q`, results are outputted to `r`.
   */
  static void quat_mul(const double p[4], const double q[4], double r[4]) {
    assert(p != NULL);
    assert(q != NULL);
    assert(r != NULL);
    quat_lmul(p, q, r);
  }

  /**
   * Return Quaternion norm
   */
  static double quat_norm(const double q[4]) {
    return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  }

  /**
   * Normalize Quaternion
   */
  static void quat_normalize(double q[4]) {
    const double n = quat_norm(q);
    q[0] = q[0] / n;
    q[1] = q[1] / n;
    q[2] = q[2] / n;
    q[3] = q[3] / n;
  }
};

/**
 * Vision Factor
 */
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

// /** Estimate Relative Pose **/
// real_t estimate_relpose(const Mapper &mapper,
//                         CameraIntrinsics &cam_ints,
//                         CameraExtrinsics &cam_exts,
//                         TrackerData tracking_km1,
//                         TrackerData tracking_k,
//                         Features &features,
//                         Pose &pose_km1,
//                         Pose &pose_k,
//                         const bool fix_features = true,
//                         const bool fix_camera_intrinsics = true,
//                         const bool fix_camera_extrinsics = true) {
//   // Settings
//   bool verbose = true;
//   int max_iter = 10;
//   int max_num_threads = 2;

//   // Setup
//   ceres::Problem::Options prob_options;
//   ceres::Problem problem{prob_options};
//   PoseLocalParameterization *pose_param = new PoseLocalParameterization();
//   ceres::LossFunction *loss_fn = new ceres::CauchyLoss(1.0);

//   // Update feature positions from mapper
//   for (auto &[fid, feature] : features) {
//     if (mapper.features.count(fid)) {
//       feature = mapper.features.at(fid);
//     }
//   }

//   // Add camera intrinsics and extrinsics to problem
//   for (auto &[cam_id, cam_int] : cam_ints) {
//     problem.AddParameterBlock(cam_int.data, 8);
//     if (fix_camera_intrinsics) {
//       problem.SetParameterBlockConstant(cam_int.data);
//     }
//   }
//   for (auto &[cam_id, cam_ext] : cam_exts) {
//     problem.AddParameterBlock(cam_ext.data, 7);
//     problem.SetParameterization(cam_ext.data, pose_param);
//     if (fix_camera_extrinsics) {
//       problem.SetParameterBlockConstant(cam_ext.data);
//     }
//   }

//   // Add poses at km1 and k, fix pose at km1
//   problem.AddParameterBlock(pose_km1.data, 7);
//   problem.AddParameterBlock(pose_k.data, 7);
//   problem.SetParameterization(pose_km1.data, pose_param);
//   problem.SetParameterization(pose_k.data, pose_param);
//   problem.SetParameterBlockConstant(pose_km1.data);

//   // Add vision factors
//   std::set<size_t> feature_ids;
//   std::map<size_t, VisionFactor *> factors;
//   auto add_factors =
//       [&](const int ts_idx, TrackerData &tracking_data, Pose &pose) {
//         for (auto &[cam_id, cam_data] : tracking_data) {
//           for (const auto &[fid, kp] : cam_data) {
//             // Add feature to problem
//             if (feature_ids.count(fid) == 0) {
//               problem.AddParameterBlock(features.at(fid).data, 3);
//               if (fix_features) {
//                 problem.SetParameterBlockConstant(features.at(fid).data);
//               }
//               feature_ids.insert(fid);
//             }

//             // Form factor
//             auto &feature = features.at(fid);
//             const real_t z[2] = {kp.pt.x, kp.pt.y};
//             auto res_fn = new VisionFactor{ts_idx, cam_id, fid, z};
//             factors[fid] = res_fn;

//             // Add factor to problem
//             std::vector<double *> param_blocks;
//             param_blocks.push_back(pose.data);
//             param_blocks.push_back(cam_exts.at(cam_id).data);
//             param_blocks.push_back(feature.data);
//             param_blocks.push_back(cam_ints.at(cam_id).data);
//             problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
//           }
//         }
//       };
//   add_factors(0, tracking_km1, pose_km1);
//   add_factors(1, tracking_k, pose_k);

//   // Solve
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = verbose;
//   options.max_num_iterations = max_iter;
//   options.num_threads = max_num_threads;
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   if (verbose) {
//     std::cout << summary.FullReport() << std::endl << std::endl;
//   }

//   // Evaluate reprojection errors
//   std::vector<double> reproj_errors;
//   std::map<size_t, double> feature_errors;
//   auto eval_residuals = [&]() {
//     for (auto &[fid, factor] : factors) {
//       double r[2] = {0.0, 0.0};

//       std::vector<double *> params;
//       if (factor->ts == 0) {
//         params.push_back(pose_km1.data);
//       } else {
//         params.push_back(pose_k.data);
//       }
//       params.push_back(cam_exts.at(factor->cam_id).data);
//       params.push_back(features[factor->feature_id].data);
//       params.push_back(cam_ints.at(factor->cam_id).data);
//       factor->Evaluate(params.data(), r, nullptr);

//       const auto reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
//       reproj_errors.push_back(reproj_error);
//       feature_errors[factor->feature_id] = reproj_error;
//     }
//   };
//   eval_residuals();

//   // printf("mean reproj error: %f\n", mean(reproj_errors));
//   // printf("median reproj error: %f\n", median(reproj_errors));
//   // printf("rmse reproj error: %f\n", rmse(reproj_errors));
//   // printf("var reproj error: %f\n", var(reproj_errors));
//   return rmse(reproj_errors);
// }

#endif // AVS_HPP
