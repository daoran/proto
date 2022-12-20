#include "avs.hpp"

///////////
// UTILS //
///////////

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

std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps) {
  std::vector<cv::Point2f> pts;
  for (const auto &kp : kps) {
    pts.push_back(kp.pt);
  }
  return pts;
}

void print_keypoint(const cv::KeyPoint &kp) {
  printf("angle: %f\n", kp.angle);
  printf("class_id: %d\n", kp.class_id);
  printf("octave: %d\n", kp.octave);
  printf("pt: [%.2f, %.2f]\n", kp.pt.x, kp.pt.y);
  printf("response: %f\n", kp.response);
  printf("size: %f\n", kp.size);
}

void sort_keypoints(std::vector<cv::KeyPoint> &kps) {
  std::sort(kps.begin(), kps.end(), [](cv::KeyPoint a, cv::KeyPoint b) {
    return a.response > b.response;
  });
}

std::vector<cv::KeyPoint>
spread_keypoints(const cv::Mat &image,
                 const std::vector<cv::KeyPoint> &kps,
                 const int min_dist,
                 const std::vector<cv::KeyPoint> &kps_prev,
                 const bool debug) {
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

void optflow_track(const cv::Mat &img_i,
                   const cv::Mat &img_j,
                   const std::vector<cv::KeyPoint> &kps_i,
                   std::vector<cv::KeyPoint> &kps_j,
                   std::vector<uchar> &inliers,
                   const int patch_size,
                   const int max_iter,
                   const int max_level,
                   const real_t epsilon,
                   const bool debug) {
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

std::vector<uchar> ransac(const std::vector<cv::KeyPoint> &kps0,
                          const std::vector<cv::KeyPoint> &kps1,
                          const undistort_func_t cam0_undist,
                          const undistort_func_t cam1_undist,
                          const real_t cam0_params[8],
                          const real_t cam1_params[8],
                          const double reproj_threshold,
                          const double confidence) {
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
  std::vector<uchar> outliers;
  cv::findFundamentalMat(pts0,
                         pts1,
                         method,
                         reproj_threshold,
                         confidence,
                         outliers);

  return outliers;
}

//////////////////
// FEATURE GRID //
//////////////////

FeatureGrid::FeatureGrid(const int image_width,
                         const int image_height,
                         const int grid_rows,
                         const int grid_cols)
    : image_width_{image_width}, image_height_{image_height},
      grid_rows_{grid_rows}, grid_cols_{grid_cols} {
  for (int i = 0; i < (grid_rows_ * grid_cols_); i++) {
    cells_.push_back(0);
  }
}

void FeatureGrid::add(const int pixel_x, const int pixel_y) {
  assert(pixel_x >= 0 && pixel_x <= image_width_);
  assert(pixel_y >= 0 && pixel_y <= image_height_);
  const int cell_idx = cellIndex(pixel_x, pixel_y);
  cells_[cell_idx] += 1;
  keypoints_.emplace_back(pixel_x, pixel_y);
}

int FeatureGrid::cellIndex(const int pixel_x, const int pixel_y) const {
  const float img_w = image_width_;
  const float img_h = image_height_;
  float grid_x = ceil((std::max(1, pixel_x) / img_w) * grid_cols_) - 1.0;
  float grid_y = ceil((std::max(1, pixel_y) / img_h) * grid_rows_) - 1.0;
  const int cell_id = int(grid_x + (grid_y * grid_cols_));
  return cell_id;
}

int FeatureGrid::count(const int cell_idx) const {
  return cells_[cell_idx];
}

void FeatureGrid::debug(const bool imshow) const {
  const int w = image_width_;
  const int h = image_height_;
  cv::Mat viz = cv::Mat::zeros(h, w, CV_32F);

  for (const auto kp : keypoints_) {
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

///////////////////
// GRID DETECTOR //
///////////////////

void GridDetector::detect(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &kps_new,
                          cv::Mat &des_new,
                          const std::vector<cv::KeyPoint> &kps_prev,
                          bool debug) {
  // Asserts
  assert(image.channels() == 1);

  // Calculate number of grid cells and max corners per cell
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  const int dx = int(std::ceil(float(img_w) / float(grid_cols_)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows_)));
  const int num_cells = grid_rows_ * grid_cols_;
  const int max_per_cell = floor(max_keypoints_ / num_cells);

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
      detector_->detect(image(roi), kps_roi);
      sort_keypoints(kps_roi);
      kps_roi = spread_keypoints(image(roi), kps_roi, 10, kps_prev);

      // Extract feature descriptors
      cv::Mat des_roi;
      if (optflow_mode_ == false) {
        detector_->compute(image(roi), kps_roi, des_roi);
      }

      // Offset keypoints
      const size_t vacancy = max_per_cell - grid.count(cell_idx);
      if (vacancy <= 0) {
        continue;
      }

      for (int i = 0; i < std::min(kps_roi.size(), vacancy); i++) {
        cv::KeyPoint kp = kps_roi[i];
        kp.pt.x += x;
        kp.pt.y += y;

        kps_new.push_back(kp);
        if (optflow_mode_ == false) {
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
    _debug(image, grid, kps_new, kps_prev);
  }
}

void GridDetector::detect(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &kps_new,
                          const std::vector<cv::KeyPoint> &kps_prev,
                          bool debug) {
  assert(optflow_mode_ == true);
  cv::Mat des_new;
  detect(image, kps_new, des_new, kps_prev, debug);
}

void GridDetector::_debug(const cv::Mat &image,
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
  const int dx = int(std::ceil(float(img_w) / float(grid_cols_)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows_)));
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

/////////////////////
// FEATURE-TRACKER //
/////////////////////

void FeatureTracker::addCamera(const camera_params_t &cam_params,
                               const extrinsic_t &cam_ext) {
  cam_params_[cam_params.cam_idx] = cam_params;
  cam_exts_[cam_params.cam_idx] = cam_ext;
}

void FeatureTracker::_reprojFilter(const int idx_i,
                                   const int idx_j,
                                   const std::vector<cv::KeyPoint> &kps_i,
                                   const std::vector<cv::KeyPoint> &kps_j) {
  // Form camera i and camera j extrinsics T_CiCj
  POSE2TF(cam_exts_.at(idx_i).data, T_C0Ci);
  POSE2TF(cam_exts_.at(idx_j).data, T_C0Cj);
  TF_INV(T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiCj, 2, T_CiC0, T_C0Cj);

  // Form Projection matrices P_i and P_j
  const real_t *params_i = cam_params_.at(idx_i).data;
  const real_t *params_j = cam_params_.at(idx_j).data;
  TF_IDENTITY(T_eye);
  real_t P_i[3 * 4] = {0};
  real_t P_j[3 * 4] = {0};
  pinhole_projection_matrix(params_i, T_eye, P_i);
  pinhole_projection_matrix(params_j, T_CiCj, P_j);

  // Check features
  std::vector<bool> inliers;
  for (size_t n = 0; n < kps_i.size(); n++) {
    // Triangulate feature
    const real_t z_i[2] = {kps_i[n].pt.x, kps_i[n].pt.y};
    const real_t z_j[2] = {kps_j[n].pt.x, kps_j[n].pt.y};
    real_t p_Ci[3] = {0};
    linear_triangulation(P_i, P_j, z_i, z_j, p_Ci);

    // Check feature depth
    if (p_Ci[2] < 0.0) {
      inliers.push_back(false);
      continue;
    }

    // Reproject feature into camera i
    const int *cam_res = cam_params_.at(idx_i).resolution;
    real_t z_i_hat[2] = {0};
    pinhole_radtan4_project(params_i, p_Ci, z_i_hat);

    // Check image point bounds
    const bool x_ok = (z_i_hat[0] < cam_res[0] && z_i_hat[0] > 0);
    const bool y_ok = (z_i_hat[1] < cam_res[1] && z_i_hat[1] > 0);
    if (!x_ok || !y_ok) {
      inliers.push_back(false);
      continue;
    }

    // Check reprojection error
    const real_t r[2] = {z_i[0] - z_i_hat[0], z_i[1] - z_i_hat[1]};
    const real_t reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
    if (reproj_error > reproj_threshold_) {
      inliers.push_back(false);
      continue;
    }

    // Passed all tests
    inliers.push_back(true);
  }
}

void FeatureTracker::detect(const cv::Mat &img0,
                            const cv::Mat &img1,
                            keyframe_t &kf,
                            const bool debug) {
  assert(cam_params_.size() == 2);
  assert(cam_exts_.size() == 2);

  // Detect
  std::vector<cv::KeyPoint> kps0_;
  std::vector<cv::KeyPoint> kps1_;
  std::vector<cv::KeyPoint> kps0_new;
  std::vector<cv::KeyPoint> kps1_new;
  cv::Mat des0_new;
  cv::Mat des1_new;
  detector_.detect(img0, kps0_new, des0_new, kps0_);
  detector_.detect(img1, kps1_new, des1_new, kps1_);

  std::vector<cv::DMatch> matches;
  matcher_->match(des0_new, des1_new, matches);

  // std::cout << "rows: " << des0_new.rows << std::endl;
  // std::cout << "cols: " << des0_new.cols << std::endl;
  // std::cout << cv::type2str(des0_new.type()) << std::endl;

  if (debug) {
    const cv::Scalar yellow = cv::Scalar{0, 255, 255};
    const cv::Scalar red = cv::Scalar{0, 0, 255};
    const auto flags = cv::DrawMatchesFlags::DEFAULT;

    cv::Mat viz0;
    cv::Mat viz1;
    drawKeypoints(img0, kps0_, viz0, yellow, flags);
    drawKeypoints(img1, kps1_, viz1, yellow, flags);
    drawKeypoints(img0, kps0_new, viz0, red, flags);
    drawKeypoints(img1, kps1_new, viz1, red, flags);

    cv::Mat viz;
    cv::drawMatches(img0, kps0_new, img1, kps1_new, matches, viz);

    // cv::Mat viz;
    // cv::hconcat(viz0, viz1, viz);

    cv::imshow("Debug", viz);
    cv::waitKey(0);
  }
}

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

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

int test_grid_detect() {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> kps_prev;
  std::vector<cv::KeyPoint> kps_new;
  cv::Mat des_new;
  GridDetector detector;
  detector.detect(img, kps_new, des_new, kps_prev, true);

  return 0;
}

int test_front_end() {
  const auto img0_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img1_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  const auto img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
  const auto img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);

  // clang-format off
  camera_params_t cam0_params;
  camera_params_t cam1_params;
  extrinsic_t cam0_ext;
  extrinsic_t cam1_ext;

  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  real_t cam0_data[8] = {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  real_t cam1_data[8] = {457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05};
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
  TF_INV(T_SC0, T_C0S);
  TF_CHAIN(T_C0C1, 2, T_C0S, T_SC1);
  tf_vector(T_C0C1, cam1_ext_data);

  camera_params_setup(&cam0_params, 0, cam_res, proj_model, dist_model, cam0_data);
  camera_params_setup(&cam1_params, 1, cam_res, proj_model, dist_model, cam1_data);
  extrinsic_setup(&cam0_ext, cam0_ext_data);
  extrinsic_setup(&cam1_ext, cam1_ext_data);
  // clang-format on

  FeatureTracker ft;
  keyframe_t kf;
  ft.addCamera(cam0_params, cam0_ext);
  ft.addCamera(cam1_params, cam1_ext);
  ft.detect(img0, img1, kf, false);

  return 0;
}

void run_unittests() {
  // TEST(test_feature_grid);
  // TEST(test_spread_keypoints);
  TEST(test_optflow_track);
  // TEST(test_grid_detect);
  // TEST(test_front_end);
}

//////////////////////////////////////////////////////////////////////////////
//                                MAIN                                      //
//////////////////////////////////////////////////////////////////////////////

int main() {
  run_unittests();

  // // Setup
  // const char *data_path = "/data/euroc/V1_01";
  // euroc_data_t *data = euroc_data_load(data_path);
  // euroc_timeline_t *timeline = data->timeline;

  // auto front_end = FeatureTracker();

  // for (size_t k = 0; k < timeline->num_timestamps; k++) {
  //   const timestamp_t ts = timeline->timestamps[k];
  //   const euroc_event_t *event = &timeline->events[k];

  //   if (event->has_cam0 && event->has_cam1) {
  //     const cv::Mat img0 = cv::imread(event->cam0_image);
  //     const cv::Mat img1 = cv::imread(event->cam1_image);

  //     front_end.detect(img0, img1, true);

  //     // cv::Mat viz;
  //     // cv::hconcat(img0, img1, viz);
  //     // cv::imshow("Stereo-Camera", viz);
  //     // cv::waitKey(1);
  //   }
  // }

  // // Clean up
  // euroc_data_free(data);
  return 0;
}
