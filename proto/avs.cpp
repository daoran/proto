#include "avs.hpp"

///////////
// UTILS //
///////////

namespace cv {

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

} // namespace cv

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
  std::vector<cv::Point2f> pts0 = cv::kps2pts(kps0);
  std::vector<cv::Point2f> pts1 = cv::kps2pts(kps1);
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

void GridDetector::detect(const cv::Mat &image,
                          const std::vector<cv::KeyPoint> &kps_prev,
                          std::vector<cv::KeyPoint> &kps_new,
                          cv::Mat &des_new,
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
      cv::Mat des_roi;

      detector_->detect(image(roi), kps_roi);
      kps_roi = spread_keypoints(image(roi), kps_roi, 10, kps_prev);
      detector_->compute(image(roi), kps_roi, des_roi);

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
        if (des_new.empty()) {
          des_new = des_roi.row(i);
        } else {
          cv::vconcat(des_roi.row(i), des_new, des_new);
        }

        grid.add(kp.pt.x, kp.pt.y);
      }

      // Update cell_idx
      cell_idx += 1;
    }
  }

  if (debug) {
    // Visualization properties
    const auto red = cv::Scalar{0, 0, 255};
    const auto yellow = cv::Scalar{0, 255, 255};
    const auto line = cv::LINE_AA;
    const auto font = cv::FONT_HERSHEY_SIMPLEX;

    // Setup
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
}

///////////////
// FRONT-END //
///////////////

void FrontEnd::addCamera(const camera_params_t &cam_params,
                         const extrinsic_t &cam_ext) {
  cam_params_[cam_params.cam_idx] = cam_params;
  cam_exts_[cam_params.cam_idx] = cam_ext;
}

void FrontEnd::_reprojFilter(const int idx_i,
                             const int idx_j,
                             const std::vector<cv::KeyPoint> &kps_i,
                             const std::vector<cv::KeyPoint> &kps_j) {
}

void FrontEnd::detect(const cv::Mat &img0,
                      const cv::Mat &img1,
                      const bool debug) {
  // Detect
  std::vector<cv::KeyPoint> kps0_new;
  std::vector<cv::KeyPoint> kps1_new;
  cv::Mat des0_new;
  cv::Mat des1_new;
  detector_.detect(img0, kps0_, kps0_new, des0_new);
  detector_.detect(img1, kps1_, kps1_new, des1_new);

  std::vector<cv::DMatch> matches;
  matcher_->match(des0_new, des1_new, matches);

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

int test_grid_detect() {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> kps_prev;
  std::vector<cv::KeyPoint> kps;
  cv::Mat des;
  GridDetector detector;
  detector.detect(img, kps_prev, kps, des, true);

  return 0;
}

int test_front_end() {
  const auto img0_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img1_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  const auto img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
  const auto img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);

  FrontEnd frontend;
  frontend.detect(img0, img1, true);

  return 0;
}

void run_unittests() {
  TEST(test_feature_grid);
  TEST(test_spread_keypoints);
  TEST(test_grid_detect);
  TEST(test_front_end);
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

  // auto front_end = FrontEnd();

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
