#include "avs.hpp"

//////////////////
// FEATURE GRID //
//////////////////

void feature_grid_setup(feature_grid_t &grid,
                        const int image_width,
                        const int image_height,
                        const int grid_rows,
                        const int grid_cols) {
  grid.image_width = image_width;
  grid.image_height = image_height;
  grid.grid_rows = grid_rows;
  grid.grid_cols = grid_cols;
  for (int i = 0; i < (grid_rows * grid_cols); i++) {
    grid.cells.push_back(0);
  }
}

void feature_grid_add(feature_grid_t &grid,
                      const int pixel_x,
                      const int pixel_y) {
  assert(pixel_x >= 0 && pixel_x <= grid.image_width);
  assert(pixel_y >= 0 && pixel_y <= grid.image_height);
  const int cell_idx = feature_grid_cell_index(grid, pixel_x, pixel_y);
  grid.cells[cell_idx] += 1;
  grid.keypoints.emplace_back(pixel_x, pixel_y);
}

int feature_grid_cell_index(const feature_grid_t &grid,
                            const int pixel_x,
                            const int pixel_y) {
  const float img_w = grid.image_width;
  const float img_h = grid.image_height;
  const int grid_rows = grid.grid_rows;
  const int grid_cols = grid.grid_cols;
  const float grid_x = ceil((std::max(1, pixel_x) / img_w) * grid_cols) - 1.0;
  const float grid_y = ceil((std::max(1, pixel_y) / img_h) * grid_rows) - 1.0;
  const int cell_id = int(grid_x + (grid_y * grid_cols));
  return cell_id;
}

int feature_grid_count(const feature_grid_t &grid, const int cell_idx) {
  return grid.cells[cell_idx];
}

void feature_grid_debug(const feature_grid_t &grid, const bool imshow) {
  const int w = grid.image_width;
  const int h = grid.image_height;
  cv::Mat viz = cv::Mat::zeros(h, w, CV_32F);

  for (const auto kp : grid.keypoints) {
    const auto x = kp.first;
    const auto y = kp.second;
    printf("(%d, %d)\n", x, y);

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

std::vector<int>
spread_keypoints(const cv::Mat &image,
                 const std::vector<cv::KeyPoint> &keypoints,
                 const int min_dist,
                 const std::vector<cv::KeyPoint> prev_keypoints,
                 const bool debug) {
  // Setup
  std::vector<int> outliers = std::vector<int>(keypoints.size(), 0);

  // Pre-check
  if (keypoints.size() == 0) {
    return outliers;
  }

  // Setup
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  uint8_t *A = CALLOC(uint8_t, img_w * img_h);
  uint8_t *W = CALLOC(uint8_t, (min_dist * 2) * (min_dist * 2));

  // Loop through previous keypoints
  for (const auto kp : prev_keypoints) {
    // Fill the area of the matrix where the next keypoint cannot be around
    const int p[2] = {(int) kp.pt.x, (int) kp.pt.y};
    const int rs = std::max(p[1] - min_dist, 0);
    const int re = std::min(p[1] + min_dist + 1, img_h);
    const int cs = std::max(p[0] - min_dist, 0);
    const int ce = std::min(p[0] + min_dist + 1, img_w);

    size_t idx = 0;
    for (size_t i = rs; i <= re; i++) {
      for (size_t j = cs; j <= ce; j++) {
        A[(i * img_w) + j] = W[idx];
        idx++;
      }
    }
  }

  // Loop through keypoints
  std::vector<cv::KeyPoint> kps_results;
  for (const auto kp : keypoints) {
    // Check if point is ok to be added to results
    const int p[2] = {(int) kp.pt.x, (int) kp.pt.y};
    if (A[(p[1] * img_w) + p[0]] > 0.0) {
      continue;
    }

    // Fill the area of the matrix where the next keypoint cannot be around
    const int rs = std::max(p[1] - min_dist, 0);
    const int re = std::min(p[1] + min_dist + 1, img_h);
    const int cs = std::max(p[0] - min_dist, 0);
    const int ce = std::min(p[0] + min_dist + 1, img_w);

    size_t idx = 0;
    for (size_t i = rs; i <= re; i++) {
      for (size_t j = cs; j <= ce; j++) {
        A[(i * img_w) + j] = W[idx];
        idx++;
      }
    }
  }

  return outliers;
}

void grid_detector_detect(const grid_detector_t &detector,
                          const cv::Mat &image,
                          const std::vector<cv::KeyPoint> &prev_kps,
                          std::vector<cv::KeyPoint> &kps,
                          cv::Mat &des,
                          bool debug = false) {
  // Asserts
  assert(image.channels() == 1);

  // Calculate number of grid cells and max corners per cell
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  const int grid_rows = detector.grid_rows;
  const int grid_cols = detector.grid_cols;
  const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
  const int num_cells = grid_rows * grid_cols;
  const int max_per_cell = floor(detector.max_keypoints / num_cells);

  // Detect corners in each grid cell
  feature_grid_t grid;
  feature_grid_setup(grid, img_w, img_h);
  for (const auto &kp : prev_kps) {
    const int pixel_x = kp.pt.x;
    const int pixel_y = kp.pt.y;
    feature_grid_add(grid, pixel_x, pixel_y);
  }

  std::vector<cv::KeyPoint> kps_all;
  cv::Mat des_all;
  int cell_idx = 0;
  for (int y = 0; y < img_h; y += dy) {
    for (int x = 0; x < img_w; x += dx) {
      // Make sure roi width and height are not out of bounds
      const int w = (x + dx > img_w) ? img_w - x : dx;
      const int h = (y + dy > img_h) ? img_h - y : dy;

      // Detect corners in grid cell
      cv::Rect roi(x, y, w, h);
      std::vector<cv::KeyPoint> kps;
      cv::Mat des;
      detector.detector->detectAndCompute(image(roi), cv::Mat(), kps, des);

      // Offset keypoints
      const size_t vacancy = max_per_cell - feature_grid_count(grid, cell_idx);
      if (vacancy <= 0) {
        continue;
      }

      for (int i = 0; i < std::min(kps.size(), vacancy); i++) {
        cv::KeyPoint kp = kps[i];
        kp.pt.x += x;
        kp.pt.y += y;
        kps_all.push_back(kp);
        // des_all.push_back(des[i, :]);

        feature_grid_add(grid, kp.pt.x, kp.pt.y);
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
        auto text = std::to_string(feature_grid_count(grid, cell_idx));
        auto origin = cv::Point2f{x + 10.0f, y + 20.0f};
        cv::putText(viz, text, origin, font, 0.5, red, 1, line);
        cell_idx += 1;
      }
    }

    // Draw keypoints
    cv::drawKeypoints(viz, kps_all, viz, red);
    cv::drawKeypoints(viz, prev_kps, viz, yellow);

    // Imshow
    cv::imshow("viz", viz);
    cv::waitKey(0);
  }
}

///////////////
// FRONT-END //
///////////////

void FrontEnd::detect(const cv::Mat &img0,
                      const cv::Mat &img1,
                      const bool debug) {
  detector_->detect(img0, kps0_);
  detector_->detect(img1, kps1_);
  descriptor_->compute(img0, kps0_, des0_);
  descriptor_->compute(img1, kps1_, des1_);

  if (debug) {
    const cv::Scalar color = cv::Scalar::all(-1);
    const auto flags = cv::DrawMatchesFlags::DEFAULT;

    cv::Mat viz0;
    cv::Mat viz1;
    drawKeypoints(img0, kps0_, viz0, color, flags);
    drawKeypoints(img1, kps1_, viz1, color, flags);

    cv::Mat viz;
    cv::hconcat(viz0, viz1, viz);
    cv::imshow("Stereo-Camera", viz);
    cv::waitKey(1);
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
  feature_grid_t grid;
  feature_grid_setup(grid, image_width, image_height);

  int grid_rows = 3;
  int grid_cols = 4;
  const float dx = (image_width / grid_cols);
  const float dy = (image_height / grid_rows);
  for (int i = 0; i < grid_rows; i++) {
    for (int j = 0; j < grid_cols; j++) {
      const int pixel_x = dx * j + dx / 2;
      const int pixel_y = dy * i + dy / 2;
      feature_grid_add(grid, pixel_x, pixel_y);
    }
  }

  // feature_grid_debug(&grid, true);

  return 0;
}

int test_grid_detect() {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.png";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> prev_kps;
  std::vector<cv::KeyPoint> kps;
  cv::Mat des;
  grid_detector_t detector;
  grid_detector_detect(detector, img, prev_kps, kps, des, true);

  return 0;
}

void run_unittests() {
  TEST(test_feature_grid);
  TEST(test_grid_detect);
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
