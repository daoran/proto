#include "avs.hpp"

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
  std::vector<cv::KeyPoint> kps_new;
  cv::Mat des_new;
  GridDetector detector;
  detector.detect(img, kps_new, des_new, kps_prev, true);

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
  real_t cam0_int[8] = {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  real_t cam1_int[8] = {457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05};

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
  TrackerKeypoints keypoints;
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
    // camera_params_t cam0_params;
    // camera_params_t cam1_params;
    // extrinsic_t cam0_ext;
    // extrinsic_t cam1_ext;

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
  }
};

// int test_front_end() {
//   // const auto img0_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
//   // const auto img1_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
//   // const auto img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
//   // const auto img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);

//   // Feature Tracker
//   EuRoCParams euroc;
//   FeatureTracker ft;
//   ft.add_camera(euroc.cam0_params, euroc.cam0_ext);
//   ft.add_camera(euroc.cam1_params, euroc.cam1_ext);
//   ft.add_overlap({0, 1});

//   // Setup
//   const char *data_path = "/data/euroc/V1_01";
//   euroc_data_t *data = euroc_data_load(data_path);
//   euroc_timeline_t *timeline = data->timeline;

//   int imshow_wait = 1;
//   for (size_t k = 0; k < timeline->num_timestamps; k++) {
//     const timestamp_t ts = timeline->timestamps[k];
//     const euroc_event_t *event = &timeline->events[k];

//     if (event->has_cam0 && event->has_cam1) {
//       const cv::Mat img0 = cv::imread(event->cam0_image, cv::IMREAD_GRAYSCALE);
//       const cv::Mat img1 = cv::imread(event->cam1_image, cv::IMREAD_GRAYSCALE);
//       assert(img0.empty() == false);
//       assert(img1.empty() == false);
//       struct timespec t_start = tic();
//       // tracker.track({{0, img0}, {1, img1}}, true);
//       ft.track(ts, {{0, img0}, {1, img1}}, true);
//       printf("track elasped: %f [s]\n", toc(&t_start));

//       char key = cv::waitKey(imshow_wait);
//       if (key == 'q') {
//         k = timeline->num_timestamps;
//       } else if (key == 's') {
//         imshow_wait = 0;
//       } else if (key == ' ' && imshow_wait == 1) {
//         imshow_wait = 0;
//       } else if (key == ' ' && imshow_wait == 0) {
//         imshow_wait = 1;
//       }
//     }
//   }

//   // Clean up
//   euroc_data_free(data);

//   return 0;
// }

void run_unittests() {
  // TEST(test_feature_grid);
  // TEST(test_spread_keypoints);
  // TEST(test_grid_detect);
  // TEST(test_optflow_track);
  // TEST(test_reproj_filter);
  // TEST(test_front_end);
}

//////////////////////////////////////////////////////////////////////////////
//                                MAIN                                      //
//////////////////////////////////////////////////////////////////////////////

int main() {
  run_unittests();

  // Setup
  // const char *data_path = "/data/euroc/V1_01";
  // euroc_data_t *data = euroc_data_load(data_path);
  // euroc_timeline_t *timeline = data->timeline;

  // auto front_end = Tracker();
  // for (size_t k = 0; k < timeline->num_timestamps; k++) {
  //   const timestamp_t ts = timeline->timestamps[k];
  //   const euroc_event_t *event = &timeline->events[k];

  //   if (event->has_cam0 && event->has_cam1) {
  //     const cv::Mat img0 = cv::imread(event->cam0_image);
  //     const cv::Mat img1 = cv::imread(event->cam1_image);

  //     front_end.detect(img0, img1, true);

  //     cv::Mat viz;
  //     cv::hconcat(img0, img1, viz);
  //     cv::imshow("Stereo-Camera", viz);
  //     cv::waitKey(1);
  //   }
  // }

  // Clean up
  // euroc_data_free(data);

  return 0;
}
