#include "prototype/munit.hpp"
#include "dataset/dataset.hpp"
#include "feature2d/image_processor.hpp"

namespace prototype {

#define TEST_DATA_PATH "test_data/kitti/raw/"
#define TEST_DATA "test_data/kitti/raw/2011_09_26/2011_09_26_drive_0001_sync"
#define TEST_CONFIG "test_configs/feature2d/image_processor.yaml"

int test_ImageProcessor_constructor() {
  ImageProcessor image_processor;
  return 0;
}

int test_ImageProcessor_configure() {
  ImageProcessor image_processor;

  int retval = image_processor.configure(TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);
  MU_CHECK_EQ(image_processor.cam0.camera_model, "pinhole");
  MU_CHECK_EQ(image_processor.cam0.distortion_model, "radtan");
  MU_CHECK_EQ(image_processor.cam1.camera_model, "pinhole");
  MU_CHECK_EQ(image_processor.cam1.distortion_model, "radtan");

  return 0;
}

int test_ImageProcessor_createImagePyramids() {
  ImageProcessor image_processor;

  int retval = image_processor.configure(TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  auto cam0_img = cv::imread(TEST_DATA "/image_00/data/0000000000.png");
  auto cam1_img = cv::imread(TEST_DATA "/image_01/data/0000000000.png");
  image_processor.createImagePyramids(cam0_img, cam1_img);

  // cv::imshow("cam0", cam0_img);
  // cv::imshow("cam1", cam1_img);
  // cv::waitKey(0);
  // std::cout << image_processor.curr_cam0_img_pyramid.size() << std::endl;

  MU_CHECK_EQ(image_processor.curr_cam0_img_pyramid.size(), 8);
  MU_CHECK_EQ(image_processor.curr_cam1_img_pyramid.size(), 8);

  return 0;
}

int test_ImageProcessor_initialize() {
  ImageProcessor image_processor;

  // Configure
  int retval = image_processor.configure(TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  // Initialize
  auto cam0_img = cv::imread(TEST_DATA "/image_00/data/0000000000.png");
  auto cam1_img = cv::imread(TEST_DATA "/image_01/data/0000000000.png");
  image_processor.createImagePyramids(cam0_img, cam1_img);
  image_processor.initialize(cam0_img);

  // Visualize results
  bool debug = false;
  if (debug) {
    image_processor.drawFeatures(cam0_img, cam1_img);
    cv::waitKey(0);
  }

  return 0;
}

int test_ImageProcessor_stereoMatch() {
  ImageProcessor image_processor;

  int retval = image_processor.configure(TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  auto cam0_img = cv::imread(TEST_DATA "/image_00/data/0000000000.png");
  auto cam1_img = cv::imread(TEST_DATA "/image_01/data/0000000000.png");
  image_processor.createImagePyramids(cam0_img, cam1_img);

  // Detect new features on the cam0 image
  int fast_min_features = 100;
  int fast_grid_rows = 4;
  int fast_grid_cols = 4;
  double fast_threshold = 3.0;
  bool fast_nonmax_suppression = false;
  bool fast_debug = false;
  const std::vector<cv::KeyPoint> new_features =
      grid_fast(cam0_img,
                fast_min_features,
                fast_grid_rows,
                fast_grid_cols,
                fast_threshold,
                fast_nonmax_suppression);

  // Find the stereo matched points for the newly detected features.
  // -- Convert cv::Keypoint to cv::Point2f
  std::vector<cv::Point2f> cam0_points(new_features.size());
  for (size_t i = 0; i < new_features.size(); ++i) {
    cam0_points[i] = new_features[i].pt;
  }
  // -- Peform stereo match
  std::vector<cv::Point2f> cam1_points(0);
  std::vector<unsigned char> inliers(0);
  image_processor.image_width = cam0_img.cols;
  image_processor.image_height = cam0_img.rows;
  image_processor.stereoMatch(cam0_points, cam1_points, inliers);
  // -- Visualize results
  bool debug = false;
  if (debug) {
    image_processor.drawFeatures(cam0_img, cam1_img);
    cv::waitKey(0);
  }

  return 0;
}

int test_ImageProcessor_test() {
  // KITTI Raw dataset
  RawDataset raw_dataset(TEST_DATA_PATH, "2011_09_26", "0001");
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!", TEST_DATA_PATH);
    return -1;
  }

  // Image processor
  ImageProcessor image_processor;
  if (image_processor.configure(TEST_CONFIG) != 0) {
    LOG_ERROR("Failed to configure image processor!");
    return -1;
  }

  for (size_t i = 0; i < raw_dataset.oxts.time.size(); i++) {
    // for (size_t i = 0; i < 5; i++) {
    const long ts = raw_dataset.oxts.timestamps[i];
    const cv::Mat cam0_img = cv::imread(raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(raw_dataset.cam1[i]);
    const Vec3 a_B = raw_dataset.oxts.a_B[i];
    const Vec3 w_B = raw_dataset.oxts.w_B[i];

    image_processor.imuCallback(a_B, w_B, ts);
    image_processor.stereoCallback(cam0_img, cam1_img, ts);

    cv::waitKey(0);
  }

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_ImageProcessor_constructor);
  // MU_ADD_TEST(test_ImageProcessor_configure);
  // MU_ADD_TEST(test_ImageProcessor_createImagePyramids);
  // MU_ADD_TEST(test_ImageProcessor_initialize);
  // MU_ADD_TEST(test_ImageProcessor_stereoMatch);
  MU_ADD_TEST(test_ImageProcessor_test);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
