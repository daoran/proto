#include "prototype/munit.hpp"
#include "prototype/calib/chessboard.hpp"

namespace proto {

#define TEST_CONFIG "test_data/calib/chessboard/chessboard.yaml"
#define TEST_IMAGE "test_data/calib/chessboard/img0.jpg"

int test_chessboard_constructor() {
  chessboard_t cb;

  MU_CHECK(cb.nb_rows == 10);
  MU_CHECK(cb.nb_cols == 10);
  MU_CHECK_FLOAT(cb.square_size, 0.1);

  return 0;
}

int test_chessboard_configure() {
  chessboard_t cb;

  int retval = chessboard_configure(cb, TEST_CONFIG);
  MU_CHECK_EQ(0, retval);
  MU_CHECK(cb.nb_rows == 6);
  MU_CHECK(cb.nb_cols == 7);
  MU_CHECK_FLOAT(cb.square_size, 0.028);

  return 0;
}

int test_chessboard_detect() {
  chessboard_t cb;

  // Configure chessboard
  int retval = chessboard_configure(cb, TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Load image
  const cv::Mat image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Failed to load image [%s]!", TEST_IMAGE);
    return -1;
  }

  // Detect chessboard
  std::vector<cv::Point2f> corners;
  retval = chessboard_detect(cb, image, corners);
  MU_CHECK_EQ(retval, 0);
  MU_CHECK_EQ((int) corners.size(), cb.nb_rows * cb.nb_cols);

  const bool debug = false;
  if (debug) {
    cv::imshow("Image", image);
    cv::waitKey();
  }

  return 0;
}

int test_chessboard_draw() {
  chessboard_t cb;

  // Configure chessboard
  int retval = chessboard_configure(cb, TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  // Load image
  cv::Mat image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Failed to load image [%s]!", TEST_IMAGE);
    return -1;
  }

  // Draw chessboard
  retval = chessboard_draw(cb, image);
  MU_CHECK_EQ(retval, 0);

  const bool debug = false;
  if (debug) {
    cv::imshow("Image", image);
    cv::waitKey();
  }

  return 0;
}

int test_chessboard_solvepnp() {
  chessboard_t cb;

  // Configure chessboard
  int retval = chessboard_configure(cb, TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  // Load image
  const cv::Mat image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Failed to load image [%s]!", TEST_IMAGE);
    return -1;
  }

  // Detect chessboard
  std::vector<cv::Point2f> corners;
  retval = chessboard_detect(cb, image, corners);
  MU_CHECK_EQ(retval, 0);
  MU_CHECK_EQ((int) corners.size(), cb.nb_rows * cb.nb_cols);

  // Solve PnP
  const mat3_t K = pinhole_K(359.8796, 341.8768, 361.4580, 255.9160);
  mat4_t T_CF;
  matx_t X;
  retval = chessboard_solvepnp(cb, corners, K, T_CF, X);
  std::cout << T_CF << std::endl;
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_chessboard_project_points() {
  chessboard_t cb;

  // Configure chessboard
  int retval = chessboard_configure(cb, TEST_CONFIG);
  MU_CHECK_EQ(retval, 0);

  // Load image
  cv::Mat image = cv::imread(TEST_IMAGE);
  if (image.empty()) {
    LOG_ERROR("Failed to load image [%s]!", TEST_IMAGE);
    return -1;
  }

  // Detect chessboard
  std::vector<cv::Point2f> corners;
  retval = chessboard_detect(cb, image, corners);
  MU_CHECK_EQ(retval, 0);

  // Chessboard solvepnp
  const mat3_t K = pinhole_K(359.8796, 341.8768, 361.4580, 255.9160);
  mat4_t T_CF;
  matx_t X;
  retval = chessboard_solvepnp(cb, corners, K, T_CF, X);
  MU_CHECK_EQ(retval, 0);

  // Project points
  chessboard_project_points(cb, K, X, image);
  MU_CHECK_EQ(retval, 0);

  // Debug
  const bool debug = false;
  if (debug) {
    cv::imshow("Image", image);
    cv::waitKey();
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_chessboard_constructor);
  MU_ADD_TEST(test_chessboard_configure);
  MU_ADD_TEST(test_chessboard_detect);
  MU_ADD_TEST(test_chessboard_draw);
  MU_ADD_TEST(test_chessboard_solvepnp);
  MU_ADD_TEST(test_chessboard_project_points);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
