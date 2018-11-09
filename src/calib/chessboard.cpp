#include "prototype/calib/chessboard.hpp"

namespace prototype {

chessboard_t::chessboard_t() {}

chessboard_t::chessboard_t(const std::string &config_file_)
    : config_file{config_file_} {
  chessboard_load(*this, config_file_);
}

chessboard_t::~chessboard_t() {}

int chessboard_load(chessboard_t &cb, const std::string &config_file) {
  // Parse config file
  config_t config{config_file};
  if (!config.ok) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }
  parse(config, "nb_rows", cb.nb_rows);
  parse(config, "nb_cols", cb.nb_cols);
  parse(config, "square_size", cb.square_size);

  // Create object points
  cb.object_points.clear();
  for (int i = 0; i < cb.nb_rows; i++) {
    for (int j = 0; j < cb.nb_cols; j++) {
      cb.object_points.emplace_back(j * cb.square_size,
                                    i * cb.square_size,
                                    0.0);
    }
  }

  cb.config_file = config_file;
  cb.ok = true;
  return 0;
}

int chessboard_detect(const chessboard_t &cb,
                      const cv::Mat &image,
                      std::vector<cv::Point2f> &corners) {
  assert(cb.ok);

  // Find the chessboard corners
  // -- Convert image to grayscale
  cv::Mat image_gray;
  if (image.channels() != 1) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = image.clone();
  }
  // -- Detect chessboard corners
  cv::Size size(cb.nb_cols, cb.nb_rows);
  if (cv::findChessboardCorners(image_gray, size, corners) == false) {
    return -1;
  }
  // -- Refine corner locations
  cv::Size win_size(11, 11);
  cv::Size zero_zone(-1, -1);
  cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                            (int) 1.0e10,
                            1.0e-10);
  cv::cornerSubPix(image_gray, corners, win_size, zero_zone, criteria);

  return 0;
}

int chessboard_draw_corners(const chessboard_t &cb, cv::Mat &image) {
  assert(cb.ok);

  // Detect corners
  std::vector<cv::Point2f> corners;
  if (chessboard_detect(cb, image, corners) != 0) {
    return -1;
  }

  // Draw
  cv::Size size(cb.nb_cols, cb.nb_rows);
  cv::drawChessboardCorners(image, size, corners, true);

  return 0;
}

int chessboard_solvepnp(const chessboard_t &cb,
                        const std::vector<cv::Point2f> corners,
                        const mat3_t &K,
                        mat4_t &T_CF,
                        matx_t &X) {
  assert(cb.ok);

  // Calculate transformation matrix
  cv::Mat D;
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(cb.object_points,
               corners,
               convert(K),
               D,
               rvec,
               tvec,
               false,
               cv::SOLVEPNP_ITERATIVE);

  // Convert rotation vector to matrix
  cv::Mat R(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, R);

  // Form transformation matrix
  // clang-format off
  T_CF <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2),
    0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Convert object points from cv::Point3f to Eigen::Matrix
  const int nb_pts = cb.nb_rows * cb.nb_cols;
  matx_t obj_pts_homo;
  obj_pts_homo.resize(4, nb_pts);
  for (int i = 0; i < nb_pts; i++) {
    obj_pts_homo(0, i) = cb.object_points[i].x;
    obj_pts_homo(1, i) = cb.object_points[i].y;
    obj_pts_homo(2, i) = cb.object_points[i].z;
    obj_pts_homo(3, i) = 1.0;
  }

  // Calculate chessboard corners
  const matx_t X_homo = T_CF * obj_pts_homo;
  X = X_homo.block(0, 0, 3, nb_pts);

  return 0;
}

void chessboard_project_points(const chessboard_t &cb,
                               const matx_t &X,
                               const mat3_t &K,
                               cv::Mat &image) {
  assert(cb.ok);

  // Project 3d point to image plane
  matx_t x = K * X;

  for (int i = 0; i < x.cols(); i++) {
    const vec3_t p = x.col(i);
    const double px = p(0) / p(2);
    const double py = p(1) / p(2);

    cv::circle(image,                 // Target image
               cv::Point(px, py),     // Center
               3.0,                   // Radius
               cv::Scalar(0, 0, 255), // Colour
               CV_FILLED,             // Thickness
               8);                    // Line type
  }
}

} //  namespace prototype
