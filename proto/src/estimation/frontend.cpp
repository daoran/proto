#include "proto/estimation/frontend.hpp"

namespace proto {

frontend_t::frontend_t() {}

frontend_t::~frontend_t() {}

int frontend_update(frontend_t &fe, const cv::Mat &image, const bool debug) {
  // Parameters for Shi-Tomasi algorithm
  std::vector<cv::Point2f> corners;
  const int max_corners = 100;
  const double quality_level = 0.01;
  const double min_distance = 10;
  const int block_size = 3;
  cv::Mat mask;
  const bool use_harris = false;
  const double k = 0.04;

  // Convert input image to gray
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  // Apply corner detection
  cv::goodFeaturesToTrack(image_gray,
                          corners,
                          max_corners,
                          quality_level,
                          min_distance,
                          mask,
                          block_size,
                          use_harris,
                          k);

  // Draw corners detected
  if (debug) {
    const int radius = 2;
    const cv::Mat img = image.clone();
    const cv::Scalar color(0, 255, 0);
    for (size_t i = 0; i < corners.size(); i++) {
      cv::circle(img, corners[i], radius, color, cv::FILLED);
    }
    cv::imshow("Frontend", img);
  }

  return 0;
}

} // namespace proto
