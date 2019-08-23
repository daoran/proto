#include "proto/vision/feature2d/grid_good.hpp"

namespace proto {

std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const double quality_level,
                                   const double min_distance,
                                   const cv::Mat mask,
                                   const int block_size,
                                   const bool use_harris_detector,
                                   const double k) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_corners_per_cell = (float) max_corners / (float) nb_cells;

  // Detect corners in each grid cell
  std::vector<cv::Point2f> corners_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const double w = (x + dx > image_width) ? image_width - x : dx;
      const double h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      const cv::Rect roi = cv::Rect(x, y, w, h);
      const cv::Mat sub_mask = (mask.rows == 0) ? cv::Mat() : mask(roi);
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(image_gray(roi),
                              corners,
                              max_corners,
                              quality_level,
                              min_distance,
                              sub_mask,
                              block_size,
                              use_harris_detector,
                              k);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      std::vector<cv::Point2f> corners_adjusted;
      for (auto &p : corners) {
        corners_adjusted.emplace_back(p.x += x, p.y += y);
        if (corners_adjusted.size() == max_corners_per_cell) {
          break;
        }
      }

      // Add to total corners detected
      corners_all.insert(std::end(corners_all),
                         std::begin(corners_adjusted),
                         std::end(corners_adjusted));
    }
  }

  return corners_all;
}

} //  namespace proto
