

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
  int count(const int cell_idx) const { return cells[cell_idx]; }

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
