#include "avs.hpp"

//////////////////
// FEATURE GRID //
//////////////////

void feature_grid_setup(feature_grid_t *grid,
                        const int image_width,
                        const int image_height,
                        const std::vector<cv::KeyPoint> &keypoints,
                        const int grid_rows,
                        const int grid_cols) {
  grid->image_width = image_width;
  grid->image_height = image_height;
  grid->keypoints = keypoints;
  grid->grid_rows = grid_rows;
  grid->grid_cols = grid_cols;
  for (int i = 0; i < (grid_rows * grid_cols); i++) {
    grid->cells.push_back(0);
  }

  for (const auto &kp : keypoints) {
    assert(kp.pt.x >= 0 and kp.pt.x <= image_width);
    assert(kp.pt.y >= 0 and kp.pt.y <= image_height);
    const int cell_idx = feature_grid_cell_index(grid, kp);
    grid->cells[cell_idx] += 1;
  }
}

int feature_grid_cell_index(const feature_grid_t *grid,
                            const cv::KeyPoint &kp) {
  const float px = kp.pt.x;
  const float py = kp.pt.y;
  const float img_w = grid->image_width;
  const float img_h = grid->image_height;
  const int grid_rows = grid->grid_rows;
  const int grid_cols = grid->grid_cols;
  const float grid_x = ceil((std::max(1.0f, px) / img_w) * grid_cols) - 1.0;
  const float grid_y = ceil((std::max(1.0f, py) / img_h) * grid_rows) - 1.0;
  const int cell_id = int(grid_x + (grid_y * grid_cols));
  return cell_id;
}

int feature_grid_count(const feature_grid_t *grid, const int cell_idx) {
  return grid->cells[cell_idx];
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

void grid_detect(const cv::Ptr<cv::Feature2D> &detector,
                 const cv::Mat &image,
                 const int max_keypoints,
                 const int grid_rows,
                 const int grid_cols,
                 const std::vector<cv::KeyPoint> &prev_keypoints) {
  // Asserts
  assert(image.channels() == 1);

  // Calculate number of grid cells and max corners per cell
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
  const int nb_cells = grid_rows * grid_cols;
  const int max_per_cell = floor(max_keypoints / nb_cells);

  // Detect corners in each grid cell
  feature_grid_t grid;
  feature_grid_setup(&grid, img_w, img_h);
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
      detector->detectAndCompute(image(roi), cv::Mat(), kps, des);

      // Offset keypoints
      const size_t vacancy = max_per_cell - feature_grid_count(&grid, cell_idx);
      if (vacancy <= 0) {
        continue;
      }

      for (int i = 0; i < std::min(kps.size(), vacancy); i++) {
        cv::KeyPoint kp = kps[i];
        kp.pt.x += x;
        kp.pt.y += y;
        kps_all.push_back(kp);
        // des_all.push_back(des[i, :]);
      }

      // Update cell_idx
      cell_idx += 1;
    }
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

//////////
// MAIN //
//////////

int main() {
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
