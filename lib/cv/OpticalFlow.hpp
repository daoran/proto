

/**
 * Track keypoints `pts_i` from image `img_i` to image `img_j` using optical
 * flow. Returns a tuple of `(pts_i, pts_j, inliers)` points in image i, j and a
 * vector of inliers.
 */
void optflow_track(const cv::Mat &img_i,
                   const cv::Mat &img_j,
                   const std::vector<cv::KeyPoint> &kps_i,
                   std::vector<cv::KeyPoint> &kps_j,
                   std::vector<uchar> &inliers,
                   const int patch_size = 50,
                   const int max_iter = 50,
                   const int max_level = 3,
                   const real_t epsilon = 0.001,
                   const bool debug = false) {
  // Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
  std::vector<cv::Point2f> pts_i = kps2pts(kps_i);
  std::vector<cv::Point2f> pts_j = kps2pts(kps_j);

  // Track features from img i to img j
  const auto win_size = cv::Size(patch_size, patch_size);
  const auto crit_type = cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS;
  const auto crit = cv::TermCriteria(crit_type, max_iter, epsilon);
  const auto errs = cv::noArray();
  std::vector<uchar> optflow_inliers;
  cv::calcOpticalFlowPyrLK(img_i,
                           img_j,
                           pts_i,
                           pts_j,
                           optflow_inliers,
                           errs,
                           win_size,
                           max_level,
                           crit);

  // Make sure keypoints are within image dimensions
  std::vector<bool> bound_inliers;
  const int img_h = img_i.rows;
  const int img_w = img_i.cols;
  for (const auto &p : pts_j) {
    const auto x_ok = p.x >= 0 && p.x <= img_w;
    const auto y_ok = p.y >= 0 && p.y <= img_h;
    bound_inliers.push_back((x_ok && y_ok));
  }

  // Write out final inliers
  assert(pts_i.size() == optflow_inliers.size());
  assert(pts_i.size() == bound_inliers.size());

  kps_j.clear();
  inliers.clear();
  for (size_t i = 0; i < pts_i.size(); i++) {
    kps_j.emplace_back(pts_j[i],
                       kps_i[i].size,
                       kps_i[i].angle,
                       kps_i[i].response,
                       kps_i[i].octave,
                       kps_i[i].class_id);
    inliers.push_back(optflow_inliers[i] && bound_inliers[i]);
  }

  // Debug
  bool quit = false;
  while (debug && quit == false) {
    // Draw properties
    const int radius = 2;
    const cv::Scalar red{0, 0, 255};

    // Setup
    cv::Mat viz_i;
    cv::Mat viz_j;
    cv::cvtColor(img_i, viz_i, cv::COLOR_GRAY2RGB);
    cv::cvtColor(img_j, viz_j, cv::COLOR_GRAY2RGB);

    // Draw KeyPoints
    for (size_t n = 0; n < kps_i.size(); n++) {
      if (inliers[n] == 0) {
        continue;
      }

      auto pt_i = kps_i[n].pt;
      auto pt_j = kps_j[n].pt;
      cv::circle(viz_i, pt_i, radius, red, -1);
      cv::circle(viz_j, pt_j, radius, red, -1);
    }

    // Stitch images horizontally
    cv::Mat viz;
    cv::hconcat(viz_i, viz_j, viz);

    // Draw lines
    for (size_t n = 0; n < kps_i.size(); n++) {
      auto pt_i = kps_i[n].pt;
      auto pt_j = kps_j[n].pt;
      pt_j.x += viz_i.cols;
      const auto line = cv::LINE_AA;
      cv::line(viz, pt_i, pt_j, red, 1, line);
    }

    // Show
    cv::imshow("Optical-Flow Tracking", viz);
    if (cv::waitKey(0) == 'q') {
      quit = true;
    }
  }
}
