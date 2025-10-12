

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
void ransac(const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            const undistort_func_t cam0_undist,
            const undistort_func_t cam1_undist,
            const real_t cam0_params[8],
            const real_t cam1_params[8],
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.5,
            const double confidence = 0.99) {
  assert(kps0.size() > 0 && kps1.size() > 0);
  assert(kps0.size() == kps1.size());
  assert(cam0_undist && cam1_undist);
  assert(cam0_params && cam1_params);

  // Undistort image points
  std::vector<cv::Point2f> pts0 = kps2pts(kps0);
  std::vector<cv::Point2f> pts1 = kps2pts(kps1);
  for (size_t i = 0; i < pts0.size(); i++) {
    const real_t z0_in[2] = {pts0[i].x, pts0[i].y};
    const real_t z1_in[2] = {pts1[i].x, pts1[i].y};

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    cam0_undist(cam0_params, z0_in, z0);
    cam1_undist(cam1_params, z1_in, z1);

    pts0[i].x = z0[0];
    pts0[i].y = z0[1];

    pts1[i].x = z1[0];
    pts1[i].y = z1[1];
  }

  // Perform ransac
  const int method = cv::FM_RANSAC;
  cv::findFundamentalMat(pts0,
                         pts1,
                         method,
                         reproj_threshold,
                         confidence,
                         inliers);
}

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
void ransac(const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            const undistort_func_t undist_func,
            const real_t cam_params[8],
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.75,
            const double confidence = 0.99) {
  ransac(kps0,
         kps1,
         undist_func,
         undist_func,
         cam_params,
         cam_params,
         inliers,
         reproj_threshold,
         confidence);
}
