#include "prototype/vision/feature2d/image_processor.hpp"

namespace prototype {

void grid_features_extract_all(const GridFeatures &grid_features,
                               std::vector<FeatureIDType> &ids,
                               std::vector<int> &lifetimes,
                               std::vector<cv::Point2f> &cam0_points,
                               std::vector<cv::Point2f> &cam1_points) {
  for (const auto &item : grid_features) {
    for (const auto &feature : item.second) {
      ids.push_back(feature.id);
      lifetimes.push_back(feature.lifetime);
      cam0_points.push_back(feature.cam0_point);
      cam1_points.push_back(feature.cam1_point);
    }
  }
}

void grid_features_extract_ids(const GridFeatures &grid_features,
                               std::vector<FeatureIDType> &ids) {
  for (const auto &item : grid_features) {
    for (const auto &feature : item.second) {
      ids.push_back(feature.id);
    }
  }
}

void grid_features_extract_points(const GridFeatures &grid_features,
                                  std::vector<cv::Point2f> &cam0_points,
                                  std::vector<cv::Point2f> &cam1_points) {
  for (const auto &item : grid_features) {
    for (const auto &feature : item.second) {
      cam0_points.push_back(feature.cam0_point);
      cam1_points.push_back(feature.cam1_point);
    }
  }
}

void grid_features_extract_points(
    const GridFeatures &grid_features,
    std::map<FeatureIDType, cv::Point2f> &cam0_points,
    std::map<FeatureIDType, cv::Point2f> &cam1_points) {
  for (const auto &features : grid_features) {
    for (const auto &feature : features.second) {
      cam0_points[feature.id] = feature.cam0_point;
      cam1_points[feature.id] = feature.cam1_point;
    }
  }
}

void klt_track_features(const std::vector<cv::Mat> &cam0_img_pyramid,
                        const std::vector<cv::Mat> &cam1_img_pyramid,
                        const std::vector<cv::Point2f> &cam0_points,
                        const std::vector<cv::Point2f> &cam1_points,
                        const int max_iterations,
                        const int track_precision,
                        const int patch_size,
                        const int pyramid_levels,
                        const int image_width,
                        const int image_height,
                        std::vector<uchar> &inlier_markers) {
  // Track features using LK optical flow method.
  assert(cam0_img_pyramid.size() > 0);
  assert(cam1_img_pyramid.size() > 0);
  const auto term_criteria =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                       max_iterations,
                       track_precision);
  cv::calcOpticalFlowPyrLK(cam0_img_pyramid,
                           cam1_img_pyramid,
                           cam0_points,
                           cam1_points,
                           inlier_markers,
                           cv::noArray(),
                           cv::Size(patch_size, patch_size),
                           pyramid_levels,
                           term_criteria,
                           cv::OPTFLOW_UPROTOTYPE_VISION_INITIAL_FLOW);

  // Mark those tracked points out of the image region
  // as untracked.
  assert(image_width > 0);
  assert(image_height > 0);
  for (size_t i = 0; i < cam1_points.size(); ++i) {
    if (inlier_markers[i] == 0) {
      continue;
    }
    if (cam1_points[i].y < 0 || cam1_points[i].y > image_height - 1 ||
        cam1_points[i].x < 0 || cam1_points[i].x > image_width - 1) {
      inlier_markers[i] = 0;
    }
  }
}

cv::Mat draw_tracked_stereo_points(
    const cv::Mat &stereo_image,
    const std::vector<FeatureIDType> &prev_ids,
    std::map<FeatureIDType, cv::Point2f> &prev_cam0_points,
    std::map<FeatureIDType, cv::Point2f> &prev_cam1_points,
    std::map<FeatureIDType, cv::Point2f> &curr_cam0_points,
    std::map<FeatureIDType, cv::Point2f> &curr_cam1_points,
    const cv::Scalar color) {
  // Create output image
  const int image_height = stereo_image.rows;
  const int image_width = stereo_image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);
  if (stereo_image.channels() == 1) {
    cv::cvtColor(stereo_image, out_image, CV_GRAY2RGB);
  } else {
    out_image = stereo_image.clone();
  }

  // Draw tracked features
  const cv::Point2f offset(0.0, image_height / 2.0);
  for (const auto &id : prev_ids) {
    if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
        curr_cam0_points.find(id) != curr_cam0_points.end()) {
      const cv::Point2f prev_pt0 = prev_cam0_points.at(id);
      const cv::Point2f prev_pt1 = prev_cam1_points.at(id) + offset;
      const cv::Point2f curr_pt0 = curr_cam0_points.at(id);
      const cv::Point2f curr_pt1 = curr_cam1_points.at(id) + offset;

      cv::circle(out_image, curr_pt0, 3, color, -1);
      cv::circle(out_image, curr_pt1, 3, color, -1);
      cv::line(out_image, prev_pt0, curr_pt0, color, 1);
      cv::line(out_image, prev_pt1, curr_pt1, color, 1);

      prev_cam0_points.erase(id);
      prev_cam1_points.erase(id);
      curr_cam0_points.erase(id);
      curr_cam1_points.erase(id);
    }
  }

  return out_image;
}

cv::Mat
draw_new_stereo_points(const cv::Mat &stereo_image,
                       const std::map<FeatureIDType, cv::Point2f> &cam0_points,
                       const std::map<FeatureIDType, cv::Point2f> &cam1_points,
                       const cv::Scalar &color) {
  // Create output image
  const int image_height = stereo_image.rows;
  const int image_width = stereo_image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);
  if (stereo_image.channels() == 1) {
    cv::cvtColor(stereo_image, out_image, CV_GRAY2RGB);
  } else {
    out_image = stereo_image.clone();
  }

  // Draw points
  cv::Point2f offset(0, image_height / 2.0);
  for (const auto &cam0_pt : cam0_points) {
    const FeatureIDType f_id = cam0_pt.first;
    const cv::Point2f pt0 = cam0_pt.second;
    const cv::Point2f pt1 = cam1_points.at(f_id);
    cv::circle(out_image, pt0, 3, color, -1);
    cv::circle(out_image, pt1 + offset, 3, color, -1);
  }

  return out_image;
}

ImageProcessor::ImageProcessor() {}

ImageProcessor::~ImageProcessor() {}

int ImageProcessor::configure(const std::string &config_file) {
  ConfigParser parser;
  // -- Camera 0 properties
  parser.addParam("cam0.camera_model", &this->cam0.camera_model);
  parser.addParam("cam0.distortion_model", &this->cam0.distortion_model);
  parser.addParam("cam0.distortion_coeffs", &this->cam0.distortion_coeffs);
  parser.addParam("cam0.intrinsics", &this->cam0.intrinsics);
  parser.addParam("cam0.resolution", &this->cam0.resolution);
  // -- Camera 1 properties
  parser.addParam("cam1.camera_model", &this->cam1.camera_model);
  parser.addParam("cam1.distortion_model", &this->cam1.distortion_model);
  parser.addParam("cam1.distortion_coeffs", &this->cam1.distortion_coeffs);
  parser.addParam("cam1.intrinsics", &this->cam1.intrinsics);
  parser.addParam("cam1.resolution", &this->cam1.resolution);
  // -- Camera / IMU extrinsics
  parser.addParam("T_imu_cam0", &this->T_imu_cam0);
  parser.addParam("T_cam1_cam0", &this->T_cam1_cam0);
  // -- Detector settings
  parser.addParam("fast.threshold", &this->fast_threshold);
  parser.addParam("fast.max_corners", &this->fast_max_corners);
  parser.addParam("fast.nonmax_suppression",
                  &this->fast_nonmax_suppression,
                  true);
  parser.addParam("fast.debug", &this->fast_debug, true);
  // -- Tracking setttings
  parser.addParam("grid_rows", &this->grid_rows);
  parser.addParam("grid_cols", &this->grid_cols);
  parser.addParam("pyramid_levels", &this->pyramid_levels);
  parser.addParam("patch_size", &this->patch_size);
  parser.addParam("max_iterations", &this->max_iterations);
  parser.addParam("track_precision", &this->track_precision);
  parser.addParam("ransac_threshold", &this->ransac_threshold);
  parser.addParam("stereo_threshold", &this->stereo_threshold);
  // -- Load config
  if (parser.load(config_file) != 0) {
    return -1;
  }
  this->nb_grid_cells = this->grid_rows * this->grid_cols;

  return 0;
}

void ImageProcessor::createImagePyramids(const cv::Mat &cam0_img,
                                         const cv::Mat &cam1_img) {
  cv::buildOpticalFlowPyramid(cam0_img,
                              this->curr_cam0_img_pyramid,
                              cv::Size(this->patch_size, this->patch_size),
                              this->pyramid_levels,
                              true,
                              cv::BORDER_REFLECT_101,
                              cv::BORDER_CONSTANT,
                              false);

  cv::buildOpticalFlowPyramid(cam1_img,
                              this->curr_cam1_img_pyramid,
                              cv::Size(this->patch_size, this->patch_size),
                              this->pyramid_levels,
                              true,
                              cv::BORDER_REFLECT_101,
                              cv::BORDER_CONSTANT,
                              false);
}

std::vector<cv::Point2f> ImageProcessor::projectPointsFromCam0ToCam1(
    const std::vector<cv::Point2f> &cam0_points, const mat4_t &T_cam1_cam0) {
  auto R_cam0_cam1 = T_cam1_cam0.block(0, 0, 3, 3);
  auto cam0_points_ud = this->cam0.undistortPoints(cam0_points, R_cam0_cam1);
  auto cam1_points = this->cam1.distortPoints(cam0_points_ud);
  return cam1_points;
}

void ImageProcessor::initialize(const cv::Mat &cam0_img) {
  // Set image width and height
  this->image_width = cam0_img.cols;
  this->image_height = cam0_img.rows;

  // Detect new features on the cam0 image
  auto keypoints = grid_fast(cam0_img,
                             this->fast_max_corners,
                             this->grid_rows,
                             this->grid_cols,
                             this->fast_threshold,
                             this->fast_nonmax_suppression);

  // Find the stereo matched points for the newly detected features.
  // -- Convert cv::Keypoint to cv::Point2f
  std::vector<cv::Point2f> cam0_points(keypoints.size());
  for (size_t i = 0; i < keypoints.size(); ++i) {
    cam0_points[i] = keypoints[i].pt;
  }
  // -- Peform stereo match
  std::vector<cv::Point2f> cam1_points(0);
  std::vector<unsigned char> inlier_markers(0);
  this->stereoMatch(cam0_points, cam1_points, inlier_markers);
  // -- Filter away outliers
  std::vector<cv::Point2f> cam0_inliers(0);
  std::vector<cv::Point2f> cam1_inliers(0);
  std::vector<float> response_inliers(0);
  for (size_t i = 0; i < inlier_markers.size(); i++) {
    if (inlier_markers[i] == 0) {
      continue;
    }
    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(keypoints[i].response);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  const int grid_width = cam0_img.cols / this->grid_cols;
  const int grid_height = cam0_img.rows / this->grid_rows;
  // -- Initialize grid_new_features
  for (int i = 0; i < this->nb_grid_cells; i++) {
    grid_new_features[i] = std::vector<FeatureMetaData>(0);
  }
  // -- Add features into corresponding grid cells
  for (size_t i = 0; i < cam0_inliers.size(); i++) {
    const cv::Point2f &cam0_point = cam0_inliers[i];
    const cv::Point2f &cam1_point = cam1_inliers[i];
    const float &response = response_inliers[i];

    const int row = static_cast<int>(cam0_point.y / grid_height);
    const int col = static_cast<int>(cam0_point.x / grid_width);
    const int idx = row * this->grid_cols + col;
    grid_new_features[idx].emplace_back(cam0_point, cam1_point, response);
  }
  // -- Sort the new features in each grid based on its response.
  for (auto &f : grid_new_features) {
    std::sort(f.second.begin(), f.second.end(), &feature_compare_by_response);
  }
  // -- Collect new features within each grid cell with high response.
  for (int i = 0; i < this->nb_grid_cells; i++) {
    std::vector<FeatureMetaData> &cell_features = curr_features[i];
    std::vector<FeatureMetaData> &new_features = grid_new_features[i];

    for (int k = 0;
         k < this->min_cell_features && k < (int) new_features.size();
         k++) {
      new_features[k].id = next_feature_id++;
      new_features[k].lifetime = 1;
      cell_features.push_back(new_features[k]);
    }
  }

  // Update status flag
  this->initialized = true;
}

void ImageProcessor::stereoMatch(const std::vector<cv::Point2f> &cam0_points,
                                 std::vector<cv::Point2f> &cam1_points,
                                 std::vector<unsigned char> &inlier_markers) {
  // Pre-check
  if (cam0_points.size() == 0) {
    return;
  }

  // Initialize cam1_points by projecting cam0_points to cam1 using the
  // rotation from stereo extrinsics
  assert(this->T_cam1_cam0.isApprox(mat4_t::Zero()) == false);
  if (cam1_points.size() == 0) {
    cam1_points =
        this->projectPointsFromCam0ToCam1(cam0_points, this->T_cam1_cam0);
  }

  // Track features using LK optical flow method.
  assert(this->image_width > 0);
  assert(this->image_height > 0);
  klt_track_features(this->curr_cam0_img_pyramid,
                     this->curr_cam1_img_pyramid,
                     cam0_points,
                     cam1_points,
                     this->max_iterations,
                     this->track_precision,
                     this->patch_size,
                     this->pyramid_levels,
                     this->image_width,
                     this->image_height,
                     inlier_markers);

  // Use essential matrix for outlier rejection
  essential_matrix_outlier_rejection(this->cam0,
                                     this->cam1,
                                     this->T_cam1_cam0,
                                     cam0_points,
                                     cam1_points,
                                     this->stereo_threshold,
                                     inlier_markers);
}

void ImageProcessor::stereoCallback(const cv::Mat &cam0_img,
                                    const cv::Mat &cam1_img,
                                    const long ts) {
  // Build the image pyramids once since they're used at multiple places
  this->createImagePyramids(cam0_img, cam1_img);

  // Detect features in the first frame.
  if (this->initialized == false) {
    this->initialize(cam0_img);

  } else {
    // Track the feature in the previous image
    this->trackFeatures(ts);

    // Add new features into the current image
    this->addNewFeatures(cam0_img);

    // Prune features into the current image.
    this->pruneGridFeatures();
  }
  this->drawFeatures(cam0_img, cam1_img);

  // Update the previous image and previous features.
  this->prev_cam0_img = CameraFrame{cam0_img.clone(), ts};
  this->prev_cam1_img = CameraFrame{cam1_img.clone(), ts};
  this->prev_features = this->curr_features;
  std::swap(this->prev_cam0_img_pyramid, this->curr_cam0_img_pyramid);

  // Initialize the current features to empty vectors.
  // this->curr_features.reset(new GridFeatures());
  this->curr_features.clear();
  for (int i = 0; i < this->nb_grid_cells; ++i) {
    this->curr_features[i] = std::vector<FeatureMetaData>(0);
  }
}

void ImageProcessor::trackFeatures(const long ts) {
  // Compute a rough relative rotation which takes a vector from the previous
  // frame to the current frame
  cv::Matx33f cam0_R_p_c;
  cv::Matx33f cam1_R_p_c;
  this->integrateImuData(ts, cam0_R_p_c, cam1_R_p_c);
  std::cout << cam0_R_p_c << std::endl;
  std::cout << cam1_R_p_c << std::endl;

  // Organize the features in the previous image.
  std::vector<FeatureIDType> prev_ids(0);
  std::vector<int> prev_lifetime(0);
  std::vector<cv::Point2f> prev_cam0_points(0);
  std::vector<cv::Point2f> prev_cam1_points(0);
  grid_features_extract_all(this->prev_features,
                            prev_ids,
                            prev_lifetime,
                            prev_cam0_points,
                            prev_cam1_points);

  // Abort tracking if there is no features in the previous frame
  if (prev_ids.size() == 0) {
    return;
  }

  // Predict where points are going to be
  std::vector<cv::Point2f> curr_cam0_points(0);
  this->predictFeatures(prev_cam0_points,
                        cam0_R_p_c,
                        convert(this->cam0.intrinsics),
                        curr_cam0_points);

  // Track features using LK optical flow method.
  std::vector<unsigned char> track_inliers(0);
  assert(this->prev_cam0_img_pyramid.size() > 0);
  klt_track_features(this->prev_cam0_img_pyramid,
                     this->curr_cam0_img_pyramid,
                     prev_cam0_points,
                     curr_cam0_points,
                     this->max_iterations,
                     this->track_precision,
                     this->patch_size,
                     this->pyramid_levels,
                     this->image_width,
                     this->image_height,
                     track_inliers);

  // Collect the tracked points.
  std::vector<FeatureIDType> prev_tracked_ids(0);
  std::vector<int> prev_tracked_lifetime(0);
  std::vector<cv::Point2f> prev_tracked_cam0_points(0);
  std::vector<cv::Point2f> prev_tracked_cam1_points(0);
  std::vector<cv::Point2f> curr_tracked_cam0_points(0);

  this->removeUnmarkedElements(prev_ids, track_inliers, prev_tracked_ids);
  this->removeUnmarkedElements(prev_lifetime,
                               track_inliers,
                               prev_tracked_lifetime);
  this->removeUnmarkedElements(prev_cam0_points,
                               track_inliers,
                               prev_tracked_cam0_points);
  this->removeUnmarkedElements(prev_cam1_points,
                               track_inliers,
                               prev_tracked_cam1_points);
  this->removeUnmarkedElements(curr_cam0_points,
                               track_inliers,
                               curr_tracked_cam0_points);

  // Number of features left after tracking.
  after_tracking = curr_tracked_cam0_points.size();

  // Outlier removal involves three steps, which forms a close
  // loop between the previous and current frames of cam0 (left)
  // and cam1 (right). Assuming the stereo matching between the
  // previous cam0 and cam1 images are correct, the three steps are:
  //
  // prev frames cam0 ----------> cam1
  //              |                |
  //              |ransac          |ransac
  //              |   stereo match |
  // curr frames cam0 ----------> cam1
  //
  // 1) Stereo matching between current images of cam0 and cam1.
  // 2) RANSAC between previous and current images of cam0.
  // 3) RANSAC between previous and current images of cam1.
  //
  // For Step 3, tracking between the images is no longer needed.
  // The stereo matching results are directly used in the RANSAC.

  // Step 1: stereo matching.
  std::vector<cv::Point2f> curr_cam1_points(0);
  std::vector<unsigned char> match_inliers(0);
  this->stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

  std::vector<FeatureIDType> prev_matched_ids(0);
  std::vector<int> prev_matched_lifetime(0);
  std::vector<cv::Point2f> prev_matched_cam0_points(0);
  std::vector<cv::Point2f> prev_matched_cam1_points(0);
  std::vector<cv::Point2f> curr_matched_cam0_points(0);
  std::vector<cv::Point2f> curr_matched_cam1_points(0);

  this->removeUnmarkedElements(prev_tracked_ids,
                               match_inliers,
                               prev_matched_ids);
  this->removeUnmarkedElements(prev_tracked_lifetime,
                               match_inliers,
                               prev_matched_lifetime);
  this->removeUnmarkedElements(prev_tracked_cam0_points,
                               match_inliers,
                               prev_matched_cam0_points);
  this->removeUnmarkedElements(prev_tracked_cam1_points,
                               match_inliers,
                               prev_matched_cam1_points);
  this->removeUnmarkedElements(curr_tracked_cam0_points,
                               match_inliers,
                               curr_matched_cam0_points);
  this->removeUnmarkedElements(curr_cam1_points,
                               match_inliers,
                               curr_matched_cam1_points);

  // Number of features left after stereo matching.
  after_matching = curr_matched_cam0_points.size();

  // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1.
  // std::vector<int> cam0_ransac_inliers(prev_matched_cam0_points.size(), 1);
  std::vector<int> cam0_ransac_inliers(0);
  two_point_ransac(prev_matched_cam0_points,
                   curr_matched_cam0_points,
                   cam0_R_p_c,
                   this->cam0,
                   this->ransac_threshold,
                   0.99,
                   cam0_ransac_inliers);

  // std::vector<int> cam1_ransac_inliers(prev_matched_cam1_points.size(), 1);
  std::vector<int> cam1_ransac_inliers(0);
  two_point_ransac(prev_matched_cam1_points,
                   curr_matched_cam1_points,
                   cam1_R_p_c,
                   this->cam1,
                   this->ransac_threshold,
                   0.99,
                   cam1_ransac_inliers);

  // Size of grid
  const int grid_height = this->image_height / this->grid_rows;
  const int grid_width = this->image_width / this->grid_cols;
  for (size_t i = 0; i < cam0_ransac_inliers.size(); i++) {
    if (cam0_ransac_inliers[i] == 0 || cam1_ransac_inliers[i] == 0) {
      continue;
    }
    int row = static_cast<int>(curr_matched_cam0_points[i].y / grid_height);
    int col = static_cast<int>(curr_matched_cam0_points[i].x / grid_width);
    int idx = row * this->grid_cols + col;
    curr_features[idx].emplace_back(prev_matched_ids[i],
                                    curr_matched_cam0_points[i],
                                    curr_matched_cam1_points[i],
                                    ++prev_matched_lifetime[i],
                                    0);
  }

  // // Compute the tracking rate.
  // int prev_feature_num = 0;
  // for (const auto &item : prev_features) {
  //   prev_feature_num += item.second.size();
  // }
  //
  // int curr_feature_num = 0;
  // for (const auto &item : curr_features) {
  //   curr_feature_num += item.second.size();
  // }
}

void ImageProcessor::addNewFeatures(const cv::Mat &cam0_img) {
  // Detect new features.
  auto new_features = grid_fast(cam0_img,
                                this->fast_max_corners,
                                this->grid_rows,
                                this->grid_cols,
                                this->fast_threshold,
                                this->fast_nonmax_suppression);

  // Filter out new features that are close to existing features
  // -- Create mask to denote which pixels are no-go
  cv::Mat mask(cam0_img.rows, cam0_img.cols, CV_8U, cv::Scalar(0));
  for (const auto &features : this->curr_features) {
    for (const auto &feature : features.second) {
      const int x = static_cast<int>(feature.cam0_point.x);
      const int y = static_cast<int>(feature.cam0_point.y);

      int up_lim = y - 2;
      int bottom_lim = y + 3;
      int left_lim = x - 2;
      int right_lim = x + 3;
      if (up_lim < 0) {
        up_lim = 0;
      }
      if (bottom_lim > cam0_img.rows) {
        bottom_lim = cam0_img.rows;
      }
      if (left_lim < 0) {
        left_lim = 0;
      }
      if (right_lim > cam0_img.cols) {
        right_lim = cam0_img.cols;
      }

      cv::Range row_range(up_lim, bottom_lim);
      cv::Range col_range(left_lim, right_lim);
      mask(row_range, col_range) = 1;
    }
  }
  // -- Loop through new points and make sure they're not near existing ones
  std::vector<cv::KeyPoint> keypoints;
  for (const auto &kp : new_features) {
    const int x = static_cast<int>(kp.pt.x);
    const int y = static_cast<int>(kp.pt.y);
    if (mask.at<int>(x, y) == 0) {
      keypoints.emplace_back(kp);
    }
  }
  new_features = keypoints;

  // Collect the new detected features based on the grid. Select the ones with
  // top response within each grid afterwards.
  const int cell_width = cam0_img.cols / this->grid_cols;
  const int cell_height = cam0_img.rows / this->grid_rows;
  std::vector<std::vector<cv::KeyPoint>> new_feature_sieve(this->nb_grid_cells);
  for (const auto &feature : new_features) {
    const int row = static_cast<int>(feature.pt.y / cell_height);
    const int col = static_cast<int>(feature.pt.x / cell_width);
    new_feature_sieve[row * this->grid_cols + col].push_back(feature);
  }

  new_features.clear();
  for (auto &item : new_feature_sieve) {
    if (item.size() > static_cast<size_t>(this->max_cell_features)) {
      std::sort(item.begin(), item.end(), &keypoint_compare_by_response);
      item.erase(item.begin() + this->max_cell_features, item.end());
    }
    new_features.insert(new_features.end(), item.begin(), item.end());
  }

  // Convert cv::KeyPoint to cv::Point2f
  std::vector<cv::Point2f> cam0_points(new_features.size());
  for (size_t i = 0; i < new_features.size(); i++) {
    cam0_points[i] = new_features[i].pt;
  }

  // Find the stereo matched points for the newly detected features
  std::vector<cv::Point2f> cam1_points(0);
  std::vector<unsigned char> inlier_markers(0);
  this->stereoMatch(cam0_points, cam1_points, inlier_markers);

  std::vector<cv::Point2f> cam0_inliers(0);
  std::vector<cv::Point2f> cam1_inliers(0);
  std::vector<float> response_inliers(0);
  for (size_t i = 0; i < inlier_markers.size(); i++) {
    if (inlier_markers[i] == 0) {
      continue;
    }

    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(new_features[i].response);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  for (int i = 0; i < this->nb_grid_cells; i++) {
    grid_new_features[i] = std::vector<FeatureMetaData>(0);
  }
  for (size_t i = 0; i < cam0_inliers.size(); ++i) {
    const cv::Point2f &cam0_point = cam0_inliers[i];
    const cv::Point2f &cam1_point = cam1_inliers[i];
    const float &response = response_inliers[i];

    int row = static_cast<int>(cam0_point.y / cell_height);
    int col = static_cast<int>(cam0_point.x / cell_width);
    int idx = row * this->grid_cols + col;
    grid_new_features[idx].emplace_back(cam0_point, cam1_point, response);
  }

  // Sort the new features in each grid based on its response.
  for (auto &item : grid_new_features) {
    std::sort(item.second.begin(),
              item.second.end(),
              &feature_compare_by_response);
  }

  // Collect new features within each grid with high response.
  for (int i = 0; i < this->nb_grid_cells; i++) {
    std::vector<FeatureMetaData> &cell_features = this->curr_features[i];
    std::vector<FeatureMetaData> &new_cell_features = grid_new_features[i];

    if (cell_features.size() >= static_cast<size_t>(this->min_cell_features)) {
      continue;
    }

    size_t vacancy_num =
        static_cast<size_t>(this->min_cell_features - cell_features.size());
    for (size_t k = 0; k < vacancy_num && k < new_cell_features.size(); k++) {
      cell_features.push_back(new_cell_features[k]);
      cell_features.back().id = next_feature_id++;
      cell_features.back().lifetime = 1;
    }
  }
}

void ImageProcessor::pruneGridFeatures() {
  for (auto &item : this->curr_features) {
    auto &grid_features = item.second;
    // Continue if the number of features in this grid does not exceed the
    // upper bound
    if (grid_features.size() <= static_cast<size_t>(this->max_cell_features)) {
      continue;
    }

    std::sort(grid_features.begin(),
              grid_features.end(),
              &feature_compare_by_lifetime);
    grid_features.erase(grid_features.begin() + this->max_cell_features,
                        grid_features.end());
  }
  return;
}

void ImageProcessor::integrateImuData(const long curr_ts,
                                      cv::Matx33f &cam0_R_p_c,
                                      cv::Matx33f &cam1_R_p_c) {
  // Find the start and the end limit within the imu msg buffer
  assert(this->prev_cam0_img.ts != 0);
  auto begin_iter = imu_buffer.begin();
  while (begin_iter != imu_buffer.end()) {
    if ((begin_iter->ts - prev_cam0_img.ts) * 1e-9 < -0.01) {
      ++begin_iter;
    } else {
      break;
    }
  }

  auto end_iter = begin_iter;
  while (end_iter != imu_buffer.end()) {
    if ((end_iter->ts - this->prev_cam0_img.ts) * 1e-9 < 0.005) {
      ++end_iter;
    } else {
      break;
    }
  }

  // Compute the mean angular velocity in the IMU frame.
  cv::vec3_tf mean_ang_vel(0.0, 0.0, 0.0);
  for (auto iter = begin_iter; iter < end_iter; ++iter) {
    mean_ang_vel += cv::vec3_tf(iter->w_B(0), iter->w_B(1), iter->w_B(2));
  }
  if (end_iter - begin_iter > 0) {
    mean_ang_vel *= 1.0f / (end_iter - begin_iter);
  }

  // Transform the mean angular velocity from the IMU frame to the cam0 and
  // cam1 frames
  const mat4_t T_cam0_imu = this->T_imu_cam0.inverse();
  const mat4_t T_cam1_imu = this->T_cam1_cam0 * T_cam0_imu;
  const cv::Matx33f R_cam0_imu = convert(T_cam0_imu.block(0, 0, 3, 3));
  const cv::Matx33f R_cam1_imu = convert(T_cam1_imu.block(0, 0, 3, 3));
  cv::vec3_tf cam0_mean_ang_vel = R_cam0_imu.t() * mean_ang_vel;
  cv::vec3_tf cam1_mean_ang_vel = R_cam1_imu.t() * mean_ang_vel;

  // Compute the relative rotation.
  assert(curr_ts != 0);
  assert(this->prev_cam0_img.ts != 0);
  double dt = (curr_ts - this->prev_cam0_img.ts) * 1e-9;
  cv::Rodrigues(cam0_mean_ang_vel * dt, cam0_R_p_c);
  cv::Rodrigues(cam1_mean_ang_vel * dt, cam1_R_p_c);
  cam0_R_p_c = cam0_R_p_c.t();
  cam1_R_p_c = cam1_R_p_c.t();

  // Clear IMU buffer
  imu_buffer.erase(imu_buffer.begin(), end_iter);
}

void ImageProcessor::predictFeatures(
    const std::vector<cv::Point2f> &input_pts,
    const cv::Matx33f &R_p_c,
    const cv::Vec4d &intrinsics,
    std::vector<cv::Point2f> &compensated_pts) {
  // Pre-check
  if (input_pts.size() == 0) {
    compensated_pts.clear();
    return;
  }

  // Form intrinsic matrix
  // clang-format off
  cv::Matx33f K(intrinsics[0], 0.0, intrinsics[2],
                0.0, intrinsics[1], intrinsics[3],
                0.0, 0.0, 1.0);
  // clang-format on

  // Form homography matrix
  cv::Matx33f H = K * R_p_c * K.inv();

  // Use homography to predict where points will be
  compensated_pts.resize(input_pts.size());
  for (size_t i = 0; i < input_pts.size(); i++) {
    cv::vec3_tf p1(input_pts[i].x, input_pts[i].y, 1.0f);
    cv::vec3_tf p2 = H * p1;
    compensated_pts[i].x = p2[0] / p2[2];
    compensated_pts[i].y = p2[1] / p2[2];
  }

  return;
}

void ImageProcessor::rescalePoints(std::vector<cv::Point2f> &pts1,
                                   std::vector<cv::Point2f> &pts2,
                                   float &scaling_factor) {
  // Calculate scaling factor
  scaling_factor = 0.0f;
  for (size_t i = 0; i < pts1.size(); i++) {
    scaling_factor += sqrt(pts1[i].dot(pts1[i]));
    scaling_factor += sqrt(pts2[i].dot(pts2[i]));
  }
  scaling_factor = (pts1.size() + pts2.size()) / scaling_factor * sqrt(2.0f);

  // Rescale points
  for (size_t i = 0; i < pts1.size(); i++) {
    pts1[i] *= scaling_factor;
    pts2[i] *= scaling_factor;
  }
}

void ImageProcessor::updateFeatureLifetime() {
  int nb_cells = this->grid_rows * this->grid_cols;

  for (int i = 0; i < nb_cells; i++) {
    std::vector<FeatureMetaData> &features = curr_features[i];

    for (const auto &feature : features) {
      if (this->feature_lifetime.find(feature.id) == feature_lifetime.end()) {
        this->feature_lifetime[feature.id] = 1;
      } else {
        this->feature_lifetime[feature.id]++;
      }
    }
  }
}

void ImageProcessor::printFeatureLifetimeStatistics() {
  std::map<int, int> lifetime_statistics;

  for (const auto &data : feature_lifetime) {
    if (lifetime_statistics.find(data.second) == lifetime_statistics.end()) {
      lifetime_statistics[data.second] = 1;
    } else {
      lifetime_statistics[data.second]++;
    }
  }

  for (const auto &data : lifetime_statistics) {
    cout << data.first << " : " << data.second << endl;
  }
}

void ImageProcessor::imuCallback(const vec3_t &a_m,
                                 const vec3_t &w_m,
                                 const long ts) {
  // Pre-check
  if (this->initialized == false) {
    return;
  }
  imu_buffer.emplace_back(a_m, w_m, ts);
}

cv::Mat draw_grid(const cv::Mat &image,
                  const int grid_rows,
                  const int grid_cols) {
  // Create output image
  const int image_height = image.rows;
  const int image_width = image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);
  if (image.channels() == 1) {
    cv::cvtColor(image, out_image, CV_GRAY2RGB);
  } else {
    out_image = image.clone();
  }

  // Draw grids on the image
  const int grid_height = image.rows / grid_rows;
  const int grid_width = image.cols / grid_cols;
  const cv::Scalar line_color(255, 0, 0);
  // -- Draw horizontal lines
  for (int i = 1; i < (grid_rows * 2); i++) {
    cv::Point pt1(0, i * grid_height);
    cv::Point pt2(image_width, i * grid_height);
    cv::line(out_image, pt1, pt2, line_color);
  }
  // -- Draw vertical lines
  for (int i = 1; i < grid_cols; i++) {
    cv::Point pt1(i * grid_width, 0);
    cv::Point pt2(i * grid_width, image_height);
    cv::line(out_image, pt1, pt2, line_color);
  }

  return out_image;
}

void ImageProcessor::drawFeatures(const cv::Mat &cam0_img,
                                  const cv::Mat &cam1_img) {
  // Draw grids on the image
  cv::Mat out0_img = draw_grid(cam0_img, this->grid_rows, this->grid_cols);
  cv::Mat out1_img = draw_grid(cam1_img, this->grid_rows, this->grid_cols);
  const int img_height = cam0_img.rows;
  const int img_width = cam0_img.cols;
  cv::Mat out_img(img_height, img_width, CV_8UC3);
  cv::vconcat(out0_img, out1_img, out_img);

  // Collect features ids in the previous frame
  std::vector<FeatureIDType> prev_ids(0);
  grid_features_extract_ids(this->prev_features, prev_ids);

  // Collect feature points in the previous frame
  std::map<FeatureIDType, cv::Point2f> prev_cam0_points;
  std::map<FeatureIDType, cv::Point2f> prev_cam1_points;
  grid_features_extract_points(this->prev_features,
                               prev_cam0_points,
                               prev_cam1_points);

  // Collect feature points in the current frame
  std::map<FeatureIDType, cv::Point2f> curr_cam0_points;
  std::map<FeatureIDType, cv::Point2f> curr_cam1_points;
  grid_features_extract_points(this->curr_features,
                               curr_cam0_points,
                               curr_cam1_points);

  // Draw tracked features
  out_img = draw_tracked_stereo_points(out_img,
                                       prev_ids,
                                       prev_cam0_points,
                                       prev_cam1_points,
                                       curr_cam0_points,
                                       curr_cam1_points);

  // Draw new features
  out_img = draw_new_stereo_points(out_img, curr_cam0_points, curr_cam1_points);

  // Display
  cv::imshow("Feature", out_img);
  cv::waitKey(1);
}

} //  namespace prototype
