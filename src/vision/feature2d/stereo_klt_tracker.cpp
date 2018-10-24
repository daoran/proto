#include "prototype/vision/feature2d/stereo_klt_tracker.hpp"

namespace prototype {

StereoKLTTracker::StereoKLTTracker() {}

StereoKLTTracker::StereoKLTTracker(const CameraProperty &cam0,
                                   const CameraProperty &cam1,
                                   const Mat4 &T_cam1_cam0,
                                   const size_t min_track_length,
                                   const size_t max_track_length)
    : type{STATIC_STEREO_TRACK}, cam0{cam0}, cam1{cam1},
      T_cam1_cam0{T_cam1_cam0}, features{min_track_length, max_track_length} {
  assert(T_cam1_cam0.isApprox(Mat4::Zero()) == false);
}

StereoKLTTracker::StereoKLTTracker(const CameraProperty &cam0,
                                   const CameraProperty &cam1,
                                   const GimbalModel &gimbal_model,
                                   const size_t min_track_length,
                                   const size_t max_track_length)
    : type{DYNAMIC_STEREO_TRACK}, cam0{cam0}, cam1{cam1},
      gimbal_model{gimbal_model}, features{min_track_length, max_track_length} {
}

StereoKLTTracker::~StereoKLTTracker() {}

std::vector<cv::Point2f> StereoKLTTracker::detect(const cv::Mat &image,
                                                  const cv::Mat &mask) {
  // Keep track of image width and height
  if (this->image_width == 0) {
    this->image_width = image.cols;
  }
  if (this->image_height == 0) {
    this->image_height = image.rows;
  }

  // Ensure image is grayscale
  const cv::Mat image_gray = rgb2gray(image);

  // Feature detection
  const std::vector<cv::Point2f> corners =
      grid_good(image,
                this->max_corners,
                this->grid_rows,
                this->grid_cols,
                this->quality_level,
                this->min_distance,
                mask);

  // auto keypoints =
  //     grid_fast(image_gray, this->max_corners, 5, 5, this->threshold);
  // std::vector<cv::Point2f> corners;
  // for (auto kp : keypoints) {
  //   corners.emplace_back(kp.pt);
  // }

  return corners;
}

void StereoKLTTracker::match(const cv::Mat &img0,
                             const cv::Mat &img1,
                             std::vector<cv::Point2f> &pts0,
                             std::vector<cv::Point2f> &pts1,
                             std::vector<uchar> &mask) {
  // Perform optical flow matching between cam0 and cam1 images
  std::vector<uchar> flow_mask;
  std::vector<float> err;
  cv::Size win_size(21, 21);
  const int pyr_levels = this->pyramid_levels;
  auto term_crit =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                       this->max_iteration,
                       this->track_precision);
  auto flags = cv::OPTFLOW_UPROTOTYPE_VISION_INITIAL_FLOW;
  cv::calcOpticalFlowPyrLK(img0,       // First image
                           img1,       // Second image
                           pts0,       // Input points
                           pts1,       // Output points
                           flow_mask,  // Tracking status
                           err,        // Tracking error
                           win_size,   // Window size
                           pyr_levels, // Pyramid levels
                           term_crit,  // Termination criteria
                           flags);     // Flags

  // RANSAC
  std::vector<uchar> ransac_mask;
  cv::findFundamentalMat(pts0, pts1, cv::FM_RANSAC, 1, 0.9999, ransac_mask);

  // Create final mask
  for (size_t i = 0; i < pts0.size(); i++) {
    const uchar status = (flow_mask[i] && ransac_mask[i]) ? 1 : 0;
    mask.push_back(status);
  }
}

int StereoKLTTracker::initialize(const cv::Mat &cam0_img,
                                 const cv::Mat &cam1_img) {
  // Detect features in cam0
  std::vector<cv::Point2f> pts0 = this->detect(cam0_img);
  if (pts0.size() == 0) {
    LOG_ERROR("Failed to detect any features !");
    return -1;
  }
  this->stats.update(pts0.size(), 0);

  // Project keypoints from cam0 to cam1 to form k1
  const Mat3 R_cam1_cam0 = this->T_cam1_cam0.block(0, 0, 3, 3);
  const auto pts0_ud = this->cam0.undistortPoints(pts0, R_cam1_cam0);
  auto pts1 = this->cam1.distortPoints(pts0_ud);

  // Perform stereo match
  std::vector<uchar> inliers;
  this->match(cam0_img, cam1_img, pts0, pts1, inliers);
  for (size_t i = 0; i < pts0.size(); i++) {
    if (inliers[i] == 1) {
      this->cam0_pts.emplace_back(pts0[i]);
      this->cam1_pts.emplace_back(pts1[i]);
    }
  }

  // Initialize track_ids
  this->track_ids = std::vector<int>(this->cam0_pts.size(), -1);

  // Make sure we're tracking someting
  if (this->cam0_pts.size() == 0) {
    LOG_ERROR("Failed stereo matching!");
    return -1;
  }

  // Initialize counters and previous cam0 and cam1 images
  this->counter_frame_id++;
  this->prev_cam0_img = cam0_img.clone();
  this->prev_cam1_img = cam1_img.clone();

  return 0;
}

int StereoKLTTracker::initialize(const cv::Mat &cam0_img,
                                 const cv::Mat &cam1_img,
                                 const double gimbal_roll,
                                 const double gimbal_pitch) {
  assert(this->type == DYNAMIC_STEREO_TRACK);
  this->gimbal_model.setAttitude(gimbal_roll, gimbal_pitch);
  this->T_cam1_cam0 = this->gimbal_model.T_ds();
  return this->initialize(cam0_img, cam1_img);
}

int StereoKLTTracker::updateTrack(const int index,
                                  const bool is_inlier,
                                  const cv::Point2f &pt0_ref,
                                  const cv::Point2f &pt0_cur,
                                  const cv::Point2f &pt1_ref,
                                  const cv::Point2f &pt1_cur) {
  auto cam0_f0 = Feature(pt0_ref);
  auto cam0_f1 = Feature(pt0_cur);
  auto cam1_f0 = Feature(pt1_ref);
  auto cam1_f1 = Feature(pt1_cur);
  int track_id = this->track_ids[index];

  // Remove feature track
  if (is_inlier == false) {
    this->features.removeTrack(track_id);
    return -1;
  }

  if (track_id == -1) {
    // Add new feature track
    if (this->type == STATIC_STEREO_TRACK) {
      this->features.addStereoTrack(this->counter_frame_id,
                                    cam0_f0,
                                    cam0_f1,
                                    cam1_f0,
                                    cam1_f1,
                                    this->T_cam1_cam0);
    } else if (this->type == DYNAMIC_STEREO_TRACK) {
      this->features.addStereoTrack(this->counter_frame_id,
                                    cam0_f0,
                                    cam0_f1,
                                    cam1_f0,
                                    cam1_f1,
                                    this->joint_angles_prev,
                                    this->joint_angles_curr);
    }
    track_id = cam0_f0.track_id;

  } else {
    // Update feature track
    int too_old = 0;
    if (this->type == STATIC_STEREO_TRACK) {
      too_old = this->features.updateStereoTrack(this->counter_frame_id,
                                                 track_id,
                                                 cam0_f1,
                                                 cam1_f1);
    } else if (this->type == DYNAMIC_STEREO_TRACK) {
      too_old = this->features.updateStereoTrack(this->counter_frame_id,
                                                 track_id,
                                                 cam0_f1,
                                                 cam1_f1,
                                                 this->joint_angles_curr);
    }

    if (too_old == 1) {
      return -1;
    }
  }

  return track_id;
}

void StereoKLTTracker::trackFeatures(const cv::Mat &cam0_img,
                                     const cv::Mat &cam1_img) {
  // Make a copy of feature points currently tracking
  std::vector<cv::Point2f> pts0_ref = this->cam0_pts;
  std::vector<cv::Point2f> pts1_ref = this->cam1_pts;
  assert(pts0_ref.size() == pts1_ref.size());
  assert(pts0_ref.size() > 0);

  // Temporally match cam0 images
  std::vector<cv::Point2f> pts0_cur = pts0_ref;
  std::vector<uchar> t0_mask;
  this->match(this->prev_cam0_img, cam0_img, pts0_ref, pts0_cur, t0_mask);

  // Temporally match cam1 images
  std::vector<cv::Point2f> pts1_cur = pts1_ref;
  std::vector<uchar> t1_mask;
  this->match(this->prev_cam1_img, cam1_img, pts1_ref, pts1_cur, t1_mask);

  // Stereo match cam0 and cam1 points
  std::vector<uchar> s_mask;
  this->match(cam0_img, cam1_img, pts0_cur, pts1_cur, s_mask);
  essential_matrix_outlier_rejection(this->cam0,
                                     this->cam1,
                                     this->T_cam1_cam0,
                                     pts0_cur,
                                     pts1_cur,
                                     10,
                                     s_mask);

  // Remove outliers
  std::vector<cv::Point2f> pts0_inliers;
  std::vector<cv::Point2f> pts1_inliers;
  std::vector<int> tracks_tracking;
  int nb_outliers = 0;

  for (size_t i = 0; i < pts0_ref.size(); i++) {
    bool is_inlier = (t0_mask[i] && t1_mask[i] && s_mask[i]) ? true : false;
    int track_id = this->updateTrack(i,
                                     is_inlier,
                                     pts0_ref[i],
                                     pts0_cur[i],
                                     pts1_ref[i],
                                     pts1_cur[i]);
    if (is_inlier && track_id != -1) {
      pts0_inliers.push_back(pts0_cur[i]);
      pts1_inliers.push_back(pts1_cur[i]);
      tracks_tracking.push_back(track_id);
    } else {
      nb_outliers++;
    }
  }

  // Update
  this->track_ids = tracks_tracking;
  this->cam0_pts = pts0_inliers;
  this->cam1_pts = pts1_inliers;
  this->prev_cam0_img = cam0_img.clone();
  this->prev_cam1_img = cam1_img.clone();
  this->stats.update(this->cam0_pts.size(), nb_outliers);
}

void StereoKLTTracker::replenishFeatures(const cv::Mat &image) {
  assert(this->T_cam1_cam0.isApprox(Mat4::Zero()) == false);

  // Clear previous new points
  this->pts0_new.clear();
  this->pts1_new.clear();

  // Pre-check
  const int replenish_size = this->max_corners - this->cam0_pts.size();
  if (replenish_size <= 0) {
    return;
  }

  // Build a grid denoting where existing keypoints already are
  assert(this->image_width > 0);
  assert(this->image_height > 0);

  // Detect new features
  const cv::Mat mask = feature_mask_opencv(this->image_width,
                                           this->image_height,
                                           this->cam0_pts,
                                           10);
  this->pts0_new = this->detect(image, mask);

  // Project new keypoints from cam0 to cam1
  const Mat3 R_cam1_cam0 = this->T_cam1_cam0.block(0, 0, 3, 3);
  const auto pts1_ud = this->cam0.undistortPoints(this->pts0_new, R_cam1_cam0);
  this->pts1_new = this->cam1.distortPoints(pts1_ud);

  // Append new keypoints to cam0_pts and cam1_pts
  std::vector<int> ids_new(this->pts0_new.size(), -1);
  extend(this->cam0_pts, this->pts0_new);
  extend(this->cam1_pts, this->pts1_new);
  extend(this->track_ids, ids_new);
}

int StereoKLTTracker::update(const cv::Mat &cam0_img, const cv::Mat &cam1_img) {
  assert(this->type == STATIC_STEREO_TRACK);

  // Initialize feature tracker
  if (this->cam0_pts.size() == 0) {
    this->cam0_pts.clear();
    this->cam1_pts.clear();
    this->initialize(cam0_img, cam1_img);
    return 0;
  }

  // Track features
  this->counter_frame_id++;
  this->trackFeatures(cam0_img, cam1_img);

  // Replenish features
  this->replenishFeatures(cam0_img);

  // Visualize
  this->showMatches(cam0_img, cam1_img);
  this->showTracking(cam0_img);

  return 0;
}

int StereoKLTTracker::update(const cv::Mat &cam0_img,
                             const cv::Mat &cam1_img,
                             const double gimbal_roll,
                             const double gimbal_pitch) {
  assert(this->type == DYNAMIC_STEREO_TRACK);

  // Update camera extrinsics with gimbal model
  this->gimbal_model.setAttitude(gimbal_roll, gimbal_pitch);
  this->T_cam1_cam0 = this->gimbal_model.T_ds();
  this->joint_angles_curr = this->gimbal_model.getJointAngles();

  // Initialize feature tracker
  if (this->cam0_pts.size() == 0) {
    this->cam0_pts.clear();
    this->cam1_pts.clear();
    this->initialize(cam0_img, cam1_img);
    return 0;
  }

  // Track features
  this->counter_frame_id++;
  this->trackFeatures(cam0_img, cam1_img);

  // Replenish features
  this->replenishFeatures(cam0_img);

  // Keep track of joint angles
  this->joint_angles_prev = this->joint_angles_curr;

  // Visualize
  this->showMatches(cam0_img, cam1_img);
  this->showTracking(cam0_img);

  return 0;
}

std::vector<FeatureTrack> StereoKLTTracker::getLostTracks() {
  // Get lost tracks
  std::vector<FeatureTrack> tracks;
  this->features.removeLostTracks(tracks);

  // Transform keypoints
  for (auto &track : tracks) {
    // Undistort points and convert pixel coordinates to image coordinates
    for (size_t i = 0; i < track.trackedLength(); i++) {
      auto &f0 = track.track0[i];
      auto &f1 = track.track1[i];
      f0.kp.pt = this->cam0.undistortPoint(f0.kp.pt);
      f1.kp.pt = this->cam1.undistortPoint(f1.kp.pt);
    }
  }

  return tracks;
}

void StereoKLTTracker::showMatches(const cv::Mat &cam0_img,
                                   const cv::Mat &cam1_img) {
  // Pre-check
  if (this->show_matches == false) {
    return;
  }

  // Stack current and previous image vertically
  cv::Mat match_img;
  cv::Mat cam0_rgb_img = gray2rgb(cam0_img);
  cv::Mat cam1_rgb_img = gray2rgb(cam1_img);
  cv::cvtColor(cam0_img, cam0_rgb_img, cv::COLOR_GRAY2BGR);
  cv::cvtColor(cam1_img, cam1_rgb_img, cv::COLOR_GRAY2BGR);
  cv::vconcat(cam0_rgb_img, cam1_rgb_img, match_img);

  // Draw matches
  for (size_t i = 0; i < this->cam0_pts.size(); i++) {
    cv::Point2f p0 = this->cam0_pts[i];
    cv::Point2f p1 = this->cam1_pts[i];

    // If point is new -- skip
    if (std::find(this->pts0_new.begin(), this->pts0_new.end(), p0) !=
        this->pts0_new.end()) {
      continue;
    }
    if (std::find(this->pts1_new.begin(), this->pts1_new.end(), p1) !=
        this->pts1_new.end()) {
      continue;
    }

    // Point 1
    p1.y += cam0_img.rows;

    // Draw circle and line
    cv::circle(match_img, p0, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0, p1, cv::Scalar(0, 255, 0));
  }

  for (auto &p : this->pts0_new) {
    cv::circle(match_img, p, 2, cv::Scalar(0, 255, 255), -1);
  }

  cv::Point2f offset(0, this->image_height);
  for (auto &p : this->pts1_new) {
    cv::circle(match_img, p + offset, 2, cv::Scalar(0, 255, 255), -1);
  }

  cv::imshow("Match", match_img);
  cv::waitKey(1);
}

void StereoKLTTracker::showTracking(const cv::Mat &cam0_img) {
  // Pre-check
  if (this->show_tracking == false) {
    return;
  }

  // Draw matches
  cv::Mat cam0_rgb_img = gray2rgb(cam0_img);
  for (const auto &p : this->cam0_pts) {
    // If point is new -- skip
    if (std::find(this->pts0_new.begin(), this->pts0_new.end(), p) !=
        this->pts0_new.end()) {
      continue;
    }

    // Draw circle
    cv::circle(cam0_rgb_img, p, 2, cv::Scalar(0, 255, 0), -1);
  }

  for (auto &p : this->pts0_new) {
    cv::circle(cam0_rgb_img, p, 2, cv::Scalar(0, 255, 255), -1);
  }

  cv::imshow("Tracking", cam0_rgb_img);
  cv::waitKey(1);
}

} //  namespace prototype
