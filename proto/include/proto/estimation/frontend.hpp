#ifndef PROTO_VISION_FRONTEND_HPP
#define PROTO_VISION_FRONTEND_HPP

#include "proto/core/core.hpp"

namespace proto {

struct feature_t {
  size_t id = 0;
  vec2s_t keypoints;
  vec3_t point{0.0, 0.0, 0.0};

  feature_t() {}

  feature_t(const size_t id_, const real_t x, const real_t y) {
    id = id_;
    keypoints.emplace_back(x, y);
  }

  void update(const real_t kx, const real_t ky) {
    keypoints.emplace_back(kx, ky);
  }

  size_t nb_frames_tracked() {
    return keypoints.size();
  }
};

struct feature_container_t {
  size_t next_id = 0;
  std::unordered_map<size_t, feature_t> tracking;
  std::vector<feature_t> lost;

  feature_container_t() {}

  void add(const real_t kx, const real_t ky) {
    tracking[next_id] = feature_t{next_id, kx, ky};
    next_id++;
  }

  void update(const size_t id, const real_t kx, const real_t ky) {
    tracking[id].update(kx, ky);
  }

  void mark_lost(const size_t id) {
    const auto &f = tracking[id];
    lost.push_back(f);
    tracking.erase(id);
  }

  void clear() {
    next_id = 0;
    tracking.clear();
    lost.clear();
  }

  void keypoints(std::vector<size_t> &ids, std::vector<cv::Point2f> &kps) {
    for (const auto &kv : tracking) {
      const auto &id = kv.first;
      const auto &kp = kv.second.keypoints.back();
      ids.push_back(id);
      kps.emplace_back(kp(0), kp(1));
    }
  }
};

struct frontend_t {
  // State
  bool initialized = false;
  size_t next_id = 0;
  feature_container_t features;
  cv::Mat image_prev;

  // Settings
  size_t min_tracking = 80;

  const int max_corners = 100;
  const int grid_rows = 4;
  const int grid_cols = 5;
  const real_t quality_level = 0.001;
  const real_t min_distance = 5;
  const cv::Mat mask = cv::Mat();
  const int block_size = 3;
  const bool use_harris_detector = true;
  const real_t k = 0.01;

  frontend_t() {}

  std::vector<cv::Point2f> detect(const cv::Mat &image) {
    const auto kps = grid_fast(image);
    if (kps.size() > 0) {
      initialized = true;
    }
    return kps;
  }

  void track(const cv::Mat &image,
             const std::vector<cv::Point2f> &kps0,
             std::vector<cv::Point2f> &kps1,
             std::vector<bool> &inliers,
             bool debug) {
    // Track using optical flow
    kps1 = kps0;
    std::vector<uchar> inliers_optflow;
    std::vector<float> err;
    cv::Size win_size(10, 10);
    int crit_type = (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS);
    int max_count = 30;
    double epsilon = 0.01;
    cv::TermCriteria criteria(crit_type, max_count, epsilon);
    cv::calcOpticalFlowPyrLK(image_prev, image,
                             kps0, kps1,
                             inliers_optflow, err,
                             win_size, 3, criteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // Remove outliers using RANSAC
    std::vector<uchar> inliers_ransac;
    int f_type = cv::FM_RANSAC;
    double param1 = 1.0;
    double param2 = 0.99;
    cv::findFundamentalMat(kps0, kps1, f_type, param1, param2, inliers_ransac);

    // Form inliers vector using results from optical flow and RANSAC
    for (size_t i = 0; i < kps0.size(); i++) {
      if (inliers_optflow[i] && inliers_ransac[i]) {
        inliers.push_back(true);
      } else {
        inliers.push_back(false);
      }
    }

    // Debug
    if (debug) {
      cv::Mat track_image;
      const int radius = 2;
      const cv::Scalar color(0, 255, 0);
      cv::vconcat(image_prev, image, track_image);

      for (size_t i = 0; i < kps0.size(); i++) {
        if (inliers[i]) {
          kps1[i].y += image.rows;
          cv::circle(track_image, kps0[i], radius, color, cv::FILLED);
          cv::circle(track_image, kps1[i], radius, color, cv::FILLED);
          kps1[i].y -= image.rows;
        }
      }

      cv::imshow("Tracking", track_image);
    }
  }

  void update(const cv::Mat &image, const bool debug=false) {
    assert(image.channels() == 1);

    // Detect or track
    std::vector<size_t> ids;
    std::vector<cv::Point2f> keypoints_prev;
    std::vector<cv::Point2f> keypoints_curr;
    std::vector<bool> inliers;
    features.keypoints(ids, keypoints_prev);

    if (initialized == false || keypoints_prev.size() < min_tracking) {
      // Detect
      keypoints_curr = detect(image);
      for (size_t i = 0; i < keypoints_curr.size(); i++) {
        const auto kp = keypoints_curr[i];
        features.add(kp.x, kp.y);
      }

    } else {
      // Track
      track(image, keypoints_prev, keypoints_curr, inliers, debug);
      for (size_t i = 0; i < keypoints_curr.size(); i++) {
        if (inliers[i]) {
          const auto kp = keypoints_curr[i];
          features.update(ids[i], kp.x, kp.y);
        } else {
          features.mark_lost(ids[i]);
        }
      }

    }
    image_prev = image;
  }

  void clear() {
    features.clear();
    // keypoints.clear();
    image_prev.release();
  }
};

} //  namespace proto
#endif // PROTO_VISION_FRONTEND_HPP
