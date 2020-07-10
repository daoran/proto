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

  size_t tracked() {
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
  const size_t min_tracking = 30;
  const size_t max_tracking = 50;

  // -- Grid FAST Settings
  const int grid_rows = 4;
  const int grid_cols = 5;
  const real_t quality_level = 0.001;
  const real_t min_distance = 5;
  const cv::Mat mask = cv::Mat();
  const int block_size = 3;
  const bool use_harris_detector = true;
  const real_t k = 0.01;

  // -- Optical Flow Settings
  const int patch_size = 10;
  const double max_level = 3;
  const int max_iter = 30;
  const double epsilon = 0.01;

  // -- RANSAC settings
  const int f_type = cv::FM_RANSAC;
  const double param1 = 1.0;
  const double param2 = 0.99;

  frontend_t() {}

  void visualize(const cv::Mat &image) {
    cv::Mat track_image = gray2rgb(image);
    const int radius = 2;
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar yellow(0, 255, 255);

    for (auto &kv : features.tracking) {
      auto &f = kv.second;
      const auto &kp = f.keypoints.back();
      const cv::Point2f p(kp(0), kp(1));

      if (f.tracked() > 3) {
        cv::circle(track_image, p, radius, green, cv::FILLED);
      } else {
        cv::circle(track_image, p, radius, yellow, cv::FILLED);
      }
    }

    cv::imshow("Tracking", track_image);
  }

  void detect(const cv::Mat &image) {
    const size_t nb_keypoints = max_tracking - features.tracking.size();
    const auto keypoints = grid_fast(image, nb_keypoints);
    if (initialized == false && keypoints.size() > 0) {
      initialized = true;
    }

    for (size_t i = 0; i < keypoints.size(); i++) {
      const auto kp = keypoints[i];
      features.add(kp.x, kp.y);
    }
  }

  void track(const cv::Mat &image, bool debug) {
    // Setup
    std::vector<size_t> ids;
    std::vector<cv::Point2f> kps0;
    std::vector<cv::Point2f> kps1;
    features.keypoints(ids, kps0);
    kps1 = kps0;

    // Track using optical flow
    std::vector<uchar> inliers_optflow;
    std::vector<float> err;
    cv::Size win_size(patch_size, patch_size);
    int crit_type = (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS);
    cv::TermCriteria criteria(crit_type, max_iter, epsilon);
    cv::calcOpticalFlowPyrLK(image_prev, image,
                             kps0, kps1,
                             inliers_optflow, err,
                             win_size, max_level, criteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // Remove outliers using RANSAC
    std::vector<uchar> inliers_ransac;
    cv::findFundamentalMat(kps0, kps1, f_type, param1, param2, inliers_ransac);

    // Update or mark feature as lost
    std::vector<bool> inliers;
    size_t lost = 0;
    for (size_t i = 0; i < kps0.size(); i++) {
      if (inliers_optflow[i] && inliers_ransac[i]) {
        inliers.push_back(true);
        const auto kp = kps1[i];
        features.update(ids[i], kp.x, kp.y);
      } else {
        inliers.push_back(false);
        features.mark_lost(ids[i]);
        lost++;
      }
    }

    // Debug
    if (debug) {
      visualize(image);
    }
  }

  void update(const cv::Mat &image, const bool debug=false) {
    assert(image.channels() == 1);

    if (initialized == false) {
      detect(image);
    } else {
      if (features.tracking.size() < min_tracking) {
        detect(image);
      }
      track(image, debug);
    }

    image_prev = image;
  }

  void clear() {
    features.clear();
    image_prev.release();
  }
};

} //  namespace proto
#endif // PROTO_VISION_FRONTEND_HPP
