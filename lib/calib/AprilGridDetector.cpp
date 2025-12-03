#include "AprilGridDetector.hpp"

namespace xyz {

AprilGridDetector::AprilGridDetector(const AprilGridConfig &target_config) {
  // Initialize Ed Olsen's AprilTag detector
  det_->quad_decimate = 1.0;
  det_->quad_sigma = 0.0; // Blur
  det_->nthreads = 2;
  det_->debug = 0;
  det_->refine_edges = 1;

  // Add target config
  target_configs_[target_config.target_id] = target_config;

  // Create tag_id -> target_id look-up-table
  for (const auto [target_id, target_config] : target_configs_) {
    const int tag_id_offset = target_config.tag_id_offset;
    const int num_tags = target_config.tag_rows * target_config.tag_cols;

    for (int tag_id = tag_id_offset; tag_id < num_tags; ++tag_id) {
      if (target_lut_.count(tag_id)) {
        FATAL("tag_id: %d already exists in target_id: %d!",
              tag_id,
              target_lut_[tag_id]);
      }

      target_lut_[tag_id] = target_id;
    }
  }
}

AprilGridDetector::AprilGridDetector(
    const std::map<int, AprilGridConfig> &target_configs)
    : target_configs_{target_configs} {
  // Initialize Ed Olsen's AprilTag detector
  det_->quad_decimate = 1.0;
  det_->quad_sigma = 0.0; // Blur
  det_->nthreads = 2;
  det_->debug = 0;
  det_->refine_edges = 1;

  // Create tag_id -> target_id look-up-table
  for (const auto [target_id, target_config] : target_configs_) {
    const int tag_id_offset = target_config.tag_id_offset;
    const int num_tags = target_config.tag_rows * target_config.tag_cols;

    for (int tag_id = tag_id_offset; tag_id < num_tags; ++tag_id) {
      if (target_lut_.count(tag_id)) {
        FATAL("tag_id: %d already exists in target_id: %d!",
              tag_id,
              target_lut_[tag_id]);
      }

      target_lut_[tag_id] = target_id;
    }
  }
}

AprilGridDetector::~AprilGridDetector() {
  apriltag_detector_destroy(det_);
  tag36h11_destroy(tf_);
}

void AprilGridDetector::olsenDetect(const cv::Mat &image,
                                    std::vector<int> &tag_ids,
                                    std::vector<int> &corner_indicies,
                                    std::vector<Vec2> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  // Make an image_u8_t header for the Mat data
  image_u8_t im = {.width = image.cols,
                   .height = image.rows,
                   .stride = image.cols,
                   .buf = image.data};

  // Detector tags
  zarray_t *detections = apriltag_detector_detect(det_, &im);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    // Check tag id
    // if (det->id < 0 || det->id >= (tag_rows_ * tag_cols_)) {
    //   continue;
    // }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= det->p[i][0] < min_border_dist_;
      bad_tag |= det->p[i][0] > image_width - min_border_dist_;
      bad_tag |= det->p[i][1] < min_border_dist_;
      bad_tag |= det->p[i][1] > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(det->id); // Bottom left
    tag_ids.push_back(det->id); // Bottom right
    tag_ids.push_back(det->id); // Top right
    tag_ids.push_back(det->id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    keypoints.emplace_back(det->p[0][0], det->p[0][1]); // Bottom left
    keypoints.emplace_back(det->p[1][0], det->p[1][1]); // Bottom right
    keypoints.emplace_back(det->p[2][0], det->p[2][1]); // Top right
    keypoints.emplace_back(det->p[3][0], det->p[3][1]); // Top left
  }
  apriltag_detections_destroy(detections);
}

void AprilGridDetector::kaessDetect(const cv::Mat &image,
                                    std::vector<int> &tag_ids,
                                    std::vector<int> &corner_indicies,
                                    std::vector<Vec2> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  const auto detections = detector.extractTags(image);
  for (const auto &tag : detections) {
    // Check detection
    if (tag.good == false) {
      continue;
    }

    // Check tag id
    // if (tag.id < 0 || tag.id >= (tag_rows_ * tag_cols_)) {
    //   continue;
    // }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= tag.p[i].first < min_border_dist_;
      bad_tag |= tag.p[i].first > image_width - min_border_dist_;
      bad_tag |= tag.p[i].second < min_border_dist_;
      bad_tag |= tag.p[i].second > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(tag.id); // Bottom left
    tag_ids.push_back(tag.id); // Bottom right
    tag_ids.push_back(tag.id); // Top right
    tag_ids.push_back(tag.id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    keypoints.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    keypoints.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
    keypoints.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    keypoints.emplace_back(tag.p[3].first, tag.p[3].second); // Top left
  }
}

std::vector<std::shared_ptr<AprilGrid>>
AprilGridDetector::detect(const timestamp_t ts,
                          const int camera_id,
                          const cv::Mat &image) {
  // Detect AprilTags
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  std::vector<Vec2> keypoints;
  kaessDetect(image, tag_ids, corner_indicies, keypoints);
  // olsenDetect(image, tag_ids, corner_indicies, keypoints);

  // Form AprilGrids
  std::map<int, std::shared_ptr<AprilGrid>> aprilgrids;
  for (const auto &[target_id, target_config] : target_configs_) {
    const auto config = target_configs_[target_id];
    aprilgrids[target_id] = std::make_shared<AprilGrid>(ts, camera_id, config);
  }

  // Add measurement
  for (size_t i = 0; i < tag_ids.size(); ++i) {
    const auto tag_id = tag_ids[i];
    if (target_lut_.count(tag_id) == 0) {
      continue;
    }

    const auto target_id = target_lut_[tag_id];
    const auto corner_index = corner_indicies[i];
    aprilgrids[target_id]->add(tag_id, corner_index, keypoints[i]);
  }

  // Form result
  std::vector<std::shared_ptr<AprilGrid>> result;
  for (const auto &[target_id, target] : aprilgrids) {
    result.push_back(target);
  }

  return result;
}

} // namespace xyz
