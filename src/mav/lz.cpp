#include "prototype/mav/lz.hpp"

namespace proto {

lz_detector_t::lz_detector_t() {}

lz_detector_t::lz_detector_t(const std::vector<int> &tag_ids,
                             const std::vector<double> &tag_sizes) {
  if (lz_detector_configure(*this, tag_ids, tag_sizes) != 0) {
    FATAL("Failed to configure landing zone!");
  }
}

lz_detector_t::lz_detector_t(const std::string &config_file,
                             const std::string &prefix) {
  if (lz_detector_configure(*this, config_file, prefix) != 0) {
    FATAL("Failed to configure landing zone!");
  }
}

lz_detector_t::~lz_detector_t() {
  if (det != nullptr) {
    delete det;
  }
}

void lz_print(const lz_t &lz) {
  printf("detected: %s\n", (lz.detected) ? "true" : "false");
  print_matrix("T_CZ", lz.T_CZ);
  printf("\n");
}

int lz_detector_configure(lz_detector_t &lz,
                          const std::vector<int> &tag_ids,
                          const std::vector<double> &tag_sizes) {
  if (tag_ids.size() != tag_sizes.size()) {
    LOG_ERROR("tag_ids.size() != tag_sizes.size()");
    return -1;
  }

  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const double tag_sz = tag_sizes[i];
    lz.targets[tag_id] = tag_sz;
  }
  lz.det = new AprilTags::TagDetector(AprilTags::tagCodes16h5);
  lz.ok = true;

  return 0;
}

int lz_detector_configure(lz_detector_t &lz,
                          const std::string &config_file,
                          const std::string &prefix) {
  std::vector<int> tag_ids;
  std::vector<double> tag_sizes;

  const std::string p = (prefix == "") ? "" : prefix + ".";
  config_t config{config_file};
  parse(config, p + "tag_ids", tag_ids);
  parse(config, p + "tag_sizes", tag_sizes);

  return lz_detector_configure(lz, tag_ids, tag_sizes);
}

int lz_detector_detect(const lz_detector_t &lz,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       proto::mat4_t &T_CZ) {
  // Convert colour image to gray
  cv::Mat gray_image;
  cvtColor(image, gray_image, CV_BGR2GRAY);

  // Extract camera intrinsics
  const double fx = pinhole.fx;
  const double fy = pinhole.fy;
  const double cx = pinhole.cx;
  const double cy = pinhole.cy;

  // Calculate relative pose
  bool detected = false;
  const auto detections = lz.det->extractTags(gray_image);
  for (const AprilTags::TagDetection &tag : detections) {
    if (lz.targets.count(tag.id) == 0) {
      continue;
    }

    const double tag_size = lz.targets.at(tag.id);
    T_CZ = tag.getRelativeTransform(tag_size, fx, fy, cx, cy);
    detected = true;
    break;
  }

  return (detected) ? 1 : 0;
}

int lz_detector_detect(const lz_detector_t &det,
                       const cv::Mat &image,
                       const pinhole_t &pinhole,
                       const mat4_t &T_BC,
                       lz_t &lz) {
  mat4_t T_CZ = I(4);
  if (lz_detector_detect(det, image, pinhole, T_CZ) == 0) {
    lz = lz_t{false, T_BC, T_CZ};
    return 0;
  }

  lz = lz_t{true, T_BC, T_CZ};
  return 1;
}

int lz_calc_corners(const lz_detector_t &lz,
                    const pinhole_t &pinhole,
                    const cv::Mat &image,
                    const mat4_t &T_CZ,
                    const int tag_id,
                    const double padding,
                    vec2_t &top_left,
                    vec2_t &btm_right) {
  // Tag size and camera intrinsics
  const double tag_size = lz.targets.at(tag_id);
  const double fx = pinhole.fx;
  const double fy = pinhole.fy;
  const double cx = pinhole.cx;
  const double cy = pinhole.cy;

  // Tag position in camera frame
  const vec3_t r_CZ = tf_trans(T_CZ);
  const double x = r_CZ(0);
  const double y = r_CZ(1);
  const double z = r_CZ(2);

  // Calculate top left and bottom right corners of tag in object frame
  top_left(0) = x - (tag_size / 2.0) - padding;
  top_left(1) = y - (tag_size / 2.0) - padding;
  btm_right(0) = x + (tag_size / 2.0) + padding;
  btm_right(1) = y + (tag_size / 2.0) + padding;

  // Project back to image frame (what it would look like in image)
  top_left(0) = (fx * top_left(0) / z) + cx;
  top_left(1) = (fy * top_left(1) / z) + cy;
  btm_right(0) = (fx * btm_right(0) / z) + cx;
  btm_right(1) = (fy * btm_right(1) / z) + cy;

  // Check corner bounds
  top_left(0) = (top_left(0) > image.cols) ? image.cols : top_left(0);
  top_left(1) = (top_left(1) > image.rows) ? image.rows : top_left(1);
  top_left(0) = (top_left(0) < 0) ? 0 : top_left(0);
  top_left(1) = (top_left(1) < 0) ? 0 : top_left(1);
  btm_right(0) = (btm_right(0) > image.cols) ? image.cols : btm_right(0);
  btm_right(1) = (btm_right(1) > image.rows) ? image.rows : btm_right(1);

  return 0;
}

} //  namespace proto
