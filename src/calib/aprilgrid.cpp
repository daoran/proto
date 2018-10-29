#include "prototype/calib/aprilgrid.hpp"

namespace prototype {

aprilgrid_t::aprilgrid_t() {}

aprilgrid_t::aprilgrid_t(const long timestamp,
                         const int tag_rows,
                         const int tag_cols,
                         const double tag_size,
                         const double tag_spacing)
    : timestamp{timestamp}, tag_rows{tag_rows}, tag_cols{tag_cols},
      tag_size{tag_size}, tag_spacing{tag_spacing} {}

aprilgrid_t::~aprilgrid_t() {}

std::ostream &operator<<(std::ostream &os,
                         const aprilgrid_t &aprilgrid) {
  // Relative rotation and translation
  if (aprilgrid.estimated) {
    os << "AprilGrid: " << std::endl;
    os << "rvec_CF: " << aprilgrid.rvec_CF.transpose();
    os << "\t";
    os << "tvec_CF: " << aprilgrid.tvec_CF.transpose();
    os << std::endl;
  }

  for (size_t i = 0; i < aprilgrid.ids.size(); i++) {
    // Tag id
    os << "tag_id: " << aprilgrid.ids[i] << std::endl;

    // Keypoint and relative position
    for (size_t j = 0; j < 4; j++) {
      os << "keypoint: " << aprilgrid.keypoints[(i * 4) + j].transpose();
      if (aprilgrid.estimated) {
        os << "\t";
        os << "p_CF: " << aprilgrid.positions_CF[(i * 4) + j].transpose();
      }
      os << std::endl;
    }
    os << std::endl;
  }

  return os;
}

void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints) {
  assert(keypoints.size() == 4);

  // Set AprilGrid as detected
  grid.detected = true;

  // Push id and keypoints
  grid.ids.push_back(id);
  for (const auto keypoint : keypoints) {
    grid.keypoints.emplace_back(keypoint.x, keypoint.y);
  }
}

int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  std::vector<vec2_t> &keypoints) {
  // Check if tag id was actually detected
  int index = -1;
  for (size_t i = 0; i < grid.ids.size(); i++) {
    if (grid.ids[i] == id) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    LOG_ERROR("Failed to find tag id [%d] in AprilTagDetection!", id);
    return -1;
  }

  // Set keypoints
  keypoints.emplace_back(grid.keypoints[(index * 4)]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 1]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 2]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 3]);

  return 0;
}

int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  std::vector<vec2_t> &keypoints,
                  std::vector<vec3_t> &positions_CF) {
  assert(grid.estimated);

  // Check if tag id was actually detected
  int index = -1;
  for (size_t i = 0; i < grid.ids.size(); i++) {
    if (grid.ids[i] == id) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    LOG_ERROR("Failed to find tag id [%d] in AprilTagDetection!", id);
    return -1;
  }

  // Set keypoints
  keypoints.emplace_back(grid.keypoints[(index * 4)]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 1]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 2]);
  keypoints.emplace_back(grid.keypoints[(index * 4) + 3]);

  // Set positions
  positions_CF.emplace_back(grid.positions_CF[(index * 4)]);
  positions_CF.emplace_back(grid.positions_CF[(index * 4) + 1]);
  positions_CF.emplace_back(grid.positions_CF[(index * 4) + 2]);
  positions_CF.emplace_back(grid.positions_CF[(index * 4) + 3]);

  return 0;
}

int aprilgrid_grid_index(const aprilgrid_t &grid,
                         const int id,
                         int &i,
                         int &j) {
  if (id > (grid.tag_rows * grid.tag_cols) || id < 0) {
    return -1;
  }

  i = int(id / grid.tag_cols);
  j = int(id % grid.tag_cols);
  return 0;
}

int aprilgrid_calc_relative_pose(aprilgrid_t &grid,
                                 const mat3_t &cam_K,
                                 const vec4_t &cam_D) {
  // Check if we actually have data to work with
  if (grid.ids.size() == 0) {
    return 0;
  }

  // Create object points (counter-clockwise, from bottom left)
  std::vector<cv::Point3f> obj_pts;
  for (const auto id : grid.ids) {
    int i = 0;
    int j = 0;
    if (aprilgrid_grid_index(grid, id, i, j) == -1) {
      LOG_ERROR("Invalid id [%d]!", id);
      return -1;
    }

    // Caculate the x and y of the tag origin (bottom left corner of tag)
    // relative to grid origin (bottom left corner of entire grid)
    const double x = j * (grid.tag_size + grid.tag_size * grid.tag_spacing);
    const double y = i * (grid.tag_size + grid.tag_size * grid.tag_spacing);

    // Calculate the x and y of each corner
    const cv::Point3f bottom_left(x, y, 0);
    const cv::Point3f bottom_right(x + grid.tag_size, y, 0);
    const cv::Point3f top_right(x + grid.tag_size, y + grid.tag_size, 0);
    const cv::Point3f top_left(x, y + grid.tag_size, 0);

    // Add object points
    obj_pts.emplace_back(bottom_left);
    obj_pts.emplace_back(bottom_right);
    obj_pts.emplace_back(top_right);
    obj_pts.emplace_back(top_left);
  }

  // Create image points
  std::vector<cv::Point2f> img_pts;
  for (const auto kp: grid.keypoints) {
    img_pts.emplace_back(kp(0), kp(1));
  }

  // Extract out camera intrinsics
  const double fx = cam_K(0, 0);
  const double fy = cam_K(1, 1);
  const double cx = cam_K(0, 2);
  const double cy = cam_K(1, 2);

  // Extract out camera distortion
  const double k1 = cam_D(0);
  const double k2 = cam_D(1);
  const double p1 = cam_D(2);
  const double p2 = cam_D(3);

  // Solve pnp
  cv::Vec4f distortion_params(k1, k2, p1, p2); // SolvPnP assumes radtan
  cv::Mat camera_matrix(3, 3, CV_32FC1, 0.0f);
  camera_matrix.at<float>(0, 0) = fx;
  camera_matrix.at<float>(1, 1) = fy;
  camera_matrix.at<float>(0, 2) = cx;
  camera_matrix.at<float>(1, 2) = cy;
  camera_matrix.at<float>(2, 2) = 1.0;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(obj_pts,
               img_pts,
               camera_matrix,
               distortion_params,
               rvec,
               tvec,
               false,
               CV_ITERATIVE);

  // Form relative tag pose as a 4x4 transformation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  grid.T_CF = transform(convert(R), convert(tvec));
  grid.rvec_CF = convert(rvec);
  grid.tvec_CF = convert(tvec);

  // Calculate corner positions
  for (size_t idx = 0; idx < grid.ids.size(); idx++) {
    // Get tag grid index
    int i = 0;
    int j = 0;
    const auto id = grid.ids[idx];
    if (aprilgrid_grid_index(grid, id, i, j) == -1) {
      LOG_ERROR("Invalid id [%d]!", id);
      return -1;
    }

    // Calculate the x and y of the tag origin (bottom left corner of tag) relative
    // to grid origin (bottom left corner of entire grid)
    const double x = j * (grid.tag_size + grid.tag_size * grid.tag_spacing);
    const double y = i * (grid.tag_size + grid.tag_size * grid.tag_spacing);

    // Calculate the x and y of each corner
    const vec4_t bottom_left(x, y, 0, 1);
    const vec4_t bottom_right(x + grid.tag_size, y, 0, 1);
    const vec4_t top_right(x + grid.tag_size, y + grid.tag_size, 0, 1);
    const vec4_t top_left(x, y + grid.tag_size, 0, 1);

    // Transform object points to corner positions expressed in camera frame
    grid.positions_CF.emplace_back((grid.T_CF * bottom_left).head(3));
    grid.positions_CF.emplace_back((grid.T_CF * bottom_right).head(3));
    grid.positions_CF.emplace_back((grid.T_CF * top_right).head(3));
    grid.positions_CF.emplace_back((grid.T_CF * top_left).head(3));
  }
  grid.estimated = true;

  return 0;
}

int aprilgrid_save(const aprilgrid_t &grid,
                   const std::string &save_path) {
  assert((grid.keypoints.size() % 4) == 0);
  assert((grid.positions_CF.size() % 4) == 0);

  // Open file for saving
  std::ofstream outfile(save_path);
  if (outfile.good() != true) {
    LOG_ERROR("Failed to open [%s] for saving!", save_path.c_str());
    return -1;
  }

  // Output header
  outfile << "id,kp_x,kp_y";
  if (grid.estimated) {
    outfile << ",";
    outfile << "pos_x,pos_y,pos_z,";
    outfile << "rvec_x,rvec_y,rvec_z,";
    outfile << "tvec_x,tvec_y,tvec_z";
  }
  outfile << std::endl;

  // Output data
  for (size_t i = 0; i < grid.ids.size(); i++) {
    const int tag_id = grid.ids[i];

    for (int j = 0; j < 4; j++) {
      const vec2_t keypoint = grid.keypoints[(i * 4) + j];
      outfile << tag_id << ",";
      outfile << keypoint(0) << ",";
      outfile << keypoint(1);

      if (grid.estimated) {
        const vec3_t position = grid.positions_CF[(i * 4) + j];
        outfile << ",";
        outfile << position(0) << ",";
        outfile << position(1) << ",";
        outfile << position(2) << ",";
        outfile << grid.rvec_CF(0) << ",";
        outfile << grid.rvec_CF(1) << ",";
        outfile << grid.rvec_CF(2) << ",";

        outfile << grid.tvec_CF(0) << ",";
        outfile << grid.tvec_CF(1) << ",";
        outfile << grid.tvec_CF(2);
      }

      outfile << std::endl;
    }
  }

  // Close up
  outfile.close();

  return 0;
}

int aprilgrid_load(aprilgrid_t &grid,
                   const std::string &data_path) {
  // Load data file
  matx_t data;
  if (csv2mat(data_path, true, data) != 0) {
    LOG_ERROR("Failed to load AprilGrid detection data [%s]!",
              data_path.c_str());
    return -1;
  }
  assert(data.cols() == 3 || data.cols() == 12);

  // Parse file
  grid.ids.clear();
  grid.keypoints.clear();
  grid.positions_CF.clear();

  // Check if tag corner positions are estimated
  if (data.cols() == 12) {
    grid.estimated = true;
  }

  // Check if any tag is detected
  if (data.rows() > 0) {
    grid.detected = true;
  }

  // Parse data
  for (int i = 0; i < data.rows(); i++) {
    const vecx_t row = data.row(i);

    // Tag id
    if (std::count(grid.ids.begin(), grid.ids.end(), row(0)) == 0) {
      grid.ids.emplace_back(row(0));

      // Relative rotation and translation
      if (grid.estimated) {
        grid.rvec_CF = vec3_t{row(6), row(7), row(8)};
        grid.tvec_CF = vec3_t{row(9), row(10), row(11)};

        cv::Mat R;
        cv::Rodrigues(convert(grid.rvec_CF), R);
        grid.T_CF = transform(convert(R), grid.tvec_CF);
      }
    }

    // Keypoint
    grid.keypoints.emplace_back(row(1), row(2));

    // Position
    if (grid.estimated) {
      grid.positions_CF.emplace_back(row(3), row(4), row(5));
    }
  }

  return 0;
}

aprilgrid_detector_t::aprilgrid_detector_t() {
  detector.thisTagFamily.blackBorder = 2;
  ASSERT(detector.thisTagFamily.blackBorder == 2,
         "detector.thisTagFamily.blackBorder not editable!");
}

aprilgrid_detector_t::aprilgrid_detector_t(const int tag_rows,
                                           const int tag_cols,
                                           const double tag_size,
                                           const double tag_spacing)
    : ok{true}, tag_rows{tag_rows}, tag_cols{tag_cols}, tag_size{tag_size},
      tag_spacing{tag_spacing} {
  detector.thisTagFamily.blackBorder = 2;
  ASSERT(detector.thisTagFamily.blackBorder == 2,
         "detector.thisTagFamily.blackBorder not editable!");
}

aprilgrid_detector_t::~aprilgrid_detector_t() {}

int aprilgrid_detector_configure(aprilgrid_detector_t &det,
                                 const std::string &target_file) {
  // Load target file
  std::string target_type;
  config_t config{target_file};
  if (config.ok == false) {
    return -1;
  }
  parse(config, "target_type", target_type);
  parse(config, "tag_rows", det.tag_rows);
  parse(config, "tag_cols", det.tag_cols);
  parse(config, "tag_size", det.tag_size);
  parse(config, "tag_spacing", det.tag_spacing);

  // Double check tag type
  if (target_type != "aprilgrid") {
    FATAL("Invalid target_type [%s], expecting 'aprilgrid'!",
          target_type.c_str());
  }

  det.ok = true;
  return 0;
}

void aprilgrid_detector_filter_tags(
    const aprilgrid_detector_t &det,
    const cv::Mat &image,
    std::vector<AprilTags::TagDetection> &tags) {
  std::vector<AprilTags::TagDetection>::iterator iter = tags.begin();

  const double min_border_dist = 4.0;
  for (iter = tags.begin(); iter != tags.end();) {
    bool remove = false;

    for (int j = 0; j < 4; j++) {
      remove |= iter->p[j].first < min_border_dist;
      remove |= iter->p[j].first > (float) (image.cols) - min_border_dist;
      remove |= iter->p[j].second < min_border_dist;
      remove |= iter->p[j].second > (float) (image.rows) - min_border_dist;
    }

    // Remove tags that are flagged as bad
    if (iter->good != 1) {
      remove |= true;
    }

    // Delete flagged tags
    if (remove) {
      iter = tags.erase(iter);
    } else {
      ++iter;
    }
  }
}

aprilgrid_t aprilgrid_detector_detect(aprilgrid_detector_t &det,
                                      const long timestamp,
                                      const cv::Mat &image) {
  // Convert image to gray-scale
  const cv::Mat image_gray = rgb2gray(image);

  // Extract tags
  std::vector<AprilTags::TagDetection> tags =
    det.detector.extractTags(image_gray);
  aprilgrid_detector_filter_tags(det, image, tags);
  std::sort(tags.begin(), tags.end(), sort_apriltag_by_id);

  // Form results
  aprilgrid_t grid(timestamp,
                   det.tag_rows,
                   det.tag_cols,
                   det.tag_size,
                   det.tag_spacing);
  for (const auto tag : tags) {
    std::vector<cv::Point2f> img_pts;
    img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Top left
    img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Bottom right
    aprilgrid_add(grid, tag.id, img_pts);
  }

  return grid;
}

aprilgrid_t aprilgrid_detector_detect(aprilgrid_detector_t &det,
                                      const long timestamp,
                                      const cv::Mat &image,
                                      const mat3_t &cam_K,
                                      const vec4_t &cam_D) {
  auto grid = aprilgrid_detector_detect(det, timestamp, image);
  aprilgrid_calc_relative_pose(grid, cam_K, cam_D);
  return grid;
}

void aprilgrid_detector_imshow(
    const std::string &title,
    const cv::Mat &image,
    const std::vector<AprilTags::TagDetection> &tags) {
  // Draw corners
  cv::Mat image_rgb = gray2rgb(image);

  for (size_t i = 0; i < tags.size(); i++) {
    // Form corner points
    cv::Point2f p1(tags[i].p[0].first, tags[i].p[0].second); // Bottom left
    cv::Point2f p2(tags[i].p[1].first, tags[i].p[1].second); // Top left
    cv::Point2f p3(tags[i].p[2].first, tags[i].p[2].second); // Top right
    cv::Point2f p4(tags[i].p[3].first, tags[i].p[3].second); // Bottom right

    // Draw corners
    cv::circle(image_rgb, p1, 2, cv::Scalar(0, 0, 255), -1); // Bottom left
    cv::circle(image_rgb, p2, 2, cv::Scalar(0, 0, 255), -1); // Top left
    cv::circle(image_rgb, p3, 2, cv::Scalar(0, 0, 255), -1); // Top right
    cv::circle(image_rgb, p4, 2, cv::Scalar(0, 0, 255), -1); // Bottom right

    // Draw Tag ID
    cv::Point2f cxy(tags[i].cxy.first - 10, tags[i].cxy.second + 5);
    cv::Scalar text_color(0, 0, 255);
    std::string text = std::to_string(tags[i].id);
    int font = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1.0;
    int thickness = 2;
    cv::putText(image_rgb, text, cxy, font, font_scale, text_color, thickness);
  }

  // Show
  cv::imshow(title, image_rgb);
  cv::waitKey(1);
}

} // namespace prototype
