#include "prototype/calib/aprilgrid.hpp"

namespace prototype {

aprilgrid_detector_t::aprilgrid_detector_t() {}

aprilgrid_detector_t::~aprilgrid_detector_t() {}

aprilgrid_t::aprilgrid_t() {}

aprilgrid_t::aprilgrid_t(const long timestamp,
                         const int tag_rows,
                         const int tag_cols,
                         const double tag_size,
                         const double tag_spacing)
    : configured{true}, timestamp{timestamp}, tag_rows{tag_rows}, tag_cols{tag_cols},
      tag_size{tag_size}, tag_spacing{tag_spacing} {}

aprilgrid_t::~aprilgrid_t() {}

void aprilgrid_add(aprilgrid_t &grid,
                   const int id,
                   const std::vector<cv::Point2f> &keypoints) {
  assert(keypoints.size() == 4);

  // Set AprilGrid as detected
  grid.detected = true;
  grid.nb_detections++;

  // Push id and keypoints
  grid.ids.push_back(id);
  for (const auto keypoint : keypoints) {
    grid.keypoints.emplace_back(keypoint.x, keypoint.y);
  }
}

void aprilgrid_remove(aprilgrid_t &grid, const int id) {
  // Get index where id is stored
  size_t index = 0;
  bool found = false;
  for (index = 0; index < grid.ids.size(); index++) {
    if (grid.ids.at(index) == id) {
      found = true;
      break;
    }
  }
  if (found == false) {
    return;
  }

  // Remove id
  auto &ids = grid.ids;
  ids.erase(ids.begin() + index);
  ids.shrink_to_fit();

  // Remove keypoints
  auto &kps = grid.keypoints;
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.erase(kps.begin() + (index * 4));
  kps.shrink_to_fit();

  // Remove points
  auto &points_CF = grid.points_CF;
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.erase(points_CF.begin() + (index * 4));
  points_CF.shrink_to_fit();
}

int aprilgrid_get(const aprilgrid_t &grid,
                  const int id,
                  vec2s_t &keypoints) {
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
                  vec2s_t &keypoints,
                  vec3s_t &points_CF) {
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

  // Set points
  points_CF.emplace_back(grid.points_CF[(index * 4)]);
  points_CF.emplace_back(grid.points_CF[(index * 4) + 1]);
  points_CF.emplace_back(grid.points_CF[(index * 4) + 2]);
  points_CF.emplace_back(grid.points_CF[(index * 4) + 3]);

  return 0;
}

void aprilgrid_set_properties(aprilgrid_t &grid,
                              const int tag_rows,
                              const int tag_cols,
                              const double tag_size,
                              const double tag_spacing) {
  grid.configured = true;
  grid.tag_rows = tag_rows;
  grid.tag_cols = tag_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;
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

int aprilgrid_object_point(const aprilgrid_t &grid,
                           const int tag_id,
                           const int corner_id,
                           vec3_t &object_point) {
  const int tag_rows = grid.tag_rows;
  const int tag_cols = grid.tag_cols;
  const double tag_size = grid.tag_size;
  const double tag_spacing = grid.tag_spacing;

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  if (aprilgrid_grid_index(grid, tag_id, i, j) != 0) {
    LOG_ERROR("Incorrect tag id [%d]!", tag_id);
    return -1;
  }

  // Caculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const double x = j * (tag_size + tag_size * tag_spacing);
  const double y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner
  switch (corner_id) {
    case 0: // Bottom left
      object_point = vec3_t(x, y, 0);
      break;
    case 1: // Bottom right
      object_point = vec3_t(x + tag_size, y, 0);
      break;
    case 2: // Top right
      object_point = vec3_t(x + tag_size, y + tag_size, 0);
      break;
    case 3: // Top left
      object_point = vec3_t(x, y + tag_size, 0);
      break;
    default: FATAL("Incorrect corner id [%d]!", corner_id); break;
  }

  return 0;
}

int aprilgrid_object_points(const aprilgrid_t &grid,
                            const int tag_id,
                            vec3s_t &object_points) {
  const int tag_rows = grid.tag_rows;
  const int tag_cols = grid.tag_cols;
  const double tag_size = grid.tag_size;
  const double tag_spacing = grid.tag_spacing;

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  if (aprilgrid_grid_index(grid, tag_id, i, j) != 0) {
    LOG_ERROR("Incorrect tag id [%d]!", tag_id);
    return -1;
  }

  // Calculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const double x = j * (tag_size + tag_size * tag_spacing);
  const double y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner
  object_points.emplace_back(x, y, 0);
  object_points.emplace_back(x + tag_size, y, 0);
  object_points.emplace_back(x + tag_size, y + tag_size, 0);
  object_points.emplace_back(x, y + tag_size, 0);

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
  for (const auto kp : grid.keypoints) {
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

  // Calculate corner points
  for (size_t idx = 0; idx < grid.ids.size(); idx++) {
    // Get tag grid index
    int i = 0;
    int j = 0;
    const auto id = grid.ids[idx];
    if (aprilgrid_grid_index(grid, id, i, j) == -1) {
      LOG_ERROR("Invalid id [%d]!", id);
      return -1;
    }

    // Calculate the x and y of the tag origin (bottom left corner of tag)
    // relative to grid origin (bottom left corner of entire grid)
    const double x = j * (grid.tag_size + grid.tag_size * grid.tag_spacing);
    const double y = i * (grid.tag_size + grid.tag_size * grid.tag_spacing);

    // Calculate the x and y of each corner
    const vec4_t bottom_left(x, y, 0, 1);
    const vec4_t bottom_right(x + grid.tag_size, y, 0, 1);
    const vec4_t top_right(x + grid.tag_size, y + grid.tag_size, 0, 1);
    const vec4_t top_left(x, y + grid.tag_size, 0, 1);

    // Transform object points to corner points expressed in camera frame
    grid.points_CF.emplace_back((grid.T_CF * bottom_left).head(3));
    grid.points_CF.emplace_back((grid.T_CF * bottom_right).head(3));
    grid.points_CF.emplace_back((grid.T_CF * top_right).head(3));
    grid.points_CF.emplace_back((grid.T_CF * top_left).head(3));
  }
  grid.estimated = true;

  return 0;
}

void aprilgrid_imshow(const aprilgrid_t &grid,
                      const std::string &title,
                      const cv::Mat &image) {
  // Draw corners
  cv::Mat image_rgb = gray2rgb(image);

  for (size_t i = 0; i < grid.ids.size(); i++) {
    const size_t id = grid.ids[i];
    vec2s_t kps;
    if (aprilgrid_get(grid, id, kps) != 0) {
      return;
    }

    // Form corner points
    cv::Point2f p1(kps[0](0), kps[0](1)); // Bottom left
    cv::Point2f p2(kps[1](0), kps[1](1)); // Top left
    cv::Point2f p3(kps[2](0), kps[2](1)); // Top right
    cv::Point2f p4(kps[3](0), kps[3](1)); // Bottom right

    // Draw corners
    cv::circle(image_rgb, p1, 2, cv::Scalar(0, 0, 255), -1); // Bottom left
    cv::circle(image_rgb, p2, 2, cv::Scalar(0, 0, 255), -1); // Top left
    cv::circle(image_rgb, p3, 2, cv::Scalar(0, 0, 255), -1); // Top right
    cv::circle(image_rgb, p4, 2, cv::Scalar(0, 0, 255), -1); // Bottom right

    // // Draw Tag ID
    // cv::Point2f cxy(tags[i].cxy.first - 10, tags[i].cxy.second + 5);
    // cv::Scalar text_color(0, 0, 255);
    // std::string text = std::to_string(tags[i].id);
    // int font = cv::FONT_HERSHEY_PLAIN;
    // double font_scale = 1.0;
    // int thickness = 2;
    // cv::putText(image_rgb, text, cxy, font, font_scale, text_color,
    // thickness);
  }

  // Show
  cv::imshow(title, image_rgb);
  cv::waitKey(1);
}

int aprilgrid_save(const aprilgrid_t &grid, const std::string &save_path) {
  assert((grid.keypoints.size() % 4) == 0);
  assert((grid.points_CF.size() % 4) == 0);

  // Check save dir
  const std::string dir_path = dir_name(save_path);
  if (dir_create(dir_path) != 0) {
    LOG_ERROR("Could not create dir [%s]!", dir_path.c_str());
    return -1;
  }

  // Open file for saving
  std::ofstream outfile(save_path);
  if (outfile.good() != true) {
    LOG_ERROR("Failed to open [%s] for saving!", save_path.c_str());
    return -1;
  }

  // Output header
  // -- Configuration
  outfile << "configured,";
  outfile << "tag_rows,";
  outfile << "tag_cols,";
  outfile << "tag_size,";
  outfile << "tag_spacing,";
  // -- Keypoints
  outfile << "ts,id,kp_x,kp_y,";
  // -- Estimation
  outfile << "estimated,";
  outfile << "p_x,p_y,p_z,";
  outfile << "q_w,q_x,q_y,q_z,";
  outfile << "t_x,t_y,t_z";
  outfile << std::endl;

  // Decompose relative pose into rotation (quaternion) and translation
  const mat3_t R_CF = grid.T_CF.block(0, 0, 3, 3);
  const quat_t q_CF{R_CF};
  const vec3_t t_CF{grid.T_CF.block(0, 3, 3, 1)};

  // Output data
  for (size_t i = 0; i < grid.ids.size(); i++) {
    const int tag_id = grid.ids[i];

    for (int j = 0; j < 4; j++) {
      outfile << grid.configured << ",";
      outfile << grid.tag_rows << ",";
      outfile << grid.tag_cols << ",";
      outfile << grid.tag_size << ",";
      outfile << grid.tag_spacing << ",";

      const vec2_t keypoint = grid.keypoints[(i * 4) + j];
      outfile << grid.timestamp << ",";
      outfile << tag_id << ",";
      outfile << keypoint(0) << ",";
      outfile << keypoint(1) << ",";

      const vec3_t point_CF = grid.points_CF[(i * 4) + j];
      outfile << grid.estimated << ",";
      if (grid.estimated) {
        outfile << point_CF(0) << ",";
        outfile << point_CF(1) << ",";
        outfile << point_CF(2) << ",";
        outfile << q_CF.w() << ",";
        outfile << q_CF.x() << ",";
        outfile << q_CF.y() << ",";
        outfile << q_CF.z() << ",";
        outfile << t_CF(0) << ",";
        outfile << t_CF(1) << ",";
        outfile << t_CF(2) << std::endl;
      } else {
        outfile << "0,0,0,0,0,0,0,0,0,0" << std::endl;
      }
    }
  }

  // Close up
  outfile.close();
  return 0;
}

int aprilgrid_load(aprilgrid_t &grid, const std::string &data_path) {
  // Load data file
  matx_t data;
  if (csv2mat(data_path, true, data) != 0) {
    LOG_ERROR("Failed to load AprilGrid detection data [%s]!",
              data_path.c_str());
    return -1;
  }
  assert(data.cols() == 20);

  // Parse file
  grid.ids.clear();
  grid.keypoints.clear();
  grid.points_CF.clear();

  // Parse data
  for (int i = 0; i < data.rows(); i++) {
    const vecx_t row = data.row(i);

    // Configuration
    grid.configured = row(0);
    grid.tag_rows = row(1);
    grid.tag_cols = row(2);
    grid.tag_size = row(3);
    grid.tag_spacing = row(4);

    // Timestamp, tag id and keypoint
    grid.timestamp = row(5);
    if (std::count(grid.ids.begin(), grid.ids.end(), row(6)) == 0) {
      grid.ids.emplace_back(row(6));
    }
    grid.keypoints.emplace_back(row(7), row(8));

    // Point
    grid.estimated = row(9);
    grid.points_CF.emplace_back(row(10), row(11), row(12));

    // AprilGrid pose
    const quat_t q_CF{row(13), row(14), row(15), row(16)};
    const vec3_t t_CF{row(17), row(18), row(19)};
    grid.T_CF = transform(q_CF.toRotationMatrix(), t_CF);
  }

  return 0;
}

int aprilgrid_configure(aprilgrid_t &grid, const std::string &target_file) {
  // Load target file
  config_t config{target_file};
  if (config.ok == false) {
    return -1;
  }
  std::string target_type;
  parse(config, "target_type", target_type);
  parse(config, "tag_rows", grid.tag_rows);
  parse(config, "tag_cols", grid.tag_cols);
  parse(config, "tag_size", grid.tag_size);
  parse(config, "tag_spacing", grid.tag_spacing);

  // Double check tag type
  if (target_type != "aprilgrid") {
    FATAL("Invalid target_type [%s], expecting 'aprilgrid'!",
          target_type.c_str());
  }

  grid.configured = true;
  return 0;
}

void aprilgrid_filter_tags(const cv::Mat &image,
                           std::vector<apriltag_t> &tags) {
  std::vector<apriltag_t>::iterator iter = tags.begin();

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

int aprilgrid_detect(aprilgrid_t &grid,
                     const aprilgrid_detector_t &detector,
                     const cv::Mat &image) {
  assert(grid.configured);

  // Convert image to gray-scale
  const cv::Mat image_gray = rgb2gray(image);

  // Extract tags
  std::vector<apriltag_t> tags = detector.det.extractTags(image_gray);
  aprilgrid_filter_tags(image, tags);
  std::sort(tags.begin(), tags.end(), sort_apriltag_by_id);

  // Form results
  for (const auto tag : tags) {
    std::vector<cv::Point2f> img_pts;
    img_pts.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    img_pts.emplace_back(tag.p[1].first, tag.p[1].second); // Top left
    img_pts.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    img_pts.emplace_back(tag.p[3].first, tag.p[3].second); // Bottom right
    aprilgrid_add(grid, tag.id, img_pts);
  }

  return grid.nb_detections;
}

int aprilgrid_detect(aprilgrid_t &grid,
                     const aprilgrid_detector_t &detector,
                     const cv::Mat &image,
                     const mat3_t &cam_K,
                     const vec4_t &cam_D) {
  assert(grid.configured);
  if (aprilgrid_detect(grid, detector, image) == 0) {
    return 0;
  }
  aprilgrid_calc_relative_pose(grid, cam_K, cam_D);

  return grid.nb_detections;
}

std::ostream &operator<<(std::ostream &os, const aprilgrid_t &aprilgrid) {
  // Relative rotation and translation
  if (aprilgrid.estimated) {
    os << "AprilGrid: " << std::endl;
    os << "configured: " << aprilgrid.configured << std::endl;
    os << "tag_rows: " << aprilgrid.tag_rows << std::endl;
    os << "tag_cols: " << aprilgrid.tag_cols << std::endl;
    os << "tag_size: " << aprilgrid.tag_size << std::endl;
    os << "tag_spacing: " << aprilgrid.tag_spacing << std::endl;
    os << std::endl;
  }

  for (size_t i = 0; i < aprilgrid.ids.size(); i++) {
    // Tag id
    os << "tag_id: " << aprilgrid.ids[i] << std::endl;
    os << "ts: " << aprilgrid.timestamp << std::endl;
    os << "----------" << std::endl;

    // Keypoint and relative position
    for (size_t j = 0; j < 4; j++) {
      os << "keypoint: " << aprilgrid.keypoints[(i * 4) + j].transpose();
      os << std::endl;
      os << "estimated: " << aprilgrid.estimated << std::endl;
      if (aprilgrid.estimated) {
        os << "p_CF: " << aprilgrid.points_CF[(i * 4) + j].transpose();
        os << std::endl;
      }
      os << std::endl;
    }
    os << std::endl;
  }

  return os;
}

} // namespace prototype
