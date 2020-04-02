#include "proto/vision/vision.hpp"

namespace proto {

/*****************************************************************************
 *                             VISION COMMON
 ****************************************************************************/

bool is_equal(const cv::Mat &m1, const cv::Mat &m2) {
  // pre-check
  if (m1.empty() && m2.empty()) {
    return true;
  }

  // check dimensions
  if (m1.cols != m2.cols) {
    return false;
  } else if (m1.rows != m2.rows) {
    return false;
  } else if (m1.dims != m2.dims) {
    return false;
  }

  // check matrix elements
  cv::Mat diff;
  cv::compare(m1, m2, diff, cv::CMP_NE);

  return cv::countNonZero(diff) ? false : true;
}

void convert(const cv::Mat &x, matx_t &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<real_t>(i, j);
    }
  }
}

void convert(const matx_t &x, cv::Mat &y) {
  y = cv::Mat(x.rows(), x.cols(), cv::DataType<real_t>::type);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      y.at<real_t>(i, j) = x(i, j);
    }
  }
}

matx_t convert(const cv::Mat &x) {
  matx_t y;
  convert(x, y);
  return y;
}

cv::Mat convert(const matx_t &x) {
  cv::Mat y;
  convert(x, y);
  return y;
}

vec3_t homogeneous(const vec2_t &x) { return vec3_t{x(0), x(1), 1.0}; }

vec4_t homogeneous(const vec3_t &x) { return vec4_t{x(0), x(1), x(2), 1.0}; }

mat4_t rvectvec2transform(const cv::Mat &rvec, const cv::Mat &tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  mat4_t T;
  T.block(0, 0, 3, 3) = convert(R);
  T.block(0, 3, 3, 1) = convert(tvec);
  T(3, 3) = 1.0;

  return T;
}

cv::Matx33d skew(const cv::Vec3d &v) {
  // clang-format off
  return cv::Matx33d(0.0, -v[2], v[1],
                     v[2], 0.0, -v[0],
                     -v[1], v[0], 0.0);
  // clang-format on
}

std::vector<cv::KeyPoint>
sort_keypoints(const std::vector<cv::KeyPoint> keypoints, const size_t limit) {
  if (keypoints.size() == 0) {
    return std::vector<cv::KeyPoint>();
  }

  // Obtain vector responses
  std::vector<int> responses;
  for (size_t i = 0; i < keypoints.size(); i++) {
    responses.push_back(keypoints[i].response);
  }

  // Sort responses
  std::vector<int> index(responses.size());
  std::iota(std::begin(index), std::end(index), 0);
  cv::sortIdx(responses, index, CV_SORT_DESCENDING);

  // Form sorted keypoints
  std::vector<cv::KeyPoint> keypoints_sorted;
  for (size_t i = 0; i < keypoints.size(); i++) {
    keypoints_sorted.push_back(keypoints[index[i]]);
    if (keypoints_sorted.size() == limit) {
      break;
    }
  }

  return keypoints_sorted;
}

cv::Mat gray2rgb(const cv::Mat &image) {
  const int image_height = image.rows;
  const int image_width = image.cols;
  cv::Mat out_image(image_height, image_width, CV_8UC3);

  if (image.channels() == 1) {
    cv::cvtColor(image, out_image, CV_GRAY2RGB);
  } else {
    return image.clone();
  }

  return out_image;
}

cv::Mat rgb2gray(const cv::Mat &image) {
  cv::Mat image_gray;

  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  } else {
    return image.clone();
  }

  return image_gray;
}

cv::Mat roi(const cv::Mat &image,
            const int width,
            const int height,
            const real_t cx,
            const real_t cy) {
  const real_t x = cx - width / 2.0;
  const real_t y = cy - height / 2.0;
  cv::Rect roi(x, y, width, height);
  return image(roi);
}

bool keypoint_compare_by_response(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2) {
  // Keypoint with higher response will be at the beginning of the vector
  return kp1.response > kp2.response;
}

bool is_rot_mat(const cv::Mat &R) {
  const cv::Mat Rt;
  cv::transpose(R, Rt);
  const cv::Mat I = cv::Mat::eye(3, 3, (Rt * R).type());
  return cv::norm(I, Rt * R) < 1e-6;
}

cv::Vec3f rot2euler(const cv::Mat &R) {
  assert(is_rot_mat(R));
  const real_t R00 = R.at<real_t>(0, 0);
  const real_t R10 = R.at<real_t>(1, 0);
  const float sy = sqrt(R00 * R00 + R10 * R10);
  bool singular = sy < 1e-6;

  float x;
  float y;
  float z;
  if (!singular) {
    x = atan2(R.at<real_t>(2, 1), R.at<real_t>(2, 2));
    y = atan2(-R.at<real_t>(2, 0), sy);
    z = atan2(R.at<real_t>(1, 0), R.at<real_t>(0, 0));

  } else {
    x = atan2(-R.at<real_t>(1, 2), R.at<real_t>(1, 1));
    y = atan2(-R.at<real_t>(2, 0), sy);
    z = 0;
  }

  return cv::Vec3f(x, y, z);
}

float rescale_points(std::vector<cv::Point2f> &pts1,
                     std::vector<cv::Point2f> &pts2) {
  // Calculate scaling factor
  float scaling_factor = 0.0f;
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

  return scaling_factor;
}

real_t reprojection_error(const vec2s_t &measured, const vec2s_t &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += (measured[i] - projected[i]).norm();
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

real_t reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected) {
  assert(measured.size() == projected.size());

  real_t sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += cv::norm(measured[i] - projected[i]);
  }
  const real_t rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::Point2f> points,
                    const int patch_width) {
  matx_t mask = ones(image_height, image_width);

  // Create a mask around each point
  for (const auto &p : points) {
    // Skip if pixel is out of image bounds
    const real_t px = static_cast<int>(p.x);
    const real_t py = static_cast<int>(p.y);
    if (px >= image_width || px <= 0) {
      continue;
    } else if (py >= image_height || py <= 0) {
      continue;
    }

    // Calculate patch top left corner, patch width and height
    vec2_t top_left{px - patch_width, py - patch_width};
    vec2_t top_right{px + patch_width, py - patch_width};
    vec2_t btm_left{px - patch_width, py + patch_width};
    vec2_t btm_right{px + patch_width, py + patch_width};
    std::vector<vec2_t *> corners{&top_left, &top_right, &btm_left, &btm_right};
    for (auto corner : corners) {
      // Check corner in x-axis
      if ((*corner)(0) < 0) {
        (*corner)(0) = 0;
      } else if ((*corner)(0) > image_width) {
        (*corner)(0) = image_width;
      }

      // Check corner in y-axis
      if ((*corner)(1) < 0) {
        (*corner)(1) = 0;
      } else if ((*corner)(1) > image_height) {
        (*corner)(1) = image_height;
      }
    }

    // Create mask around pixel
    const int row = top_left(1);
    const int col = top_left(0);
    int width = top_right(0) - top_left(0) + 1;
    int height = btm_left(1) - top_left(1) + 1;
    width = (col + width) > image_width ? width - 1 : width;
    height = (row + height) > image_height ? height - 1 : height;

    // std::cout << "---" << std::endl;
    // std::cout << image_width << std::endl;
    // std::cout << image_height << std::endl;
    // std::cout << row << std::endl;
    // std::cout << col << std::endl;
    // std::cout << width << std::endl;
    // std::cout << height << std::endl;
    // std::cout << "---" << std::endl;

    mask.block(row, col, height, width) = zeros(height, width);
  }

  return mask;
}

matx_t feature_mask(const int image_width,
                    const int image_height,
                    const std::vector<cv::KeyPoint> keypoints,
                    const int patch_width) {
  std::vector<cv::Point2f> points;
  for (const auto &kp : keypoints) {
    points.emplace_back(kp.pt);
  }

  return feature_mask(image_width, image_height, points, patch_width);
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::Point2f> points,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, points, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat feature_mask_opencv(const int image_width,
                            const int image_height,
                            const std::vector<cv::KeyPoint> keypoints,
                            const int patch_width) {
  auto mask = feature_mask(image_width, image_height, keypoints, patch_width);

  cv::Mat mask_cv;
  cv::eigen2cv(mask, mask_cv);
  mask_cv.convertTo(mask_cv, CV_8UC1);

  return mask_cv;
}

cv::Mat radtan_undistort_image(const mat3_t &K,
                               const vecx_t &D,
                               const cv::Mat &image) {
  cv::Mat image_ud;
  cv::Mat K_ud = convert(K).clone();
  cv::undistort(image, image_ud, convert(K), convert(D), K_ud);
  return image_ud;
}

cv::Mat equi_undistort_image(const mat3_t &K,
                             const vecx_t &D,
                             const cv::Mat &image,
                             const real_t balance,
                             cv::Mat &Knew) {
  // Estimate new camera matrix first
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(convert(K),
                                                          convert(D),
                                                          image.size(),
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, convert(K), convert(D), Knew);

  return image_ud;
}

void illum_invar_transform(cv::Mat &image,
                           const real_t lambda_1,
                           const real_t lambda_2,
                           const real_t lambda_3) {
  // The following method is adapted from:
  // Illumination Invariant Imaging: Applications in Robust Vision-based
  // Localisation, Mapping and Classification for Autonomous Vehicles
  // Maddern et al (2014)

  // clang-format off
  real_t alpha = (lambda_1 * lambda_3 - lambda_1 * lambda_2) /
          			 (lambda_2 * lambda_3 - lambda_1 * lambda_2);
  // clang-format on

  std::vector<cv::Mat> channels(3);
  split(image, channels);
  channels[0].convertTo(channels[0], CV_32F);
  channels[1].convertTo(channels[1], CV_32F);
  channels[2].convertTo(channels[2], CV_32F);

  channels[0].row(0).setTo(cv::Scalar(1));
  channels[1].row(0).setTo(cv::Scalar(1));
  channels[2].row(0).setTo(cv::Scalar(1));

  cv::Mat log_ch_1, log_ch_2, log_ch_3;
  cv::log(channels[0] / 255.0, log_ch_1);
  cv::log(channels[1] / 255.0, log_ch_2);
  cv::log(channels[2] / 255.0, log_ch_3);

  image = 0.5 + log_ch_2 - alpha * log_ch_3 - (1 - alpha) * log_ch_1;
  image.setTo(0, image < 0);
  image.setTo(1, image > 1);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}

/*****************************************************************************
 *                                DRAW
 ****************************************************************************/

cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status) {
  // Draw tracks
  for (size_t i = 0; i < status.size(); i++) {
    // Check if point was lost
    if (status[i] == 0) {
      continue;
    }

    // Draw circle and line
    cv::circle(img_cur, p0[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img_cur, p1[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::line(img_cur, p0[i], p1[i], cv::Scalar(0, 255, 0));
  }

  return img_cur;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i]) {
      cv::Point2f p0 = k0[i];
      cv::Point2f p1 = k1[i];

      // Point 1
      p1.y += img0.rows;

      // Draw circle and line
      cv::circle(match_img, p0, 2, cv::Scalar(0, 255, 0), -1);
      cv::circle(match_img, p1, 2, cv::Scalar(0, 255, 0), -1);
      cv::line(match_img, p0, p1, cv::Scalar(0, 255, 0));
    }
  }

  return match_img;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < matches.size(); i++) {
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    cv::KeyPoint p0 = k0[k0_idx];
    cv::KeyPoint p1 = k1[k1_idx];

    // Point 1
    p1.pt.y += img0.rows;

    // Draw circle and line
    cv::circle(match_img, p0.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0.pt, p1.pt, cv::Scalar(0, 255, 0));
  }

  return match_img;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features) {
  cv::Mat out_image = image.clone();

  // Draw corners
  for (auto p : features) {
    cv::circle(out_image, p, 2, cv::Scalar(0, 255, 0), -1);
  }

  // Draw vertical lines
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;

  for (int x = dx; x < image_width; x += dx) {
    const cv::Point start(x, 0);
    const cv::Point end(x, image_height);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  // Draw horizontal lines
  for (int y = dy; y < image_height; y += dy) {
    const cv::Point start(0, y);
    const cv::Point end(image_width, y);
    const cv::Scalar color(0, 0, 255);
    cv::line(out_image, start, end, color, 2);
  }

  return out_image;
}

cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features) {
  std::vector<cv::Point2f> points;
  for (const auto &f : features) {
    points.emplace_back(f.pt);
  }

  return draw_grid_features(image, grid_rows, grid_cols, points);
}

/*****************************************************************************
 *                              FEATURES2D
 ****************************************************************************/

std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int max_corners,
                                    const int grid_rows,
                                    const int grid_cols,
                                    const real_t threshold,
                                    const bool nonmax_suppression) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_corners_per_cell = (float) max_corners / (float) nb_cells;

  // Detect corners in each grid cell
  std::vector<cv::KeyPoint> keypoints_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      cv::Rect roi = cv::Rect(x, y, w, h);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image_gray(roi), keypoints, threshold, nonmax_suppression);

      // Sort by keypoint response
      keypoints = sort_keypoints(keypoints);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      std::vector<cv::KeyPoint> keypoints_adjusted;
      for (auto &kp : keypoints) {
        keypoints_adjusted.emplace_back(kp.pt.x += x, kp.pt.y += y, kp.size);
        if (keypoints_adjusted.size() == max_corners_per_cell) {
          break;
        }
      }

      // Add to total keypoints detected
      keypoints_all.insert(std::end(keypoints_all),
                           std::begin(keypoints_adjusted),
                           std::end(keypoints_adjusted));
    }
  }

  return keypoints_all;
}

std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners,
                                   const int grid_rows,
                                   const int grid_cols,
                                   const real_t quality_level,
                                   const real_t min_distance,
                                   const cv::Mat mask,
                                   const int block_size,
                                   const bool use_harris_detector,
                                   const real_t k) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray = rgb2gray(image);

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_corners_per_cell = (float) max_corners / (float) nb_cells;

  // Detect corners in each grid cell
  std::vector<cv::Point2f> corners_all;

  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const real_t w = (x + dx > image_width) ? image_width - x : dx;
      const real_t h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      const cv::Rect roi = cv::Rect(x, y, w, h);
      const cv::Mat sub_mask = (mask.rows == 0) ? cv::Mat() : mask(roi);
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(image_gray(roi),
                              corners,
                              max_corners,
                              quality_level,
                              min_distance,
                              sub_mask,
                              block_size,
                              use_harris_detector,
                              k);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      std::vector<cv::Point2f> corners_adjusted;
      for (auto &p : corners) {
        corners_adjusted.emplace_back(p.x += x, p.y += y);
        if (corners_adjusted.size() == max_corners_per_cell) {
          break;
        }
      }

      // Add to total corners detected
      corners_all.insert(std::end(corners_all),
                         std::begin(corners_adjusted),
                         std::end(corners_adjusted));
    }
  }

  return corners_all;
}

/****************************************************************************
 *                            RADIAL-TANGENTIAL
 ***************************************************************************/

radtan4_t::radtan4_t() {}

radtan4_t::radtan4_t(const real_t *distortion_)
  : k1{distortion_[0]}, k2{distortion_[1]},
    p1{distortion_[2]}, p2{distortion_[3]} {}

radtan4_t::radtan4_t(const vec4_t &distortion_)
  : k1{distortion_(0)}, k2{distortion_(1)},
    p1{distortion_(2)}, p2{distortion_(3)} {}

radtan4_t::radtan4_t(const real_t k1_,
                     const real_t k2_,
                     const real_t p1_,
                     const real_t p2_)
  : k1{k1_}, k2{k2_}, p1{p1_}, p2{p2_} {}

radtan4_t::radtan4_t(radtan4_t &radtan4)
  : k1{radtan4.k1}, k2{radtan4.k2}, p1{radtan4.p1}, p2{radtan4.p2} {}

radtan4_t::radtan4_t(const radtan4_t &radtan4)
  : k1{radtan4.k1}, k2{radtan4.k2}, p1{radtan4.p1}, p2{radtan4.p2} {}

radtan4_t::~radtan4_t() {}

vec2_t radtan4_t::distort(const vec2_t &p) {
  return static_cast<const radtan4_t &>(*this).distort(p);
}

vec2_t radtan4_t::distort(const vec2_t &p) const {
  const real_t x = p(0);
  const real_t y = p(1);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  return vec2_t{x_ddash, y_ddash};
}

mat2_t radtan4_t::J_point(const vec2_t &p) {
  return static_cast<const radtan4_t &>(*this).J_point(p);
}

mat2_t radtan4_t::J_point(const vec2_t &p) const {
  const real_t x = p(0);
  const real_t y = p(1);

  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  // clang-format off
  mat2_t J_point;
  J_point(0, 0) = 1 + k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(1, 0) = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1 + k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2);
  // clang-format on
  // Above is generated using sympy

  return J_point;
}

mat_t<2, 4> radtan4_t::J_param(const vec2_t &p) {
  return static_cast<const radtan4_t &>(*this).J_param(p);
}

mat_t<2, 4> radtan4_t::J_param(const vec2_t &p) const {
  const real_t x = p(0);
  const real_t y = p(1);

  const real_t xy = x * y;
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  mat_t<2, 4> J_params = zeros(2, 4);
  J_params(0, 0) = x * r2;
  J_params(0, 1) = x * r4;
  J_params(0, 2) = 2 * xy;
  J_params(0, 3) = 3 * x2 + y2;

  J_params(1, 0) = y * r2;
  J_params(1, 1) = y * r4;
  J_params(1, 2) = x2 + 3 * y2;
  J_params(1, 3) = 2 * xy;

  return J_params;
}

void radtan4_t::operator=(const radtan4_t &src) throw() {
  k1 = src.k1;
  k2 = src.k2;
  p1 = src.p1;
  p2 = src.p2;
}

std::ostream &operator<<(std::ostream &os, const radtan4_t &radtan4) {
  os << "k1: " << radtan4.k1 << std::endl;
  os << "k2: " << radtan4.k2 << std::endl;
  os << "p1: " << radtan4.p1 << std::endl;
  os << "p2: " << radtan4.p2 << std::endl;
  return os;
}

vec4_t distortion_coeffs(const radtan4_t &radtan) {
  return vec4_t{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
}

vec2_t distort(const radtan4_t &radtan, const vec2_t &point) {
  const real_t k1 = radtan.k1;
  const real_t k2 = radtan.k2;
  const real_t p1 = radtan.p1;
  const real_t p2 = radtan.p2;
  const real_t x = point(0);
  const real_t y = point(1);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  return vec2_t{x_ddash, y_ddash};
}

vec2_t distort(const radtan4_t &radtan, const vec2_t &p, mat2_t &J_point) {
  const real_t k1 = radtan.k1;
  const real_t k2 = radtan.k2;
  const real_t p1 = radtan.p1;
  const real_t p2 = radtan.p2;
  const real_t x = p(0);
  const real_t y = p(1);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  // clang-format off
  J_point(0, 0) = 1 + k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(1, 0) = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1 + k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2);
  // clang-format on
  // Above is generated using sympy

  return vec2_t{x_ddash, y_ddash};
}

vec2_t distort(const radtan4_t &radtan,
               const vec2_t &p,
               mat2_t &J_point,
               mat_t<2, 4> &J_params) {
  const vec2_t p_distorted = distort(radtan, p, J_point);

  const real_t x = p(0);
  const real_t y = p(1);

  const real_t xy = x * y;
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  J_params(0, 0) = x * r2;
  J_params(0, 1) = x * r4;
  J_params(0, 2) = 2 * xy;
  J_params(0, 3) = 3 * x2 + y2;

  J_params(1, 0) = y * r2;
  J_params(1, 1) = y * r4;
  J_params(1, 2) = x2 + 3 * y2;
  J_params(1, 3) = 2 * xy;

  return p_distorted;
}

matx_t distort(const radtan4_t &radtan, const matx_t &points) {
  assert(points.rows() == 2);
  assert(points.cols() > 0);

  const real_t k1 = radtan.k1;
  const real_t k2 = radtan.k2;
  const real_t p1 = radtan.p1;
  const real_t p2 = radtan.p2;
  const arrayx_t x = points.row(0).array();
  const arrayx_t y = points.row(1).array();

  // Apply radial distortion
  const arrayx_t x2 = x * x;
  const arrayx_t y2 = y * y;
  const arrayx_t r2 = x2 + y2;
  const arrayx_t r4 = r2 * r2;
  const arrayx_t x_dash = x * (1 + (k1 * r2) + (k2 * r4));
  const arrayx_t y_dash = y * (1 + (k1 * r2) + (k2 * r4));

  // Apply tangential distortion
  const arrayx_t xy = x * y;
  const arrayx_t x_ddash = x_dash + (2 * p1 * xy + p2 * (r2 + 2 * x2));
  const arrayx_t y_ddash = y_dash + (p1 * (r2 + 2 * y2) + 2 * p2 * xy);

  // Form results
  const int nb_points = points.cols();
  matx_t distorted_points{2, nb_points};
  distorted_points.row(0) = x_ddash;
  distorted_points.row(1) = y_ddash;

  return distorted_points;
}

vec2_t undistort(const radtan4_t &radtan,
                 const vec2_t &p0,
                 const int max_iter) {
  vec2_t p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const vec2_t p_distorted = distort(radtan, p);
    const vec2_t err = (p0 - p_distorted);

    // Jacobian
    mat2_t J;
    distort(radtan, p, J);
    const mat2_t pinv = (J.transpose() * J).inverse() * J.transpose();
    const vec2_t dp = pinv * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

/****************************************************************************
 *                              EQUI-DISTANCE
 ***************************************************************************/

equi4_t::equi4_t(const real_t k1_,
                 const real_t k2_,
                 const real_t k3_,
                 const real_t k4_)
    : k1{k1_}, k2{k2_}, k3{k3_}, k4{k4_} {}

equi4_t::~equi4_t() {}

std::ostream &operator<<(std::ostream &os, const equi4_t &equi4) {
  os << "k1: " << equi4.k1 << std::endl;
  os << "k2: " << equi4.k2 << std::endl;
  os << "k3: " << equi4.k3 << std::endl;
  os << "k4: " << equi4.k4 << std::endl;
  return os;
}

vec2_t distort(const equi4_t &equi, const vec2_t &point) {
  const real_t k1 = equi.k1;
  const real_t k2 = equi.k2;
  const real_t k3 = equi.k3;
  const real_t k4 = equi.k4;
  const real_t x = point(0);
  const real_t y = point(1);
  const real_t r = sqrt(pow(x, 2) + pow(y, 2));

  if (r < 1e-8) {
    return point;
  }

  // Apply equi distortion
  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t th_d = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t x_dash = (th_d / r) * x;
  const real_t y_dash = (th_d / r) * y;

  return vec2_t{x_dash, y_dash};
}

vec2_t distort(const equi4_t &equi, const vec2_t &point, mat2_t &J_point) {
  const real_t k1 = equi.k1;
  const real_t k2 = equi.k2;
  const real_t k3 = equi.k3;
  const real_t k4 = equi.k4;
  const real_t x = point(0);
  const real_t y = point(1);
  const real_t r = sqrt(pow(x, 2) + pow(y, 2));

  if (r < 1e-8) {
    J_point = I(2);
    return point;
  }

  // Apply equi distortion
  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;
  const real_t x_dash = s * x;
  const real_t y_dash = s * y;

  // Form jacobian
  // clang-format off
  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;
  J_point(0,0) = s + x * s_r * r_x;
  J_point(0,1) = x * s_r * r_y;
  J_point(1,0) = y * s_r * r_x;
  J_point(1,1) = s + y * s_r * r_y;
  // clang-format on

  return vec2_t{x_dash, y_dash};
}

matx_t distort(const equi4_t &equi, const matx_t &points) {
  assert(points.rows() == 2);
  assert(points.cols() > 0);

  // Setup
  const real_t k1 = equi.k1;
  const real_t k2 = equi.k2;
  const real_t k3 = equi.k3;
  const real_t k4 = equi.k4;
  const arrayx_t x = points.row(0).array();
  const arrayx_t y = points.row(1).array();
  const arrayx_t r = (x.pow(2) + y.pow(2)).sqrt();

  // Apply equi distortion
  const auto th = r.atan();
  const auto th2 = th.pow(2);
  const auto th4 = th.pow(4);
  const auto th6 = th.pow(6);
  const auto th8 = th.pow(8);
  const auto thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const auto s = thd / r;
  const auto x_dash = s * x;
  const auto y_dash = s * y;

  // Project equi distorted points to image plane
  const int nb_points = points.cols();
  matx_t distorted_points{2, nb_points};
  distorted_points.row(0) = x_dash;
  distorted_points.row(1) = y_dash;
  return distorted_points;
}

vec2_t undistort(const equi4_t &equi, const vec2_t &p) {
  const real_t k1 = equi.k1;
  const real_t k2 = equi.k2;
  const real_t k3 = equi.k3;
  const real_t k4 = equi.k4;
  const real_t thd = sqrt(p(0) * p(0) + p(1) * p(1));

  real_t th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const real_t scaling = tan(th) / thd;
  vec2_t p_ud{p(0) * scaling, p(1) * scaling};
  return p_ud;
}

/****************************************************************************
 *                                PINHOLE
 ***************************************************************************/

pinhole_t::pinhole_t() {}

pinhole_t::pinhole_t(const real_t *intrinsics_)
    : fx{intrinsics_[0]}, fy{intrinsics_[1]}, cx{intrinsics_[2]},
      cy{intrinsics_[3]} {}

pinhole_t::pinhole_t(const vec4_t &intrinsics_)
    : fx{intrinsics_(0)}, fy{intrinsics_(1)}, cx{intrinsics_(2)},
      cy{intrinsics_(3)} {}

pinhole_t::pinhole_t(const mat3_t &K_)
    : fx{K_(0, 0)}, fy{K_(1, 1)}, cx{K_(0, 2)}, cy{K_(1, 2)} {}

pinhole_t::pinhole_t(const real_t fx_,
                     const real_t fy_,
                     const real_t cx_,
                     const real_t cy_)
    : fx{fx_}, fy{fy_}, cx{cx_}, cy{cy_} {}

pinhole_t::pinhole_t(pinhole_t &pinhole)
    : fx{pinhole.fx}, fy{pinhole.fy}, cx{pinhole.cx}, cy{pinhole.cy} {}

pinhole_t::pinhole_t(const pinhole_t &pinhole)
    : fx{pinhole.fx}, fy{pinhole.fy}, cx{pinhole.cx}, cy{pinhole.cy} {}

pinhole_t::~pinhole_t() {}

vec2_t pinhole_t::project(const vec2_t &p) {
  return static_cast<const pinhole_t &>(*this).project(p);
}

vec2_t pinhole_t::project(const vec2_t &p) const {
  return vec2_t{p(0) * fx + cx, p(1) * fy + cy};
}

mat2_t pinhole_t::J_point() {
  return static_cast<const pinhole_t &>(*this).J_point();
}

mat2_t pinhole_t::J_point() const {
  mat2_t J_K = zeros(2, 2);
  J_K(0, 0) = fx;
  J_K(1, 1) = fy;
  return J_K;
}

mat_t<2, 4> pinhole_t::J_param(const vec2_t &p) {
  return static_cast<const pinhole_t &>(*this).J_param(p);
}

mat_t<2, 4> pinhole_t::J_param(const vec2_t &p) const {
  const real_t x = p(0);
  const real_t y = p(1);

  mat_t<2, 4> J_param = zeros(2, 4);
  J_param(0, 0) = x;
  J_param(1, 1) = y;
  J_param(0, 2) = 1;
  J_param(1, 3) = 1;

  return J_param;
}

void pinhole_t::operator=(const pinhole_t &src) throw() {
  fx = src.fx;
  fy = src.fy;
  cx = src.cx;
  cy = src.cy;
}

std::ostream &operator<<(std::ostream &os, const pinhole_t &pinhole) {
  os << "fx: " << pinhole.fx << std::endl;
  os << "fy: " << pinhole.fy << std::endl;
  os << "cx: " << pinhole.cx << std::endl;
  os << "cy: " << pinhole.cy << std::endl;
  return os;
}

mat3_t
pinhole_K(const real_t fx, const real_t fy, const real_t cx, const real_t cy) {
  mat3_t K;
  // clang-format off
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

mat3_t pinhole_K(const pinhole_t &pinhole) { return pinhole_K(*pinhole.data); }

mat3_t pinhole_K(const vec2_t &image_size,
                 const real_t lens_hfov,
                 const real_t lens_vfov) {
  const real_t fx = pinhole_focal_length(image_size(0), lens_hfov);
  const real_t fy = pinhole_focal_length(image_size(1), lens_vfov);
  const real_t cx = image_size(0) / 2.0;
  const real_t cy = image_size(1) / 2.0;
  return pinhole_K(fx, fy, cx, cy);
}

mat34_t pinhole_P(const mat3_t &K, const mat3_t &C_WC, const vec3_t &r_WC) {
  mat34_t A;
  A.block(0, 0, 3, 3) = C_WC;
  A.block(0, 3, 3, 1) = -C_WC * r_WC;
  const mat34_t P = K * A;
  return P;
}

real_t pinhole_focal_length(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

vec2_t pinhole_focal_length(const vec2_t &image_size,
                            const real_t hfov,
                            const real_t vfov) {
  const real_t fx = ((image_size(0) / 2.0) / tan(deg2rad(hfov) / 2.0));
  const real_t fy = ((image_size(1) / 2.0) / tan(deg2rad(vfov) / 2.0));
  return vec2_t{fx, fy};
}

vec2_t project(const vec3_t &p) { return vec2_t{p(0) / p(2), p(1) / p(2)}; }

vec2_t project(const vec3_t &p, mat_t<2, 3> &J_P) {
  const real_t x = p(0);
  const real_t y = p(1);
  const real_t z = p(2);

  // Projection Jacobian
  J_P = zeros(2, 3);
  J_P(0, 0) = 1.0 / z;
  J_P(1, 1) = 1.0 / z;
  J_P(0, 2) = -x / (z * z);
  J_P(1, 2) = -y / (z * z);

  return vec2_t{x / z, x / z};
}

vec2_t project(const pinhole_t &model, const vec2_t &p) {
  return vec2_t{p(0) * model.fx + model.cx, p(1) * model.fy + model.cy};
}

vec2_t project(const pinhole_t &model, const vec3_t &p) {
  const real_t px = p(0) / p(2);
  const real_t py = p(1) / p(2);
  return vec2_t{px * model.fx + model.cx, py * model.fy + model.cy};
}

vec2_t project(const pinhole_t &model, const vec3_t &p, mat_t<2, 3> &J_h) {
  const real_t x = p(0);
  const real_t y = p(1);
  const real_t z = p(2);

  const real_t px = x / z;
  const real_t py = y / z;

  // Projection Jacobian
  mat_t<2, 3> J_P = zeros(2, 3);
  J_P(0, 0) = 1.0 / z;
  J_P(1, 1) = 1.0 / z;
  J_P(0, 2) = -x / (z * z);
  J_P(1, 3) = -y / (z * z);

  // Intrinsics Jacobian
  mat2_t J_K = model.J_point();

  // Measurement Jacobian
  J_h = J_K * J_P;

  return vec2_t{px * model.fx + model.cx, py * model.fy + model.cy};
}

} //  namespace proto
