#include "prototype/vision/util.hpp"

namespace proto {

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
      y(i, j) = x.at<double>(i, j);
    }
  }
}

void convert(const matx_t &x, cv::Mat &y) {
  y = cv::Mat(x.rows(), x.cols(), cv::DataType<double>::type);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      y.at<double>(i, j) = x(i, j);
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
            const double cx,
            const double cy) {
  const double x = cx - width / 2.0;
  const double y = cy - height / 2.0;
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
  const double R00 = R.at<double>(0, 0);
  const double R10 = R.at<double>(1, 0);
  const float sy = sqrt(R00 * R00 + R10 * R10);
  bool singular = sy < 1e-6;

  float x;
  float y;
  float z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
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

double reprojection_error(const vec2s_t &measured, const vec2s_t &projected) {
  assert(measured.size() == projected.size());

  double sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += (measured[i] - projected[i]).norm();
  }
  const double rmse = sqrt(sse / nb_keypoints);

  return rmse;
}

double reprojection_error(const std::vector<cv::Point2f> &measured,
                          const std::vector<cv::Point2f> &projected) {
  assert(measured.size() == projected.size());

  double sse = 0.0;
  const size_t nb_keypoints = measured.size();
  for (size_t i = 0; i < nb_keypoints; i++) {
    sse += cv::norm(measured[i] - projected[i]);
  }
  const double rmse = sqrt(sse / nb_keypoints);

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
    const double px = static_cast<int>(p.x);
    const double py = static_cast<int>(p.y);
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
                             cv::Mat &Knew,
                             const double balance) {
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

} //  namespace proto
