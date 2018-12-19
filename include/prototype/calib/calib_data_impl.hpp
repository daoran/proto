#ifndef PROTOTYPE_CALIB_CALIB_DATA_IMPL_HPP
#define PROTOTYPE_CALIB_CALIB_DATA_IMPL_HPP

#include "calib_data.hpp"

namespace proto {

template <typename CM, typename DM>
cv::Mat validate_intrinsics(const cv::Mat &image,
                            const vec2s_t &keypoints,
                            const vec3s_t &points,
                            const camera_geometry_t<CM, DM> &camera_geometry) {
  assert(image.empty() == false);
  assert(keypoints.size() != 0);
  assert(points.size() != 0);
  assert(keypoints.size() == points.size());

  // Project points to image plane
  vec2s_t projected;
  for (const auto &point : points) {
    const auto p = camera_geometry_project(camera_geometry, point);
    projected.emplace_back(p);
  }

  // Colors
  const cv::Scalar red{0, 0, 255};
  const cv::Scalar green{0, 255, 0};

  // Draw detected chessboard corners
  const auto keypoints_color = red;
  const auto projected_color = green;
  cv::Mat result = draw_calib_validation(image,
                                         keypoints,
                                         projected,
                                         keypoints_color,
                                         projected_color);

  return result;
}

template <typename CM, typename DM>
cv::Mat validate_stereo(const cv::Mat &image0,
                        const cv::Mat &image1,
                        const vec2s_t &kps0,
                        const vec3s_t &points0,
                        const vec2s_t &kps1,
                        const vec3s_t &points1,
                        const camera_geometry_t<CM, DM> &cam0,
                        const camera_geometry_t<CM, DM> &cam1,
                        const mat4_t &T_C0C1) {
  assert(image0.empty() == false);
  assert(image1.empty() == false);
  assert(kps0.size() == kps1.size());
  assert(points0.size() == points1.size());

  if (kps0.size() == 0 || kps1.size() == 0) {
    cv::Mat result;
    cv::vconcat(image0, image1, result);
    return result;
  }

  // Project points observed in cam1 to cam0 image plane
  vec2s_t projected0;
  for (const auto &point_C1F : points1) {
    const auto point_C0F = (T_C0C1 * point_C1F.homogeneous()).head(3);
    const auto p = camera_geometry_project(cam0, point_C0F);
    projected0.emplace_back(p);
  }

  // Project points observed in cam0 to cam1 image plane
  vec2s_t projected1;
  const mat4_t T_C1C0 = T_C0C1.inverse();
  for (const auto &point_C0F : points0) {
    const auto point_C1F = (T_C1C0 * point_C0F.homogeneous()).head(3);
    const auto p = camera_geometry_project(cam1, point_C1F);
    projected1.emplace_back(p);
  }

  // Draw
  const cv::Scalar red{0, 0, 255};
  const cv::Scalar green{0, 255, 0};
  auto result0 = draw_calib_validation(image0, kps0, projected0, red, green);
  auto result1 = draw_calib_validation(image1, kps1, projected1, red, green);

  // Combine cam0 and cam1 images
  cv::Mat result;
  cv::vconcat(result0, result1, result);
  return result;
}

} //  namespace proto
#endif // PROTOTYPE_CALIB_CALIB_DATA_IMPL_HPP
