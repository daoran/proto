/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_IMPL_HPP
#define PROTOTYPE_CALIB_CALIB_IMPL_HPP

#include "calib.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

template <typename CM, typename DM>
cv::Mat validate_intrinsics(const cv::Mat &image,
                            const std::vector<vec2_t> &keypoints,
                            const std::vector<vec3_t> &points,
                            const camera_geometry_t<CM, DM> &camera_geometry) {
  assert(image.empty() == false);

  // Project points to image plane
  std::vector<vec2_t> projected;
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

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_IMPL_HPP
