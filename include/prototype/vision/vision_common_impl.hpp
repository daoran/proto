#ifndef PROTOTYPE_VISION_VISION_HPP
#define PROTOTYPE_VISION_VISION_HPP

#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "prototype/core/core.hpp"
#include "prototype/vision/camera/camera_geometry.hpp"

namespace proto {

// template <typename CAMERA_MODEL, typename DISTORTION_MODEL>
// void essential_matrix_outlier_rejection(
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam0,
//     const camera_geometry_t<CAMERA_MODEL, DISTORTION_MODEL> &cam1,
//     const mat4_t &T_cam1_cam0,
//     const std::vector<cv::Point2f> &cam0_points,
//     const std::vector<cv::Point2f> &cam1_points,
//     const double threshold,
//     std::vector<uchar> &inlier_markers) {
//   // Remove outliers using essential matrix
//   // -- Compute the relative rotation between the cam0 frame and cam1 frame
//   const cv::Matx33d R_cam1_cam0 = convert(T_cam1_cam0.block(0, 0, 3, 3));
//   const cv::Vec3d t_cam0_cam1 = convert(T_cam1_cam0.block(0, 3, 3, 1));
//   // -- Compute the essential matrix
//   const cv::Matx33d E = skew(t_cam0_cam1) * R_cam1_cam0;
//   // -- Calculate norm pixel unit
//   const double cam0_fx = cam0.camera_model.fx;
//   const double cam0_fy = cam0.camera_model.fy;
//   const double cam1_fx = cam1.camera_model.fx;
//   const double cam1_fy = cam1.camera_model.fy;
//   const double norm_pixel_unit = 4.0 / (cam0_fx + cam0_fy + cam1_fx + cam1_fy);
//   // -- Further remove outliers based on essential matrix
//   // std::vector<cv::Point2f> cam0_points_ud = undistort(cam0_points);
//   // std::vector<cv::Point2f> cam1_points_ud = undistort(cam1_points);
//   // std::vector<cv::Point2f> cam0_points_ud =
//   // cam0.undistortPoints(cam0_points); std::vector<cv::Point2f> cam1_points_ud
//   // = cam1.undistortPoints(cam1_points); for (size_t i = 0; i <
//   // cam0_points_ud.size(); i++) {
//   //   if (inlier_markers[i] == 0) {
//   //     continue;
//   //   }
//   //
//   //   const cv::Vec3d pt0(cam0_points_ud[i].x, cam0_points_ud[i].y, 1.0);
//   //   const cv::Vec3d pt1(cam1_points_ud[i].x, cam1_points_ud[i].y, 1.0);
//   //   const cv::Vec3d el = E * pt0;
//   //   double err = fabs((pt1.t() * el)[0]) / sqrt(el[0] * el[0] + el[1] *
//   //   el[1]); if (err > threshold * norm_pixel_unit) {
//   //     inlier_markers[i] = 0;
//   //   }
//   // }
// }

} //  namespace proto
#endif // PROTOTYPE_VISION_VISION_HPP
