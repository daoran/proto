/**
 * @file
 * @ingroup vision
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_DRAW_HPP
#define PROTOTYPE_VISION_FEATURE2D_DRAW_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace prototype {
/**
 * @addtogroup vision
 * @{
 */

/**
  * Draw tracks
  *
  * @param img_cur Current image frame
  * @param p0 Previous corners
  * @param p1 Current corners
  * @param status Corners status
  *
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
  * Draw tracks
  *
  * @param img_cur Current image frame
  * @param p0 Previous corners
  * @param p1 Current corners
  * @param status Corners status
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
  * Draw matches
  *
  * @param img0 Image frame 0
  * @param img1 Image frame 1
  * @param k0 Previous keypoints
  * @param k1 Current keypoints
  * @param matches Feature matches
  *
  * @returns Image with feature matches between frame 0 and 1
  */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status);

/**
  * Draw matches
  *
  * @param img0 Previous image frame
  * @param img1 Current image frame
  * @param k0 Previous keypoints
  * @param k1 Current keypoints
  * @param matches Feature matches
  *
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
 * Draw grid features
 *
  * @param image Image frame
  * @param grid_rows Grid rows
  * @param grid_cols Grid cols
  * @param features List of features
  *
  * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::Point2f> features);

/**
 * Draw grid features
 *
  * @param image Image frame
  * @param grid_rows Grid rows
  * @param grid_cols Grid cols
  * @param features List of features
  *
  * @returns Grid features image
 */
cv::Mat draw_grid_features(const cv::Mat &image,
                           const int grid_rows,
                           const int grid_cols,
                           const std::vector<cv::KeyPoint> features);

/** @} group vision */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_DRAW_HPP
