#ifndef PROTOTYPE_VISION_FEATURE2D_GRID_GOOD_HPP
#define PROTOTYPE_VISION_FEATURE2D_GRID_GOOD_HPP

#include <cstring>
#include <math.h>
#include <vector>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "prototype/core.hpp"
#include "prototype/vision/util.hpp"

namespace prototype {

/**
 * Grid fast
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] quality_level Quality level
 * @param[in] min_distance Min distance
 * @param[in] mask Mask
 * @param[in] block_size Block size
 * @param[in] use_harris_detector Use Harris detector
 * @param[in] k Free parameter for Harris detector
 *
 * @returns List of points
 */
std::vector<cv::Point2f> grid_good(const cv::Mat &image,
                                   const int max_corners = 100,
                                   const int grid_rows = 5,
                                   const int grid_cols = 5,
                                   const double quality_level = 0.01,
                                   const double min_distance = 10,
                                   const cv::Mat mask = cv::Mat(),
                                   const int block_size = 3,
                                   const bool use_harris_detector = false,
                                   const double k = 0.04);

} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_GRID_GOOD_HPP
