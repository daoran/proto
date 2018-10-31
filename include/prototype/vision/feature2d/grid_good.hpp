/**
 * @file
 * @ingroup feature2d
 */
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

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Grid fast
 *
 * @param image Input image
 * @param max_corners Max number of corners
 * @param grid_rows Number of grid rows
 * @param grid_cols Number of grid cols
 * @param quality_level Quality level
 * @param min_distance Min distance
 * @param mask Mask
 * @param block_size Block size
 * @param use_harris_detector Use Harris detector
 * @param k Free parameter for Harris detector
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
                                   const bool useHarrisDetector = false,
                                   const double k = 0.04);

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_GRID_GOOD_HPP
