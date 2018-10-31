/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_GRID_FAST_HPP
#define PROTOTYPE_VISION_FEATURE2D_GRID_FAST_HPP

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
 * @param threshold Fast threshold
 * @param nonmax_suppression Nonmax Suppression
 * @param debug Debug mode
 *
 * @returns List of keypoints
 */
std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int max_corners = 100,
                                    const int grid_rows = 5,
                                    const int grid_cols = 5,
                                    const double threshold = 10.0,
                                    const bool nonmax_suppression = true);

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_GRID_FAST_HPP
